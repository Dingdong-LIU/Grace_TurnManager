import time
import grace_attn_msgs.msg
import grace_attn_msgs.srv
from utils.dialogflow_connector import DialogflowConnector
from utils.action_composer import ActionComposer
from modules.turn_module import TurnSegmenter, ThreadWithReturnValue, Turn
import logging
import queue
import rospy
import std_msgs

class ProgressivePolicy:
    """
    This class is the progressive policy module of the Grace Pace Monitor.
    """
    def __init__(self, asr_listener, emotion_listener, config:dict):
        self.chatbot = DialogflowConnector(
            link=config["TM"]["DialogFlow"]["url"],
            api_version=config["TM"]["API_Version"]
        )

        self.action_composer = ActionComposer(
            database_file=config["TM"]["Database"]["path"],
            api_version = config["TM"]["API_Version"],
            config=config
        )

        self.turn_segmenter = TurnSegmenter(
            asr_listener=asr_listener,
            emotion_listener=emotion_listener,
            action_composer= self.action_composer,
            config=config["TM"]
        )

        # self.processing_task = None
        # Initialize a Queue of size 10 for processing turn object
        self.processing_task_pool = queue.Queue(maxsize=10)
        self.revert_task = None # only for debugging purpose
        self.__logger = logging.getLogger(__name__)


        self.handle_barge_in = config["TM"]["Debug"]["enable_barge_in"]
        self.__config = config

        #ad hoc fixes
        self.need_revert = False
        self.pending_robot_speech_finish_after_revert = False
        self.robot_speech_finish_signal_sub = rospy.Subscriber(      
                                        config['Custom']['Behavior']['speech_completion_topic'], 
                                        std_msgs.msg.Bool,
                                        self.__robot_speech_fin_callback,
                                        queue_size=config['Custom']['Ros']['queue_size'])
        # self.__enable_revert_blck = config["TM"]["Debug"]['enable_revert_block']

    def __robot_speech_fin_callback(self,msg):
        if(msg.data):
            #if a speech finished normaliy while pending the speech finish flag
            self.chatbot.consecutive_revert_flag = False

    def set_fake_chatbot(self, use_fake_chatbot):
        self.chatbot.debug_mode(enabled=use_fake_chatbot)
    
    def process_longlong_not_owned_turn(self, turn: Turn):
        self.action_composer.publish_turn_taking_signal()
        self.__logger.warning("Human stop speaking for too long, need gracefully end")
        res = self.chatbot.gracefully_end()
        utterance, params = self.action_composer.parse_reply_from_chatbot(res)
        req = self.action_composer.compose_req(command="comp_exec", utterance=utterance, params=params)
        return req

    def process_human_turn(self, turn:Turn) -> grace_attn_msgs.srv.GraceBehaviorRequest:
        # indicate robot wants to take turn
        self.action_composer.publish_turn_taking_signal()

        # get the engagement level
        engagement_level = turn.get_engagement_level()
        # Gracefully end the conversation if the engagement level is "agitated"
        if engagement_level == "agitated":
            self.__logger.warning("Ask robot to gracefully end due to agitation of patient")
            res = self.chatbot.gracefully_end()
            utterance, params = self.action_composer.parse_reply_from_chatbot(res)
            req = self.action_composer.compose_req(command="comp_exec", utterance=utterance, params=params)
        # If the engagement level is "distracted", then ask the user to repeat
        elif engagement_level == "distracted":
            self.__logger.warning("Ask robot to repeat due to distraction of patient")
            res = self.chatbot.repeat()
            utterance, params = self.action_composer.parse_reply_from_chatbot(res)
            req = self.action_composer.compose_req(command="comp_exec", utterance=utterance, params=params)
        # If the engagement level is "engaged" or "undefined", then process user's utterance
        # communicate with the chatbot and wrap the response into a request
        else:
            # get user's sentence from the turn object
            user_utterance = turn.get_asr() # This function may need some time to execute
            res = self.chatbot.normal_communicate(user_utterance)
            utterance, params = self.action_composer.parse_reply_from_chatbot(res)
            req = self.action_composer.compose_req(command="comp_exec", utterance=utterance, params=params)

        return req



    def applyPolicy(self, state_dict):

        # Handle agitation voting
        # If the agitation is too high, then ask the robot to gracefully end
        agitation = self.turn_segmenter.emotion_listener.vote_agitation(
            agitation_threshold=self.__config["TM"]["Emotion"]['agitation_threshold'],
            agitation_length=self.__config["TM"]["Emotion"]['consecutive_agitation_time']*self.__config["TM"]["Emotion"]['frequency'],
        )
        if agitation:
            self.__logger.warning("Ask robot to gracefully end due to agitation of patient. This is an agitation voting result from emotion recognition module.")
            res = self.chatbot.gracefully_end()
            utterance, params = self.action_composer.parse_reply_from_chatbot(res)
            req = self.action_composer.compose_req(command="comp_exec", utterance=utterance, params=params)
            return req


        # Yield the robot's turn if robot finish talking
        robot_speaking_meta = state_dict['robot_speaking']
        if robot_speaking_meta['val'] == "not_speaking" and robot_speaking_meta["transition"]:
            # Yield robot's turn
            self.action_composer.publish_turn_yielding_signal()
            
        
        # Handle Barge-in: when robot is speaking and patient is speaking
        # Immediately let robot to stop and hand over the turn ownership
        robot_turn = (state_dict["turn_ownership"]["val"] == 'robot_turn')
        human_speaking = (state_dict["human_speaking"]["val"] == "speaking")
        if self.handle_barge_in and robot_turn and human_speaking:
            req = self.action_composer.stop_talking_action()

            # Immediately create a stop processing action for robot
            self.action_composer.publish_turn_yielding_signal()

            self.turn_segmenter.reconstruct_flag = True

            # Discard the currently processing task, if any
            # if not self.processing_task_pool.empty():
            #     task_to_discard = self.processing_task_pool.get(block=False)

            
            # no revert in the first turn
            # no revert if just reverted and no complete speech delivered yet
            if self.turn_segmenter.last_human_turn: 
                # set discard turn timestamp, human turn before this timestamp will be discarded
                self.turn_segmenter.discard_turn = self.turn_segmenter.last_human_turn.create_time
                # Send a revert request to the chatbot, if this is not the first turn
                # self.revert_task = ThreadWithReturnValue(target=self.chatbot.revert_last_turn)
                # self.revert_task.start()
                self.need_revert = True


                #Start to pending a speech finish flag upon first revert
                #even with the post-poned revert system it is safe to sete the flag here already
                # self.pending_robot_speech_finish_after_revert = True
                # self.chatbot.consecutive_revert_flag = True

            return req

        # Check if there is a finished turn processing result
        req = None

        ## Maintain the running task pool

        # Get the oldest task in the queue, but don't remove it
        oldest_task = None if self.processing_task_pool.empty() else self.processing_task_pool.queue[0]
        # Check if this req need to be discarded. If it is, discard it
        # As turn construction is much slower than mainloop. We only process one task per loop.
        if self.need_revert:
            discard_obj = None
            if oldest_task and oldest_task._args[0].create_time <= self.turn_segmenter.discard_turn:
                discard_obj = self.processing_task_pool.get(block=False)
                # empty the queue
                while not self.processing_task_pool.empty():
                    self.processing_task_pool.get(block=False)
            if discard_obj:
                def complex_revert(discard_obj, revert_obj):
                    discard_obj.join()
                    revert_obj()
                #Postpone revert till processing finish
                self.revert_task = ThreadWithReturnValue(target=complex_revert, args=(discard_obj, self.chatbot.revert_last_turn))
            else:
                #Do revert immediately
                self.revert_task = ThreadWithReturnValue(target=self.chatbot.revert_last_turn)
            self.revert_task.start()
            
            self.need_revert = False
            

        oldest_task = None if self.processing_task_pool.empty() else self.processing_task_pool.queue[0]
        if oldest_task and not oldest_task.is_alive():
            # If the task is finished, then get the result
            finished_task = self.processing_task_pool.get(block=False)
            req = finished_task.join()



        # Get the turn object. 
        # If this is exact a transition time, a turn object is created
        # Otherwise None is returned
        turn = self.turn_segmenter.update_turn_information(state_dict)

        # check if there is a too long not owned turn
        if turn and turn.get_ownership() == "longlong_not_owned":
            req = self.process_longlong_not_owned_turn(turn)
            return req


        # if turn object is not none and it is not a reconstructed turn, then there is a turn need to be processed
        if turn and turn.get_ownership() == "human_turn": 
            # Then start a thread to process human turn
            latest_task = ThreadWithReturnValue(target=self.process_human_turn, args=(turn,))
            latest_task.start()
            self.processing_task_pool.put(latest_task, block=1)
        
        return req
    
    def initialize_conversation(self):
        # indicate robot wants to take turn
        self.action_composer.publish_turn_taking_signal()
        # Construct an initial turn object
        initial_turn = Turn(ownership="robot_turn", time_stamp=time.time())
        self.turn_segmenter.last_turn = initial_turn
        self.turn_segmenter.last_turn_ownership = initial_turn.get_ownership()

        # Ask the robot to start conversation
        res = self.chatbot.start_conversation()
        utterance, params = self.action_composer.parse_reply_from_chatbot(res)
        req = self.action_composer.compose_req(command="comp_exec", utterance=utterance, params=params)
        return req
