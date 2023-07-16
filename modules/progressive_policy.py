import time
import grace_attn_msgs.msg
import grace_attn_msgs.srv
from utils.dialogflow_connector import DialogflowConnector
from utils.action_composer import ActionComposer
from modules.turn_module import TurnSegmenter, ThreadWithReturnValue, Turn



class ProgressivePolicy:
    """
    This class is the progressive policy module of the Grace Pace Monitor.
    """
    def __init__(self, asr_listener, emotion_listener, config:dict):
        self.chatbot = DialogflowConnector()

        self.action_composer = ActionComposer(
            database_file=config["TM"]["Database"]["path"],
            config=config
        )

        self.turn_segmenter = TurnSegmenter(
            asr_listener=asr_listener,
            emotion_listener=emotion_listener,
            action_composer= self.action_composer
        )

        self.processing_task = None
        self.revert_task = None # only for debugging purpose

        
    
    def set_fake_chatbot(self, use_fake_chatbot):
        self.chatbot.debug_mode(enabled=use_fake_chatbot)
    
    def process_human_turn(self, turn:Turn) -> grace_attn_msgs.srv.GraceBehaviorRequest:
        # indicate robot wants to take turn
        self.action_composer.publish_turn_taking_signal()

        # get the engagement level
        engagement_level = turn.get_engagement_level()
        # Gracefully end the conversation if the engagement level is "agitated"
        if engagement_level == "agitated":
            res = self.chatbot.gracefully_end()
            utterance, params = self.action_composer.parse_reply_from_chatbot(res)
            req = self.action_composer.compose_req(command="comp_exec", utterance=utterance, params=params)
            return req
        # If the engagement level is "distracted", then ask the user to repeat
        elif engagement_level == "distracted":
            res = self.chatbot.repeat()
            utterance, params = self.action_composer.parse_reply_from_chatbot(res)
            req = self.action_composer.compose_req(command="comp_exec", utterance=utterance, params=params)
            return req
        # If the engagement level is "engaged" or "undefined", then process user's utterance
        # communicate with the chatbot and wrap the response into a request
        else:
            # get user's sentence from the turn object
            user_utterance = turn.get_asr() # This function may need some time to execute
            res = self.chatbot.communicate(user_utterance)
            utterance, params = self.action_composer.parse_reply_from_chatbot(res)
            req = self.action_composer.compose_req(command="comp_exec", utterance=utterance, params=params)
            return req



    def applyPolicy(self, state_dict):
        # Yield the robot's turn if robot finish talking
        robot_speaking = state_dict['robot_speaking']['val']
        if robot_speaking == "not_speaking":
            # Yield robot's turn
            self.action_composer.publish_turn_yielding_signal()
            pass
        
        # Handle Barge-in: when robot is speaking and patient is speaking
        # Immediately let robot to stop and hand over the turn ownership
        robot_turn = (state_dict["turn_ownership"]["val"] == 'robot_turn')
        human_speaking = (state_dict["human_speaking"]["val"] == "speaking")
        if robot_turn and human_speaking:
            # Immediately create a stop processing action for robot
            self.action_composer.publish_turn_yielding_signal()
            # Send a revert request to the chatbot
            self.revert_task = ThreadWithReturnValue(target=self.chatbot.revert_last_turn)
            self.revert_task.start()

            req = self.action_composer.stop_talking_action()
            self.turn_segmenter.reconstruct_flag = True
            
            return req

        # Get the turn object. 
        # If this is exact a transition time, a turn object is created
        # Otherwise None is returned
        turn = self.turn_segmenter.update_turn_information(state_dict)
        # if turn object is none, then there is no turn need to be processed, return empty actions
        if turn is None:
            return None
        
        # if a task is being processed, then wait for it to finish - don't accept any turn object at the time
        if self.processing_task is not None:
            if self.processing_task.is_alive():
                return None
            # if the task is finished, then get the result and return it
            else:
                req = self.processing_task.join()
                self.processing_task = None
                return req

        # if turn object is not none and it is not a reconstructed turn, then there is a turn need to be processed
        if turn.get_ownership() == "human_turn": 
            # if not turn.reconstruct_flag:
            #     # start a thread to process human turn
            #     self.processing_task = ThreadWithReturnValue(target=self.process_human_turn, args=(turn,))
            #     self.processing_task.start()
            # else:
            # Then start a thread to process human turn
            self.processing_task = ThreadWithReturnValue(target=self.process_human_turn, args=(turn,))
            self.processing_task.start()
        else:   
            return None
    
    def initialize_conversation(self):
        # indicate robot wants to take turn
        self.action_composer.publish_turn_taking_signal()
        # Construct an initial turn object
        initial_turn = Turn(ownership="robot_turn", time_stamp=time.time())
        self.turn_segmenter.last_turn = initial_turn

        # Ask the robot to start conversation
        res = self.chatbot.start_conversation()
        utterance, params = self.action_composer.parse_reply_from_chatbot(res)
        req = self.action_composer.compose_req(command="comp_exec", utterance=utterance, params=params)
        return req
