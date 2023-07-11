import grace_attn_msgs.msg
import grace_attn_msgs.srv
from utils.dialogflow_connector import DialogflowConnector
from utils.action_composer import ActionComposer
from modules.turn_module import TurnSegmenter, ThreadWithReturnValue, Turn



class ProgressivePolicy:
    """
    This class is the progressive policy module of the Grace Pace Monitor.
    """
    def __init__(self, asr_listener, emotion_listener):
        self.chatbot = DialogflowConnector()

        self.turn_segmenter = TurnSegmenter(
            asr_listener=asr_listener,
            emotion_listener=emotion_listener
        )

        self.processing_task = None
        self.revert_task = None # only for debugging purpose

        self.action_composer = ActionComposer()
    
    def process_human_turn(self, turn:Turn) -> grace_attn_msgs.srv.GraceBehaviorRequest:
        # get the engagement level
        engagement_level = turn.get_engagement_level()
        # Gracefully end the conversation if the engagement level is "agitated"
        if engagement_level == "agitated":
            res = self.chatbot.gracefully_end()
            utterance, params = self.action_composer.parse_reply_from_chatbot(res)
            req = self.action_composer.compose_req(command="exec", utterance=utterance, params=params)
            return req
        # If the engagement level is "distracted", then ask the user to repeat
        elif engagement_level == "distracted":
            res = self.chatbot.repeat()
            utterance, params = self.action_composer.parse_reply_from_chatbot(res)
            req = self.action_composer.compose_req(command="exec", utterance=utterance, params=params)
            return req
        # If the engagement level is "engaged" or "undefined", then process user's utterance
        # communicate with the chatbot and wrap the response into a request
        else:
            # get user's sentence from the turn object
            user_utterance = turn.get_asr() # This function may need some time to execute

            utterance, params = self.chatbot.communicate(user_utterance)
            req = self.action_composer.compose_req(command="exec", utterance=utterance, params=params)
            return req



    def applyPolicy(self, state_dict):
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
            if not turn.reconstruct_flag:
                # start a thread to process human turn
                self.processing_task = ThreadWithReturnValue(target=self.process_human_turn, args=(turn,))
            else:
                # process a reconstructed human turn
                # First send a revert request to the chatbot
                self.revert_task = ThreadWithReturnValue(target=self.chatbot.revert_last_turn())
                # Then start a thread to process human turn
                self.processing_task = ThreadWithReturnValue(target=self.process_human_turn, args=(turn,))
        else:
            return None