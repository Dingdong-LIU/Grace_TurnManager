from collections import Counter
from typing import Union, Optional, Literal
import logging
import time
import threading
from utils.asr_connector import ASR_Interim_Sentence, ASR_Word_Stream
from utils.emotion_connector import FE_Connector
from utils.action_composer import ActionComposer
# import os
# import sys


# PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
# sys.path.append(PROJECT_ROOT)


class ThreadWithReturnValue(threading.Thread):
    """
    A thread class that returns a value when join() is called

    Args:
        threading (_type_): _description_
    """
    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs: Optional[dict]=None, Verbose=None):
        if kwargs is None:
            kwargs = {}
        threading.Thread.__init__(self, group, target, name, args, kwargs)
        self._return = None
    def run(self):
        if self._target is not None:
            self._return = self._target(*self._args, **self._kwargs)
    def join(self, *args):
        threading.Thread.join(self, *args)
        return self._return

class Turn:
    """
    A turn is a combination of ASR input and emotion input.
    """
    def __init__(self, ownership="robot_turn", asr_input_thread=None, emotion_input=None, attention_input=None, exiting_asr = None, reconstruct = False, time_stamp=0):
        # sensor data for extraction
        self.asr_input = None
        self.asr_input_thread = asr_input_thread
        self.emotion_input = emotion_input
        self.attention_input = attention_input
        self.create_time = time_stamp

        # redo construction flag
        self.exiting_asr = exiting_asr
        self.reconstruct_flag = reconstruct

        # property data
        # could be "not_owned", "robot_turn", "human_turn"
        self.ownership = ownership
        # engagement level
        self.engagement_level = "undefined"

    def get_engagement_level(self) -> Literal['undefined', 'agitated', 'distracted', 'engaged']:
        """
        This function will update and return the engagement level of the turn.

        Returns:
            str: engagement level, can be {'engaged', 'distracted', 'agitated', 'undefined'}
        """
        emotion = self.get_emotion()
        attention = self.get_attention()

        if emotion in ["Absence", "EXCEPTION!!", ""] or attention == "":
            # engagement level undefined if this module is turned off
            self.engagement_level = "undefined"
        elif emotion in ["Anger", "Agitation"]:
            self.engagement_level = "agitated"
        elif attention == "False":
            self.engagement_level = "distracted"
        else:
            self.engagement_level = "engaged"

        return self.engagement_level

    def get_attention(self) -> str:
        """
        This function will return the attention of the turn. The attention is the most frequent attention in the attention input.

        Returns:
            str: attention, can be "True" or "False";
        """
        attention = Counter(self.attention_input).most_common(1)[0][0]
        return attention


    def get_emotion(self) -> str:
        """
        This function will return the emotion of the turn. The emotion is the most frequent emotion in the emotion input.

        Returns:
            str: emotion, can be {'Anger', 'Contempt', 'Agitation' ('Disgust', 'Fear'), 'Happiness', 'Neutral', 'Sadness', 'Surprise', "Abscence"(no face), "EXCEPTION!!"(didn't track patient)}
        """
        emotion = Counter(self.emotion_input).most_common(1)[0][0]
        return emotion

    def get_asr(self) -> str:
        """
        This function will return the ASR input of the turn.

        Returns:
            str: ASR input
        """
        asr = self.asr_input_thread.join()
        if self.reconstruct_flag:
            self.asr_input = self.exiting_asr + '\n' + asr
        else:
            self.asr_input = asr
        return self.asr_input

    def get_ownership(self):
        """
        This function will return the ownership of the turn.

        Returns:
            str: could be "not_owned", "robot_turn", "human_turn"
        """
        return self.ownership

    def get_timestamp(self):
        """
        This function will return the timestamp of the turn.

        Returns:
            int: timestamp of create_time
        """
        return self.create_time


class TurnSegmenter:
    """
    This Segmenter segments sensor inputs into turns
    """
    def __init__(
            self, 
            asr_listener: Union[ASR_Word_Stream, ASR_Interim_Sentence], 
            emotion_listener:FE_Connector,
            action_composer: ActionComposer
        ):
        # Class functions
        # self.frequency = frequency
        self.logger = logging.getLogger(__name__)

        # Sensor inputs
        self.asr_listener = asr_listener
        self.emotion_listener = emotion_listener
        # self.VAD = None

        # Robot high level action state - whether robot is doing meanful replies
        self.in_action = False

        # Last turn object, default to available
        self.last_turn = None
        self.last_human_turn = None

        # Last turn ownership, for debug purpose only, can safely ignore
        # could be "not_owned", "robot_turn", "human_turn"
        self.last_turn_ownership = 0

        # Timeout (seconds) for a new turn
        self.timeout = 5

        # Turn action composer
        self.turn_action_composer = action_composer

        # Barge-in flag
        self.reconstruct_flag = False

    def construct_turn(self, turn_ownership:str):
        # TODO: solve the timing issue: if the turn is constructed too early, the asr input will be empty. How to choose between the asr sentence stream and the asr word stream?
        """
        This function will construct a turn object. It will return a turn object, and update the last turn reference. ASR and emotion input will be kept a sequence of words.

        Returns:
            _type_: _description_
        """
        asr_input_thread = self.get_asr_input()
        emotion_input = self.emotion_listener.get_emotion()
        attention_input = self.emotion_listener.get_attention()
        turn = Turn(
            ownership=turn_ownership, time_stamp=time.time(), asr_input_thread=asr_input_thread, emotion_input=emotion_input, attention_input=attention_input
        )
        self.last_turn = turn
        
        if turn_ownership == "human_turn":
            self.last_human_turn = turn
            # start to get human ASR when it is a human turn
            asr_input_thread.start()
        
        self.logger.debug("Consturct a new %s turn: %s", turn_ownership, turn)

        return turn

    def get_asr_input(self) -> ThreadWithReturnValue:
        """
        This function will return a ThreadWithReturnValue object that will get the ASR input. The ThreadWithReturnValue will return the asr input got when the thread.join() is called.

        Returns:
            ThreadWithReturnValue: a thread object that will return the asr input when join() is called
        """
        if isinstance(self.asr_listener, ASR_Word_Stream):
            # Get sentence from word stream
            # This function will immediately return the asr input (from word stream), although the asr input can be empty
            # asr = self.asr_listener.get_current_sentence()
            asr_thread = ThreadWithReturnValue(target=self.asr_listener.get_current_sentence)
        elif isinstance(self.asr_listener, ASR_Interim_Sentence):
            # Get sentence from full sentence stream
            # asr = self.asr_listener.get_asr_full_sentence()
            asr_thread = ThreadWithReturnValue(target=self.asr_listener.get_current_sentence)
        # asr_thread.start()
        return asr_thread

    def redo_human_turn(self, turn_ownership:str):
        """
        This function will redo the last human turn. It will merge the current and last human turn to create a new turn object
        """
        self.logger.info("Redoing human turn, concatenating the current and last human turn")
        # Get the asr input from the last turn
        exiting_asr = self.last_human_turn.get_asr()

        # Construct a new turn object
        asr_input_thread = self.get_asr_input()
        emotion_input = self.emotion_listener.get_emotion()
        emotion_input.extend(self.last_human_turn.get_emotion())
        attention_input = self.emotion_listener.get_attention()
        attention_input.extend(self.last_human_turn.get_attention())
        turn = Turn(
            ownership=turn_ownership, time_stamp=time.time(), asr_input_thread=asr_input_thread, emotion_input=emotion_input, attention_input=attention_input, exiting_asr=exiting_asr, reconstruct=True
        )
        # start wait for ASR input from human
        asr_input_thread.start()

        return turn

    def update_turn_information(self, state_dict):
        """
        This function will update the turn information. It will accumulate sensor information if no turn transition happens, but construct new turn objects if turn transition happens.
        """
        # Debug if turn ownership consistancy is broken
        if self.last_turn and self.last_turn.get_ownership() != self.last_turn_ownership:
            self.logger.info("Get turn update from instant policy, update turn ownership")
            self.last_turn_ownership = self.last_turn.get_ownership()
        

        # Extract current turn ownership from state_dict
        turn_ownership_meta = state_dict["turn_ownership"]
        current_turn_ownership = turn_ownership_meta.get("val", "unknown")
        # Log an error if current turn ownership is unknown
        if current_turn_ownership == "unknown":
            self.logger.error("Current turn ownership is unknown")
            return None

        # This is a bug. Should use the turn_ownership_meta["transition"] instead, because internal state maintainance is not realtime
        # if current_turn_ownership == self.last_turn.get_ownership():
        if turn_ownership_meta.get("transition", False) == True:
            # Do nothing if turn ownership does not change
            # Return None
            return None
        # Construct a new turn object if turn ownership changes

        # Debug info: if there is a transition and the self.last_turn_ownership is not the same as "from" in "turn_ownership"
        if turn_ownership_meta.get("transition", False) and self.last_turn_ownership != turn_ownership_meta.get('from', 'unknown'):
        # if turn_ownership_meta.get('transition', False):
        # TODO: check self.last_turn == None
            self.logger.error(
                "last_turn_ownership: '%s' is not the same as turn transition info '%s'",
                self.last_turn_ownership, turn_ownership_meta.get('from', 'unknown')
            )

        
        # Turn object is constructed at the end of a turn, so the ownership is last turn ownership
        last_turn_ownership = turn_ownership_meta.get("from", "unknown")

        # Consider a mis-segmentation if the time span is too short and now it is a human's turn
        if self.last_human_turn and current_turn_ownership == "human_turn" and time.time() - self.last_turn.get_timestamp() < self.timeout:
            # Mis-segmentation happens
            # # Signal the robot to stop all actions, if any
            # self.turn_action_composer.publish_stop_talking_action()
            # Indicate there is a need to redo the last human turn
            # new_turn_object = self.redo_turn(turn_ownership=current_turn_ownership)
            self.reconstruct_flag = True
            self.logger.warn("Potential mis-segmentation as time span is too short and now it is a human's turn")
        
        # Consider a reconstruct when there is a barge in
        if self.reconstruct_flag:
            # For the first human turn, do not redo the turn
            if self.last_human_turn is None:
                new_turn_object = self.construct_turn(turn_ownership=last_turn_ownership)
                self.logger.warn("Tried to revert the first Human turn. Discard this action and construct a normal turn")
            else:
                # Only redo turn when there is previous human turn
                new_turn_object = self.redo_human_turn(turn_ownership=last_turn_ownership)
                self.reconstruct_flag = False
        else:
            new_turn_object = self.construct_turn(turn_ownership=last_turn_ownership)
        return new_turn_object
