import time
import logging
import rospy
import dynamic_reconfigure.client
import hr_msgs


class ASR_Word_Stream:
    """This class listens to ASR word stream.
    The class will store listened word and timestamp it sees an word input.
    """
    def __init__(self, args) -> None:
        """Create a subscriber listen to ASR word stream - the instant word from ASR module.

        Args:
            args (Namespace): should contain a "tos_topic" namespace, which is a dict that has 
            key "ASR_word" refer to path of ASR_word_stream
        """
        #rospy.init_node("ASR_Word_Stream")

        # TODO: change to interium stream
        self.word_listener = rospy.Subscriber(
            args["ASRVAD"]["asr_words_topic"],
            hr_msgs.msg.ChatMessage,
            self.callback,
            queue_size=100
        )
        self.word = ""
        self.timestamp = 0
        self.new_word = False
        self.logger = logging.getLogger(__name__)

    def callback(self, msg) -> None:
        self.word = msg.utterance
        self.timestamp = time.time()
        self.new_word = True
        self.logger.debug("%s: '%s' ", self.__class__.__name__, self.word)

    def get_time_stamp(self) :
        output = (self.new_word, self.timestamp)
        self.new_word = False
        return output

    def get_current_sentence(self):
        #  TODO: Confirm if we need a mannual cache clean
        # or a sleep to get the sull asr?
        # time.sleep(1)
        return self.word


class ASR_Sentence_Stream:
    """
    This class is deprecated and left unimplemented. Use ASR_Interim_Sentence instead.
    """    
    def __init__(self) -> None:
        pass



class ASR_Interim_Sentence:
    """
    This class listens to ASR interim sentence.
    """    
    def __init__(self, args) -> None:

        # Subscriber listening to ROS topic
        #rospy.init_node("ASR_Full_Sentence_Node")
        self.full_sentence_listener = rospy.Subscriber(
            args["ASRVAD"]["asr_interim_speech_topic"],
            hr_msgs.msg.ChatMessage,
            self.ASR_sentence_callback,
            queue_size=100
        )
        # Variables to be filled
        self.asr_full_sentence = ""
        self.sentence_format = {
            "lang" : "", "confidence": 0,
            "source" : "", "audio_path": "",
        }
        self.logger = logging.getLogger(__name__)

        self.new_sentence = False
        self.timestamp = 0

    def ros_dynamic_configuration(self, lang="HK"):
        """Dynamic configuration of ASR language settings, 
        choose between Cantonese yue-Hant-HK and English en-GB"""
        #rospy.init_node('myconfig_py', anonymous=True)
        client = dynamic_reconfigure.client.Client('/hr/perception/speech_recognizer')    
        if lang=="EN":
            params = { 'enable': True, 'language':'en-GB'} #'en-GB'
        else:
            params = { 'enable': True, 'language':'yue-Hant-HK'} #'yue-Hant-HK'
        config = client.update_configuration(params)
        return config

    def ASR_sentence_callback(self, msg):
        # Update the asr message storage
        self.asr_full_sentence = msg.utterance
        self.sentence_format["lang"] = msg.lang
        self.sentence_format["confidence"] = msg.confidence
        self.sentence_format["source"] = msg.source
        self.sentence_format["audio_path"] = msg.audio_path


        self.logger.info("%s: '%s' ", self.__class__.__name__, self.asr_full_sentence)

        self.new_sentence = True
        self.timestamp = time.time()


    def get_full_sentence(self):
        if self.new_sentence:
            self.new_sentence = False
            wait = False
            return (wait, self.asr_full_sentence)
        else:
            wait = True
            return (wait, self.asr_full_sentence)

    def get_time_stamp(self):
        output = (self.new_sentence, self.timestamp)
        self.new_sentence = False
        return output

    def get_current_sentence(self):
        start_waiting_time = time.time()
        timeout = False
        while not timeout:
            wait, sentence = self.get_full_sentence()
            if not wait:
                return sentence
            # update timeout
            timeout = (time.time() - start_waiting_time) > 1
        if timeout:
            self.logger.error("%s ASR timeout, return previous sentence '%s'", self.__class__.__name__, sentence)
        return sentence, timeout