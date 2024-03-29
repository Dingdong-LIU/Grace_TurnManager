from collections import deque, Counter
import rospy
import grace_attn_msgs

class FE_Connector:
    """
    This class is a ROS subscriber to the emotion recognition topic. It will store the emotion and attention data in a deque.
    """
    def __init__(self, subscriber_configs:dict, emotion_configs:dict) -> None:
        #rospy.init_node("emotion_recognition_node")
        self.emotion_sub = rospy.Subscriber(
            subscriber_configs["Perception"]["topic_emotion_attention_state_estimation"],
            grace_attn_msgs.msg.EmotionAttentionResult,
            callback=self.callback,
            queue_size=100
        )
        self.debug_queue_length = emotion_configs["Emotion"]["frequency"]*emotion_configs["Emotion"]["time_window"]
        self.attention = deque([""]*self.debug_queue_length, maxlen=self.debug_queue_length)
        self.emotion = deque([""]*self.debug_queue_length, maxlen=self.debug_queue_length)
        self.visualization_frame = None # This one is of no use, so not updated
        self.frequency = emotion_configs["Emotion"]["frequency"]

    def callback(self, msg):
        # Do with emotion messages here
        self.attention.appendleft(msg.attention.data)
        self.emotion.appendleft(msg.emotion.data)
        # self.visualization_frame = msg.visualization_frame
        # need to send to mainloop to trigger something.

    def get_signal_state(self, signal_name):
        """Get the singal value from emotion recognition subscriber. This method perform a voting of the most common signals within 10 samples

        Args:
            signal_name (str): either "attention" or "emotion"

        Returns:
            str: The most common signal in 10 most recent calls. "attention" could be "True" or "False"; "emotion" could be {'Anger', 'Contempt', 'Agitation' ('Disgust', 'Fear'), 'Happiness', 'Neutral', 'Sadness', 'Surprise', "Abscence"(no face), "EXCEPTION!!"(didn't track patient)}
        """
        assert signal_name in ["attention", "emotion"], f"{signal_name} is not supported"
        queue_object = None
        if signal_name == "attention":
            queue_object = self.attention
        elif signal_name == "emotion":
            queue_object = self.emotion
        occurence_count = Counter(queue_object)
        attn = occurence_count.most_common(1)
        if attn != "":
            return attn[0][0]
        else:
            attn = occurence_count.most_common(2)
            return attn[1][0]

    def get_attention(self):
        """Get the attention signal from emotion recognition subscriber

        Returns:
            deque[str]: A deque of attention signal. "True" or "False"
        """
        return self.attention

    def get_emotion(self):
        """Get the emotion signal from emotion recognition subscriber

        Returns:
            deque[str]: A deque of emotion signal. {'Anger', 'Contempt', 'Agitation' ('Disgust', 'Fear'), 'Happiness', 'Neutral', 'Sadness', 'Surprise', "Abscence"(no face), "EXCEPTION!!"(didn't track patient)}
        """
        return self.emotion
    
    def vote_agitation(self, agitation_threshold=3, agitation_length=30, enabled=True):
        """Vote the agitation signal from emotion recognition subscriber. Using two criterion:
        1. If "Agitation" exceeds a limit, then return Ture
        2. If there is a consecutive "Agitation" signal for a certain length, then return True
        """
        if not enabled:
            return False
        # check criterion 1
        if (self.emotion.count("Agitation") + self.emotion.count('Anger')) > agitation_threshold * self.frequency:
            return True
        # check criterion 2
        consceutive_agitation = 0
        for i, emotion in enumerate(self.emotion):
            if emotion == "Agitation" or emotion == "Anger":
                consceutive_agitation += 1
            else:
                consceutive_agitation = 0
            if consceutive_agitation > agitation_length:
                return True
        return False
