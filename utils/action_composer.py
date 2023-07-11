import grace_attn_msgs.msg
import grace_attn_msgs.srv

from utils.database_reader import database_reader


class ActionComposer:
    """
    This class is used to compose a action request for Grace Robot
    """
    def __init__(self):
        self.database_reader = database_reader(filename="data/intent_emotion_mapping.xlsx")
        self.req = None
        self.lang = 'yue-Hant-HK'

    def parse_reply_from_chatbot(self, res:dict):
        intent = res["responses"]['intent']
        utterance = res["responses"]['text']
        params = self.database_reader.lookup_table(intent_name=intent)
        return (utterance, params)

    def compose_req(self, command:str, utterance:str, params:dict) -> grace_attn_msgs.srv.GraceBehaviorRequest:
        """Compose a string using params and command

        Args:
            command (str): Command for Grace Robot, can only be 'exec' or 'stop'
            utterance (str): A string for Grace to speak
            params (dict): a set of params

        Returns:
            grace_attn_msgs.srv.GraceBehaviorRequest: a request
        """
        self.req = grace_attn_msgs.srv.GraceBehaviorRequest(**params)
        self.req.utterance = utterance
        self.req.command = command
        self.req.lang = self.lang
        return self.req
