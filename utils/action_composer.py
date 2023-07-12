import grace_attn_msgs.msg
import grace_attn_msgs.srv
import logging
from utils.database_reader import database_reader


class ActionComposer:
    """
    This class is used to compose a action request for Grace Robot
    """
    def __init__(self):
        self.database_reader = database_reader(filename="data/intent_emotion_mapping.xlsx")
        self.req = None
        self.lang = 'yue-Hant-HK'
        self.logger = logging.getLogger(__name__)

    def parse_reply_from_chatbot(self, res:dict):
        intent = res["responses"]['intent']
        utterance = res["responses"]['text']
        # Incase there is no corresponding entry in table
        try:
            params = self.database_reader.lookup_table(intent_name=intent)
        except Exception as e:
            self.logger.error("Unable to find corresponding action params for intent %s", intent)
            params = None
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
        if params:
            self.req = grace_attn_msgs.srv.GraceBehaviorRequest(**params)
        else:
            self.req = grace_attn_msgs.srv.GraceBehaviorRequest()
        self.req.utterance = utterance
        self.req.command = command
        self.req.lang = self.lang
        return self.req
