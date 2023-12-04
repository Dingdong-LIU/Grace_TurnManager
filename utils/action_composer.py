import grace_attn_msgs.msg
import grace_attn_msgs.srv
import std_msgs
import rospy

import logging
from utils.database_reader import database_reader
from utils.cantonese_annotator import Cantonese_Annotator


class ActionComposer:
    """
    This class is used to compose a action request for Grace Robot
    """
    def __init__(
            self, database_file:str = "data/intent_emotion_mapping.xlsx",
            api_version = 'amt',
            config:dict = None,
        ):
        self.database_reader = database_reader(filename=database_file)
        self.req = None
        self.lang = config["BehavExec"]["TTS"]["tts_language_code"]
        self.logger = logging.getLogger(__name__)
        self.__config = config
        self.cantonese_annotator = Cantonese_Annotator()

        self.api_version = 'amt' if api_version == 'amt' else 'paf'

        self.turn_action_publisher = rospy.Publisher(
                                        self.__config['Custom']['TM']['turn_action_topic'], 
                                        data_class=std_msgs.msg.String, 
                                        queue_size=config['Custom']['Ros']['queue_size'])

    def get_intent_from_chatbot_reply(self, response):
        return response['responses']['intent'] if self.api_version=='amt' else response['responses']['next_question_id']
    
    def get_utterance_from_chatbot_reply(self, response):
        return response['responses']['text'] if self.api_version=='amt' else response['responses']['next_question_text']

    def parse_reply_from_chatbot(self, res:dict):
        intent = self.get_intent_from_chatbot_reply(res)
        utterance = self.get_utterance_from_chatbot_reply(res)
        # Incase there is no corresponding entry in table
        try:
            params = self.database_reader.lookup_table(intent_name=intent)
            params["intent"] = intent
        except Exception as e:
            self.logger.error(f"Unable to find corresponding action params for intent {intent}", e, exc_info=True)
            params = None

        try:
            annotation = self.cantonese_annotator.lookup_annotation(utterance)
            utterance = annotation
            print("+++++++++++++\n\n\n",annotation, "\n\n\n+++++")
        except Exception as e:
            self.logger.error(f"Unable to find the Cantonese Annotation for the utterance {intent}", e, exc_info=True)
        return (utterance, params)

    def compose_req(self, command:str, utterance:str, params:dict) -> dict:
        """
        Compose a request for Grace Robot

        Args:
            command (str): command to execute, can only be "comp_exec" for example. Detailed definition in the config file cfg_behav_exec.yaml
            utterance (str): A string for Grace to speak. Comming from the chatbot
            params (dict): a set of params. Comming from the database.

        Returns:
            dict: a request dict with "cmd" and "content" field
        """
        # When there is a database lookup error happens. This should be an comp_exec command but without valid utterance or params
        if command == "comp_exec" and (utterance == "" or params is None):
            return None

        if utterance=="" and params is None:
            # when compose a stop action, params is None and utterance is empty
            action_content = None
        else:
            action_content = params
            action_content['utterance'] = utterance
            action_content['lang'] = self.lang
            action_content['end_conversation'] = False
            if utterance == '冇問題, 我明白. 我會搵第個護士嚟幫手.':
                action_content['end_conversation'] = True
            else:
                intent = params.get('intent', None)
                if intent and intent in ["(Q10.Skip) Repeat Address - Reach max limit", "(Q10.Success) Repeat Address", "(Special) Gracefully exit", "(Special) Emergency"]:
                    action_content['end_conversation'] = True
        
        
        req = {
            "cmd": command,
            "content" : action_content,
            "end_conversation": action_content.get("end_conversation", False) if action_content else False
        }
        return req

    # def compose_req(self, command:str, utterance:str, params:dict) -> grace_attn_msgs.srv.GraceBehaviorRequest:
    #     """Compose a string using params and command

    #     Args:
    #         command (str): Command for Grace Robot, can only be 'exec' or 'stop'
    #         utterance (str): A string for Grace to speak
    #         params (dict): a set of params

    #     Returns:
    #         grace_attn_msgs.srv.GraceBehaviorRequest: a request
    #     """
    #     if params:
    #         self.req = grace_attn_msgs.srv.GraceBehaviorRequest(**params)
    #     else:
    #         self.req = grace_attn_msgs.srv.GraceBehaviorRequest()
    #     self.req.utterance = utterance
    #     self.req.command = command
    #     self.req.lang = self.lang
    #     return self.req
    
    def publish_turn_taking_signal(self):
        self.turn_action_publisher.publish(self.__config['InstState']['TurnAction']['robot_take_turn_action_name'])

    def publish_turn_yielding_signal(self):
        self.turn_action_publisher.publish(self.__config['InstState']['TurnAction']['robot_yield_turn_action_name'])
    
    def stop_talking_action(self):
        # self.publish_turn_yielding_signal()
        req = self.compose_req(
            command=self.__config["BehavExec"]["General"]["utterance_behav_stop_cmd"],
            utterance="",
            params=None
        )
        return req