import logging
import os
import datetime

from utils.dialogflow_connector import DialogflowConnector
from utils import logging_config
from utils.action_composer import ActionComposer

import sys
sys.path.append("..")

from CommonConfigs.grace_cfg_loader import loadGraceConfigs
from CommonConfigs.logging import setupLogger

if __name__ == "__main__":
    l = setupLogger(logging.DEBUG, 
                    logging.INFO, 
                    __name__,
                    os.path.join("./logs/log_") + datetime.datetime.now().strftime("%d_%m_%Y_%H_%M_%S"))
    l.info("Start test_dialogflow.py")
    chatbot = DialogflowConnector(link="https://68c7-143-89-145-170.ngrok-free.app")
    chatbot.debug_mode(enabled=False)
    action_composer = ActionComposer(config=loadGraceConfigs())

    # start the conversation
    res = chatbot.start_conversation()
    utterance, params = action_composer.parse_reply_from_chatbot(res)
    req = action_composer.compose_req(command="exec", utterance=utterance, params=params)
    print(req)

    while True:
        sentence = input("Input for chatbot: ")
        if sentence == "end":
            res = chatbot.gracefully_end()
        elif sentence == "repeat":
            res = chatbot.repeat()
        elif sentence == "revert":
            res = chatbot.revert_last_turn()
            continue
        elif sentence == "exit":
            break
        else:
            res = chatbot.communicate(sentence)
        utterance, params = action_composer.parse_reply_from_chatbot(res)
        req = action_composer.compose_req(command="exec", utterance=utterance, params=params)
        print(req)
        if req["content"]["utterance"] == '冇問題, 我明白. 我會搵第個護士嚟幫手.':
            exit(0)