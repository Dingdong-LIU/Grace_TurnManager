from utils.dialogflow_connector import DialogflowConnector
from utils import logging_config
from utils.action_composer import ActionComposer


if __name__ == "__main__":
    chatbot = DialogflowConnector(link="https://1cc3-143-89-145-170.ngrok-free.app")
    action_composer = ActionComposer()

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
        else:
            res = chatbot.communicate(sentence)
        utterance, params = action_composer.parse_reply_from_chatbot(res)
        req = action_composer.compose_req(command="exec", utterance=utterance, params=params)
        print(req)