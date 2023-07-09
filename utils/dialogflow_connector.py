import logging
import random
import time
import requests


NGROK_LINK = 'https://grace-dialogue-module.ngrok.app'
class DialogflowConnector:
    def __init__(self) -> None:
        self.session_id = random.randint(10000000, 500000000)
        self.logger = logging.getLogger(__name__)

    def test(self):
        print(self.session_id)

    def communicate(self, asr_text):
        self.logger.debug("Start to communicate with chatbot: %s" ,asr_text, exe_info=True)
        response = requests.post(
            f"{NGROK_LINK}/dialogflow_result",
            json={
                "text": asr_text,
                "session_id": self.session_id
            },
            timeout=3,
        )
        self.logger.debug("Received replies from chatbot: %s", str(response.json()))
        return response.json()

    def fake_response(self, asr_text) -> dict:
        """Generate fake reponse to assist debugging. It will have a 1.5s timeout to simulate the communication latency.

        Args:
            asr_text (str): Received sentence to be submitted to the Dialogflow Chatbot

        Returns:
            dict: reponse.json()
        """
        self.logger.debug("(Fake response) Start to communicate with chatbot: %s" ,asr_text, exe_info=True)
        time.sleep(1.5) # sleep to fake the latency
        response = {
            "responses" : {
                "intent" : "(Q0.Success) How are you - Bad",
                "text" : "This is a fake reponse from Grace. You must have waited for 1.5 seconds!"
            }
        }
        self.logger.debug("Received replies from chatbot: %s", str(response))
        return response
