import logging
import random
import time
import requests



class DialogflowConnector:
    """
    Connect to the Dialogflow chatbot module 
    """
    def __init__(self, link='https://1cc3-143-89-145-170.ngrok-free.app') -> None:
        self.NGROK_LINK = link
        random.seed(time.time())
        self.session_id = random.randint(10000000, 500000000)
        self.logger = logging.getLogger(__name__)

        self.revert_magic_string = "revert previous intnt due to barge in"
        self.start_conversation_magic_string = "This is a magic phrase to initialize grace agent to welcome intent."
        self.gracefully_end_magic_string = "gracefully exit the interaction."
        self.repeat_magic_string = "please repeat"


    def test(self):
        print(self.session_id)

    def communicate(self, asr_text):
        self.logger.info("Start to communicate with chatbot: %s" ,asr_text)
        empty_response = {
            "responses" : {
                "intent" : "",
                "text" : ""
                }
            }
        try:
            response = requests.post(
                f"{self.NGROK_LINK}/dialogflow_result",
                json={
                    "text": asr_text,
                    "session_id": self.session_id
                },
                timeout=3,
            )
            if response.status_code == 200:
                self.logger.info("Received replies from chatbot: %s", str(response.json()))
                return response.json()

            # If status code is not 200
            self.logger.warning("Request failed with status code: %d. Returning empty dict", response.status_code)
            return empty_response
        except requests.exceptions.RequestException as err:
            self.logger.error(
                err, 
                "Error in communicating with dialogueflow. Return empty response. url=%s, json=%s", 
                f"{self.NGROK_LINK}/dialogflow_result", 
                str({
                    "text": asr_text,
                    "session_id": self.session_id
                }),
                exc_info=True
            )
            return empty_response

    def fake_response(self, asr_text) -> dict:
        """Generate fake reponse to assist debugging. It will have a 1.5s timeout to simulate the communication latency.

        Args:
            asr_text (str): Received sentence to be submitted to the Dialogflow Chatbot

        Returns:
            dict: reponse.json()
        """
        self.logger.debug("(Fake response) Start to communicate with chatbot: %s" ,asr_text)
        time.sleep(1.5) # sleep to fake the latency
        response = {
            "responses" : {
                "intent" : "(Q0.Success) How are you - Bad",
                "text" : "This is a fake reponse from Grace. You must have waited for 1.5 seconds!"
            }
        }
        self.logger.debug("Received replies from chatbot: %s", str(response))
        return response
    
    # revert API and behavior
        # incomplete sentence --> utterance
        # magic string to revert --> utterance ## also immediate, twice as fast
        # complete sentence --> correct utterance
    # frequent barge-in
        # let go of this question if too much barge-in
    def revert_last_turn(self):
        self.logger.info("Revert previous sentence with magic string: %s", self.revert_magic_string)
        response = self.communicate(asr_text=self.revert_magic_string)
        return response

    def start_conversation(self):
        self.logger.info(
            "Start conversation with magic string: %s", self.start_conversation_magic_string
        )
        response = self.communicate(asr_text=self.start_conversation_magic_string)
        return response

    def gracefully_end(self):
        self.logger.info(
            "Gracefully end the conversation with magic string: %s", self.gracefully_end_magic_string
        )
        return self.communicate(asr_text=self.gracefully_end_magic_string)

    def repeat(self):
        self.logger.info(
            "Repeat with magic string: %s", self.repeat_magic_string
        )
        return self.communicate(asr_text=self.repeat_magic_string)
