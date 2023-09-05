import logging
import random
import time
import requests

# NGROL_URI = 'https://c0c9-143-89-145-170.ngrok-free.app'

class DialogflowConnector:
    """
    Connect to the Dialogflow chatbot module 
    """
    def __init__(self, link='https://c0c9-143-89-145-170.ngrok-free.app') -> None:
        self.NGROK_LINK = link
        random.seed(time.time())
        self.session_id = random.randint(10000000, 500000000)
        self.logger = logging.getLogger(__name__)

        self.revert_magic_string = "revert previous intent due to barge in"
        self.start_conversation_magic_string = "This is a magic phrase to initialize grace agent to welcome intent."
        self.gracefully_end_magic_string = "gracefully exit the interaction."
        self.repeat_magic_string = "please repeat"

        self.communicate = self.real_communicate
        self.call_count = 0

        # Last utterance intent is used to revert the intent when barge in happens
        self.last_utterance_intent = ""
        self.consecutive_revert_flag = False

    def debug_mode(self, enabled=False):
        if enabled:
            self.communicate = self.fake_response
        else:
            self.communicate = self.real_communicate
        return

    def test(self):
        print(self.session_id)

    def real_communicate(self, asr_text):            
        self.logger.info("Start to communicate with chatbot: %s" ,asr_text)
        empty_response = {
            "responses" : {
                "intent" : "",
                "text" : ""
                }
            }
        if not asr_text or asr_text == "":
            self.logger.error("Attempt to send a empty string to dialogflow, replace it with please repeat.")
            return empty_response
        try:
            response = requests.post(
                f"{self.NGROK_LINK}/dialogflow_result",
                json={
                    "text": asr_text,
                    "session_id": self.session_id,
                    "message_list": asr_text.split("\n"),
                    "redo": len(asr_text.split("\n"))>1,
                },
            )
            if response.status_code == 200:
                self.logger.info("Received replies from chatbot: %s", str(response.json()))
                self.last_utterance_intent = dict(response.json())["responses"]['intent']
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

    def fake_response(self, asr_text, fake_latency=1.2) -> dict:
        """Generate fake reponse to assist debugging. It will have a fake latency to simulate the communication latency.

        Args:
            asr_text (str): Received sentence to be submitted to the Dialogflow Chatbot
            fake_latency (float, optional): Fake latency to simulate the communication latency. Defaults to 1.2 seconds.

        Returns:
            dict: reponse.json()
        """
        self.logger.info("(Fake response) Start to communicate with chatbot: %s" ,asr_text)
        time.sleep(fake_latency) # sleep to fake the latency

        response = {
            "responses" : {
                "intent" : "(Q0.Success) How are you - Bad",
            }
        }
        if asr_text == self.revert_magic_string:
            return {
                "responses" : {
                    "intent" : self.revert_magic_string,
                    "text" : self.revert_magic_string,
                    }
            }
        
        elif asr_text in [
            self.repeat_magic_string, self.gracefully_end_magic_string,
            self.start_conversation_magic_string]:
            response["responses"]["text"] = asr_text
        else:
            response["responses"]["text"] = f"Fake response {self.call_count}. You must have waited for 1.5 seconds! Count 1 2 3 4 5 6 7 8 9 10."
        # if this is not a magic string then return the fake response sentence
        self.call_count += 1
        self.logger.info("Received replies from chatbot: %s", str(response))
        return response
    
    # revert API and behavior
        # incomplete sentence --> utterance
        # magic string to revert --> utterance ## also immediate, twice as fast
        # complete sentence --> correct utterance
    # frequent barge-in
        # let go of this question if too much barge-in
    def revert_last_turn(self):
        if self.consecutive_revert_flag:
            self.logger.info('REVERT COMMAND SKIPPED')
            return
        self.consecutive_revert_flag = True
        self.logger.info("Revert previous sentence with magic string and Intent: '%s', '%s'", self.revert_magic_string, self.last_utterance_intent)
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

    def normal_communicate(self, patient_sentence:str):
        res = self.communicate(patient_sentence)
        # self.consecutive_revert_flag = False
        return res