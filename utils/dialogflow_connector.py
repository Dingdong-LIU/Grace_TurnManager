import logging
import random
import time
import requests

from modules.turn_module import ThreadWithReturnValue
from utils.shared_data import SharedData

# NGROL_URI = 'https://c0c9-143-89-145-170.ngrok-free.app'

class DialogflowConnector:
    """
    Connect to the Dialogflow chatbot module 
    """
    def __init__(self, link='https://c0c9-143-89-145-170.ngrok-free.app', api_version='paf', lang="yue-Hant-HK", shared_data:SharedData=None, sentiment_analysis_url:str="") -> None:

        self.shared_data = shared_data
        self.sentiment_analysis_url = sentiment_analysis_url

        self.NGROK_LINK = link
        self.logger = logging.getLogger(__name__)

        self.lang = lang
        if lang not in ["yue-Hant-HK", "en-US"]:
            self.logger.error(f"Langaguage code {lang} not recognizable, reset to yue-Hant-HK")
            self.lang = "yue-Hant-HK"

        self.revert_magic_string = "revert previous intent due to barge in"
        # self.start_conversation_magic_string = "INITIALIZE-SESSION" if self.lang == "yue-Hant-HK" else "INITIALIZE-SESSION-ENGLISH\\"
        self.start_conversation_magic_string = "INITIALIZE-SESSION-CANTONESE-GPT-Redo"
        self.gracefully_end_magic_string = "gracefully exit the interaction."
        self.repeat_magic_string = "please repeat"
        self.long_time_no_answer_string = "no response after a long time (say 10 seconds)"

        self.communicate = self.real_communicate
        self.call_count = 0

        # Last utterance intent is used to revert the intent when barge in happens
        # self.last_utterance_intent = ""
        self.consecutive_revert_flag = False

        # API version and conflicts handling
        self.api_endpoint = "dialogue" if api_version.lower() == 'paf' else 'dialogflow_result'

        
        self.session_id = self.get_session_id()
        print(f"+++++++++++++++++++\n\n Session ID for the new Interaction is: {self.session_id}\n\n+++++++++++++++++")

    def get_session_id(self):
        # Return a fixed session ID for the interaction. Handle the duplicated session ID problem.
        # fixed_session_id = 12345678
        fixed_session_id = input("Please enter the session id below: \n")
        # fixed_session_id = int(fixed_session_id)
        self.logger.info("Received session id %s", fixed_session_id)
        r = requests.post(f"{self.NGROK_LINK}/delete_session", json={"session_id": fixed_session_id})
        if r.status_code != 200:
            self.logger.error("Fail to initialize a session. Please refresh the page and try again.")
            # Fallback to random session ID
            random.seed(time.time())
            return random.randint(10000000, 500000000)
        else:
            self.logger.info("Session initialized! " + r.json().get("responses",{}).get("result", {}))
        return fixed_session_id
    
    def sentiment_analysis(self, user_utterance: str):
        with self.shared_data.sentiment_analysis_lock:
            sentiment = requests.post(
                self.sentiment_analysis_url + "/predict",
                json={
                    "conversation": {
                        "ai_question": self.shared_data.previous_question,
                        "user_input": user_utterance,
                    }
                },
            ).json()["output"]
            self.shared_data.write_to_queue(sentiment)
            self.shared_data.sentiment_ready = True

    def debug_mode(self, enabled=False):
        if enabled:
            self.communicate = self.fake_response
        else:
            self.communicate = self.real_communicate
        return

    def test(self):
        print(self.session_id)

    def real_communicate(self, asr_text):            

        # Do additional sentiment analysis here
        ## Do thematic analysis here with a seperate Thread
        sentiment_thread = ThreadWithReturnValue(target=self.sentiment_analysis, args=(asr_text,))
        sentiment_thread.start()

        self.logger.info("Start to communicate with chatbot: %s" ,asr_text)
        # time.sleep(0.1)
        empty_response = {
            "responses" : {
                "intent" : "",
                "text" : "",
                "next_question_id": ""
                }
            }
        if not asr_text or asr_text == "":
            self.logger.error("Attempt to send a empty string to dialogflow, replace it with please repeat.")
            return empty_response
        try:
            response = requests.post(
                f"{self.NGROK_LINK}/{self.api_endpoint}",
                json={
                    "text": asr_text,
                    "session_id": self.session_id,
                    "message_list": asr_text.split("\n"),
                    "redo": len(asr_text.split("\n"))>1,
                },
                timeout=100,
            )
            if response.status_code == 200:
                self.logger.info("Session ID: %s  Received replies from chatbot: %s", str(self.session_id), str(response.json()))
                # self.last_utterance_intent = dict(response.json())["responses"]['intent']
                return response.json()

            # If status code is not 200
            self.logger.warning("Request failed with status code: %d. Returning empty dict", response.status_code)
            return empty_response
        except requests.exceptions.RequestException as err:
            self.logger.error(
                err, 
                "Error in communicating with dialogueflow. Return empty response. url=%s, json=%s", 
                f"{self.NGROK_LINK}/{self.api_endpoint}", 
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
        self.logger.warning("(Fake response) Start to communicate with chatbot: %s" ,asr_text)
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
            response["responses"]["text"] = f"Fake response {self.call_count}."
            # response["responses"]["text"] = f"Fake response {self.call_count}. You must have waited for 1.5 seconds! Count 1 2 3 4 5 6 7 8 9 10."
        # if this is not a magic string then return the fake response sentence
        self.call_count += 1
        self.logger.warn("Received replies from chatbot: %s", str(response))
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
        self.logger.info("Revert previous sentence with magic string: '%s'", self.revert_magic_string)
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

    def long_time_no_answer(self):
        self.logger.info(
            "Tell the chatbot long time no response: %s", self.long_time_no_answer_string
        )
        return self.communicate(asr_text=self.long_time_no_answer_string)

    def repeat(self):
        self.logger.info(
            "Repeat with magic string: %s", self.repeat_magic_string
        )
        return self.communicate(asr_text=self.repeat_magic_string)

    def normal_communicate(self, patient_sentence:str):
        res = self.communicate(patient_sentence)
        # self.consecutive_revert_flag = False
        return res
