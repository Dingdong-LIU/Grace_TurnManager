# general
import logging
import os
import sys
import time
from datetime import datetime
from inspect import getsourcefile
from signal import SIGINT, signal


# Respond to exit signal
def handle_sigint(signalnum, frame):
    # terminate
    print("Main interrupted! Exiting.")
    sys.exit()


# Load necessary modules from the instantaneous part
file_path = os.path.dirname(os.path.realpath(getsourcefile(lambda: 0)))
sys.path.append(os.path.join(file_path, ".."))
import grace_attn_msgs.msg
import grace_attn_msgs.srv
import Grace_Behav_Executor.DisengageExec
import Grace_Behav_Executor.grace_behav_exec
import Grace_Behav_Executor.utils.TTSExec
import Grace_Instantaneous_Policy.grace_instantaneous_policy
import Grace_Pace_Monitor.grace_instantaneous_state_monitor
import rospy
import std_msgs
import std_msgs.msg
from CommonConfigs.grace_cfg_loader import loadGraceConfigs
from CommonConfigs.logging import setupLogger

# import progressive policy module
from modules.progressive_policy import ProgressivePolicy

# Load necessary modules from progressive part
# Load the logging configuration
# import the ASR, Visual Class
from utils.asr_connector import ASR_Interim_Sentence
from utils.emotion_connector import FE_Connector
from utils.shared_data import SharedData

def killSelf():
    pid = os.getpid()
    os.kill(pid, SIGINT)


class TurnManager:
    def __init__(self, config_data):
        # Miscellaneous
        signal(SIGINT, handle_sigint)

        # Shared data between threads
        self.shared_data = SharedData()

        # Config
        self.__config_data = config_data

        self.__logger = setupLogger(
            logging.DEBUG,
            logging.INFO,
            self.__class__.__name__,
            os.path.join(file_path, "./logs/log_")
            + datetime.now().strftime(
                self.__config_data["Custom"]["Logging"]["time_format"]
            ),
        )

        rospy.init_node(self.__config_data["TM"]["Ros"]["node_name"])
        self.__nh = True

        self.__conv_alive_flag = not self.__config_data["TM"]["Debug"]["has_gui"]
        self.__start_conv_sub = rospy.Subscriber(
            self.__config_data["Custom"]["Flow"]["topic_start_conv"],
            std_msgs.msg.Bool,
            self.__startConvCallback,
            queue_size=self.__config_data["Custom"]["Ros"]["queue_size"],
        )

        self.__end_of_conv_sub = rospy.Subscriber(
            self.__config_data["Custom"]["Flow"]["topic_stop_all"],
            std_msgs.msg.Bool,
            self.__endOfConvCallback,
            queue_size=self.__config_data["Custom"]["Ros"]["queue_size"],
        )

        # # Load sensor config
        # sensor_config_path = os.path.join(
        #     os.path.dirname(os.path.realpath(getsourcefile(lambda:0))),
        #     "..", 'Grace_Sensor_Interface/config/config.yaml'
        # )
        # self.__sensor_config_data = loadConfig(sensor_config_path)

        # Behavior execution

        # self.__grace_behav_client = rospy.ServiceProxy(
        #                                 self.__config_data['Custom']['Behavior']['grace_behavior_service'],
        #                                 grace_attn_msgs.srv.GraceBehavior)
        self.__gaze_behav_exec = Grace_Behav_Executor.grace_behav_exec.BehavExec(
            self.__config_data,
            False,
            Grace_Behav_Executor.grace_behav_exec.ExecFnc.GAZE,
        )
        self.__nod_behav_exec = Grace_Behav_Executor.grace_behav_exec.BehavExec(
            self.__config_data, False, Grace_Behav_Executor.grace_behav_exec.ExecFnc.NOD
        )
        self.__speak_behav_exec = Grace_Behav_Executor.grace_behav_exec.BehavExec(
            self.__config_data,
            False,
            Grace_Behav_Executor.grace_behav_exec.ExecFnc.SPEECH,
        )
        self.__hum_behav_exec = Grace_Behav_Executor.grace_behav_exec.BehavExec(
            self.__config_data, False, Grace_Behav_Executor.grace_behav_exec.ExecFnc.HUM
        )

        self.__rotation_pub = rospy.Publisher(
            self.__config_data["Custom"]["Behavior"]["rotation_motion_topic"],
            std_msgs.msg.Float32,
            queue_size=self.__config_data["Custom"]["Ros"]["queue_size"],
        )

        # Instantiate critical components of instantaneous parts
        self.__state_monitor_inst = Grace_Pace_Monitor.grace_instantaneous_state_monitor.InstantaneousStateMonitor(
            self.__config_data, self.__logger, self.__nh
        )
        self.__policy_instantaneous = (
            Grace_Instantaneous_Policy.grace_instantaneous_policy.InstantaneousPolicy(
                self.__config_data, self.__logger, shared_data=self.shared_data
            )
        )

        if self.__config_data["TM"]["Debug"]["enable_prog_part"]:
            # Instantiate critical components of progressive parts
            # self.__state_monitor_prog = None
            # self.__policy_turn = None

            # Start the ASR service
            # asr_listener = ASR_Word_Stream(self.__config_data["HR"])
            asr_listener = ASR_Interim_Sentence(self.__config_data["HR"])
            # Start the vision module
            emotion_listener = FE_Connector(
                subscriber_configs=self.__config_data["Custom"],
                emotion_configs=self.__config_data["TM"],
            )
            # Start the policy
            self.__policy_progressive = ProgressivePolicy(
                asr_listener=asr_listener,
                emotion_listener=emotion_listener,
                config=self.__config_data,
                shared_data=self.shared_data
            )
            # Set the fake chatbot
            if self.__config_data["TM"]["Debug"].get("fake_chatbot", False):
                self.__policy_progressive.set_fake_chatbot(
                    self.__config_data["TM"]["Debug"]["fake_chatbot"]
                )

        if self.__config_data["TM"]["Debug"]["cache_tts"]:
            # Run through all the tts sentences to cache them locally and generate audio files for proof-hearing
            from utils.mannual_cache import read_cantonese_translation_json

            # Get a list of utterance from the annotated json file
            annotation_dict = read_cantonese_translation_json()
            # Publisher for tts request
            tmp_tts_exec = Grace_Behav_Executor.utils.TTSExec.TTSExec(
                self.__config_data, self.__logger
            )
            # Subscriber for tts file
            tts_path_subscriber = rospy.Subscriber(
                "/hr/control/speech/tts_stream",
                std_msgs.msg.String,
                self.ttsPathCallback,
                queue_size=100,
            )
            # Loop over all annotations
            audio_path = "../AudioFiles/Annotated"
            self.generated_audio_file_path = None
            for utterance_key in annotation_dict:
                self.generateAudio(
                    annotation_dict[utterance_key],
                    audio_path,
                    utterance_key,
                    tmp_tts_exec,
                )
                time.sleep(1.5)

    # For TTS cache
    def ttsPathCallback(self, msg):
        self.generated_audio_file_path = msg.data

    def generateAudio(self, text_in, file_dir, raw_text, tmp_tts_exec):
        import urllib

        host_address = "http://192.168.99.10:8000"
        rate = rospy.Rate(10)
        # Compose a behavior request object and publish for execution
        tmp_tts_exec.say(
            text_in, self.__config_data["BehavExec"]["TTS"]["cantonese_language_code"]
        )
        # Retrieve the audio files
        while True:
            if self.generated_audio_file_path is not None:
                print("Raw text is %s." % (raw_text))
                break
            rate.sleep()
        # Copy the audio file
        remote_url = host_address + self.generated_audio_file_path
        file_path = os.path.join(file_dir, raw_text + ".wav")
        urllib.request.urlretrieve(remote_url, file_path)
        self.generated_audio_file_path = None

    def __physicalDisengageRoutine(self):
        # Call the physical disengagement service in a separate thread
        if self.__config_data["TM"]["Debug"]["enable_physical_disengage"]:
            self.__rotation_pub.publish(
                self.__config_data["BehavExec"]["General"]["disengage_ang"]
            )

    def __startConvCallback(self, msg):
        self.__conv_alive_flag = True

    def __endOfConvCallback(self, msg=None):
        # Call the physical disengagement service in a separate thread
        if self.__config_data["TM"]["Debug"]["enable_physical_disengage"]:
            self.__physicalDisengageRoutine()

        # Say disengage word
        self.__speak_behav_exec.initiateBehaviorThread(
            self.__composeBehavReq(
                cmd=self.__config_data["BehavExec"]["General"]["comp_behav_exec_cmd"],
                args=self.__config_data["BehavExec"]["CompositeBehavior"]["Predefined"][
                    "EndOfConv"
                ],
            ),
            end_of_conv=True,
        )
        killSelf()

    def __initiateDialogue(self):
        # For progressive part, initialize turn and trigger first action

        initial_action = self.__policy_progressive.initialize_conversation()
        # Execute the "start conversation action"
        self.__speak_behav_exec.initiateBehaviorThread(
            self.__composeBehavReq(
                cmd=initial_action["cmd"], args=initial_action["content"]
            )
        )
        self.__logger.info(initial_action)

    def __applyPolicy(self):
        decisions = {}

        # Apply the instantaneous policy
        state = self.__state_monitor_inst.getState()
        decisions["inst_act"] = self.__policy_instantaneous.applyPolicy(state)

        if self.__config_data["TM"]["Debug"]["enable_prog_part"]:
            # Apply the progressive policy
            progressive_actions = self.__policy_progressive.applyPolicy(state)
            if progressive_actions is not None:
                decisions["prog_act"] = progressive_actions

                # Update asked question here
                self.shared_data.change_previous_question(progressive_actions["content"]["utterance"])
                self.shared_data.sentiment_ready = False

        return decisions

    def __mergeExec(self, decisions):
        # Apply gaze from instantaneous part
        gaze_action = decisions["inst_act"]["gaze_action"]
        if gaze_action is not None:
            self.__gaze_behav_exec.initiateBehaviorThread(
                self.__composeBehavReq(gaze_action)
            )
            self.__logger.info("Gaze action: %s" % decisions["inst_act"]["gaze_action"])

        # Check if there is speech action from the progressive part
        if "prog_act" in decisions:
            # If there is, apply the speech action from prog part

            if decisions["prog_act"] is not None:
                progressive_action = decisions["prog_act"]
                # First check if there is a "end_conversation" flag, if so, disable barge-in
                # So that the disengage utterance won't be barged in
                if (
                    "end_conversation" in progressive_action
                    and progressive_action["end_conversation"]
                ):
                    self.__policy_progressive.handle_barge_in = False

                self.__logger.info(progressive_action)
                self.__speak_behav_exec.initiateBehaviorThread(
                    self.__composeBehavReq(
                        cmd=progressive_action["cmd"],
                        args=progressive_action["content"],
                    ),
                    end_of_conv=(
                        "end_conversation" in progressive_action
                        and progressive_action["end_conversation"]
                    ),
                )
                self.__logger.info("Speech: %s" % decisions["prog_act"])

                if ("end_conversation" in progressive_action) and progressive_action[
                    "end_conversation"
                ]:
                    # Rotation
                    self.__physicalDisengageRoutine()
                    time.sleep(
                        self.__config_data["BehavExec"]["General"]["rotation_sleep"]
                    )

                    # Post rotation remarks
                    if "after_rotation" in progressive_action["content"]:
                        progressive_action["content"]["utterance"] = progressive_action[
                            "content"
                        ]["after_rotation"]
                        self.__logger.info("Call post rotation remarks")
                        self.__speak_behav_exec.initiateBehaviorThread(
                            self.__composeBehavReq(
                                cmd=progressive_action["cmd"],
                                args=progressive_action["content"],
                            ),
                            end_of_conv=(
                                "end_conversation" in progressive_action
                                and progressive_action["end_conversation"]
                            ),
                        )

                    killSelf()

        elif decisions["inst_act"]["bc_action"] is not None:
            # If no action from the progressive side, apply actions from the inst part (if any)
            nodding_action = decisions["inst_act"]["bc_action"]["nodding"]
            if nodding_action is not None:
                self.__nod_behav_exec.initiateBehaviorThread(
                    self.__composeBehavReq(
                        nodding_action["cmd"], args=nodding_action["args"]
                    )
                )
                self.__logger.info("Nodding: %s" % nodding_action["cmd"])

            humming_action = decisions["inst_act"]["bc_action"]["hum"]
            if humming_action is not None:
                self.__hum_behav_exec.initiateBehaviorThread(
                    self.__composeBehavReq(
                        humming_action["cmd"], humming_action["content"]
                    )
                )
                self.__logger.info("Humming: %s" % humming_action["cmd"])

        else:
            self.__logger.debug("No action to take.")

    def __composeBehavReq(self, cmd, args=None):
        req = grace_attn_msgs.srv.GraceBehaviorRequest()

        req.command = cmd
        if req.command == self.__config_data["BehavExec"]["General"]["nod_cmd"]:
            req.utterance = args["nod_mode"]
        elif (
            req.command
            == self.__config_data["BehavExec"]["General"]["comp_behav_exec_cmd"]
            or req.command
            == self.__config_data["BehavExec"]["General"]["hum_behav_exec_cmd"]
        ):
            req.utterance = args["utterance"]
            req.lang = self.__config_data["BehavExec"]["TTS"]["tts_language_code"]

            req.expressions = args["expressions"]
            req.exp_start = args["exp_start"]
            req.exp_end = args["exp_end"]
            req.exp_mag = args["exp_mag"]

            req.gestures = args["gestures"]
            req.ges_start = args["ges_start"]
            req.ges_end = args["ges_end"]
            req.ges_mag = args["ges_mag"]

        return req

    def mainLoop(self):
        # Setup main loop
        rate = rospy.Rate(self.__config_data["TM"]["General"]["main_freq"])
        it_cnt = 0

        # Wait for trigger
        while not self.__conv_alive_flag:
            rate.sleep()

        # Initiate dialogue
        if self.__config_data["TM"]["Debug"]["enable_prog_part"]:
            self.__initiateDialogue()

        # Enter tm main loop
        while self.__conv_alive_flag:
            it_cnt = it_cnt + 1
            self.__logger.debug("[Iteration %.6d]" % it_cnt)

            if it_cnt == 1:
                # Special processing to initialize instantaneous state
                # We initialize after dialogue initiation so that the timestamp in the state is more accurate
                # Note that the instantaneous state monitor assumes that the dialogue starts in robot's turn
                self.__state_monitor_inst.initializeState()
                self.__state_monitor_inst.initVAD()
            else:
                # Update instantaneous states
                self.__state_monitor_inst.updateState()

            # #debug
            # self.__logger.info("Feed %s %s" % (self.__state_monitor_inst.getState()['human_speaking']['val'],self.__state_monitor_inst.getState()['turn_ownership']['val']))

            # Apply policies
            decisions = self.__applyPolicy()

            # Merge and execute actions
            self.__mergeExec(decisions)

            # Sleep by rate
            self.__logger.debug("******************\n")
            rate.sleep()


if __name__ == "__main__":
    grace_config = loadGraceConfigs()
    grace_tm = TurnManager(grace_config)
    grace_tm.mainLoop()
