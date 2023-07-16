#general
import yaml
import rospy
import os
import re
import threading
from signal import signal
from signal import SIGINT
import logging
import sys
from datetime import datetime
import time
from inspect import getsourcefile
from os.path import abspath


import dynamic_reconfigure.client
import sensor_msgs.msg
import std_msgs.msg
import hr_msgs.msg
import grace_attn_msgs.msg
import grace_attn_msgs.srv
import hr_msgs.msg
import hr_msgs.cfg
import hr_msgs.srv
import std_msgs



#Respond to exit signal
def handle_sigint(signalnum, frame):
    # terminate
    print('Main interrupted! Exiting.')
    sys.exit()


#Load necessary modules from the instantaneous part
file_path = os.path.dirname(os.path.realpath(getsourcefile(lambda:0)))
sys.path.append(os.path.join(file_path, '..'))
import Grace_Pace_Monitor.grace_instantaneous_state_monitor
import Grace_Instantaneous_Policy.grace_instantaneous_policy
import Grace_Behav_Executor.grace_behav_exec
from CommonConfigs.grace_cfg_loader import loadGraceConfigs
from CommonConfigs.logging import setupLogger

# Load necessary modules from progressive part
# Load the logging configuration
import utils.logging_config
# import the ASR, Visual Class
from utils.asr_connector import ASR_Word_Stream, ASR_Interim_Sentence
from utils.emotion_connector import FE_Connector
# import progressive policy module
from modules.progressive_policy import ProgressivePolicy




class TurnManager:

    def __init__(self, config_data):
        #Miscellaneous
        signal(SIGINT, handle_sigint)

        #Config
        self.__config_data = config_data

        self.__logger = setupLogger(
                    logging.INFO, 
                    logging.INFO, 
                    self.__class__.__name__,
                    os.path.join(file_path,"./logs/log_") + datetime.now().strftime(self.__config_data['Custom']['Logging']['time_format']))
        

        rospy.init_node(self.__config_data['TM']['Ros']['node_name'])
        self.__nh = True

        # # Load sensor config
        # sensor_config_path = os.path.join(
        #     os.path.dirname(os.path.realpath(getsourcefile(lambda:0))), 
        #     "..", 'Grace_Sensor_Interface/config/config.yaml'
        # )
        # self.__sensor_config_data = loadConfig(sensor_config_path)

        #Behavior execution

        # self.__grace_behav_client = rospy.ServiceProxy(
        #                                 self.__config_data['Custom']['Behavior']['grace_behavior_service'], 
        #                                 grace_attn_msgs.srv.GraceBehavior)
        #Yifan note: use the right instance for the right functionality
        self.__gaze_behav_exec = Grace_Behav_Executor.grace_behav_exec.BehavExec(
                                        self.__config_data, False,
                                        Grace_Behav_Executor.grace_behav_exec.ExecFnc.GAZE)
        self.__nod_behav_exec = Grace_Behav_Executor.grace_behav_exec.BehavExec(
                                        self.__config_data, False,
                                        Grace_Behav_Executor.grace_behav_exec.ExecFnc.NOD)
        self.__speak_behav_exec = Grace_Behav_Executor.grace_behav_exec.BehavExec(
                                        self.__config_data, False, 
                                        Grace_Behav_Executor.grace_behav_exec.ExecFnc.SPEECH)
        self.__hum_behav_exec = Grace_Behav_Executor.grace_behav_exec.BehavExec(
                                        self.__config_data, False, 
                                        Grace_Behav_Executor.grace_behav_exec.ExecFnc.HUM)


        #Instantiate critical components of instantaneous parts
        self.__state_monitor_inst = Grace_Pace_Monitor.grace_instantaneous_state_monitor.InstantaneousStateMonitor(self.__config_data, self.__logger, self.__nh)
        self.__policy_instantaneous = Grace_Instantaneous_Policy.grace_instantaneous_policy.InstantaneousPolicy(self.__config_data, self.__logger)
        

        if(self.__config_data['TM']['Debug']['enable_prog_part']):
            #Instantiate critical components of progressive parts
            # self.__state_monitor_prog = None
            # self.__policy_turn = None

            # Start the ASR service
            asr_listener = ASR_Word_Stream(self.__config_data["HR"])
            # Start the vision module
            emotion_listener = FE_Connector(self.__config_data["Custom"])
            # Start the policy
            self.__policy_progressive = ProgressivePolicy(
                asr_listener=asr_listener,
                emotion_listener=emotion_listener,
                config=self.__config_data
            )
            # Set the fake chatbot
            if self.__config_data["TM"]["Debug"].get("fake_chatbot", False):
                self.__policy_progressive.set_fake_chatbot(self.__config_data["TM"]["Debug"]['fake_chatbot'])

    def __initiateDialogue(self):
        #For progressive part, initialize turn and trigger first action

        initial_action = self.__policy_progressive.initialize_conversation()
        # Execute the "start conversation action"
        self.__speak_behav_exec.initiateBehaviorThread(self.__composeBehavReq(
            cmd=initial_action['cmd'], args=initial_action['content'])
        )
        self.__logger.info(initial_action)

    def __applyPolicy(self):
        decisions = {}

        # Apply the instantaneous policy
        state = self.__state_monitor_inst.getState()
        decisions['inst_act'] = self.__policy_instantaneous.applyPolicy(state)

        if(self.__config_data['TM']['Debug']['enable_prog_part']):
            # Apply the progressive policy
            progressive_actions = self.__policy_progressive.applyPolicy(
                state
            )
            if(progressive_actions != None):
                decisions['prog_act'] = progressive_actions


        return decisions

    def __mergeExec(self, decisions):
        #Apply gaze from instantaneous part
        gaze_action = decisions['inst_act']['gaze_action']
        if(gaze_action != None):
            self.__gaze_behav_exec.initiateBehaviorThread(self.__composeBehavReq(gaze_action))
            self.__logger.info('Gaze action: %s' % decisions['inst_act']['gaze_action'])

        #Check if there is speech action from the progressive part
        if('prog_act' in decisions):
            #If there is, apply the speech action from prog part

            #Yifan note: decode the dict output and implement action execution command
            if decisions['prog_act'] is not None:
                progressive_action = decisions['prog_act']
                self.__speak_behav_exec.initiateBehaviorThread(self.__composeBehavReq(
                    cmd=progressive_action['cmd'], args=progressive_action['content'])
                )
                self.__logger.info("Speech: %s" % decisions['prog_act'])

        else:
            #If no action from the progressive side, apply actions from the inst part (if any)
            nodding_action = decisions['inst_act']['bc_action']['nodding']
            if(nodding_action != None):
                self.__nod_behav_exec.initiateBehaviorThread(self.__composeBehavReq(nodding_action))
                self.__logger.info('Nodding: %s' % nodding_action)

            humming_action = decisions['inst_act']['bc_action']['hum']
            if(humming_action != None):
                self.__hum_behav_exec.initiateBehaviorThread(self.__composeBehavReq(humming_action['cmd'],humming_action['content']))
                self.__logger.info('Humming: %s' % humming_action['cmd'])

    def __composeBehavReq(self, cmd, args = None):
        req = grace_attn_msgs.srv.GraceBehaviorRequest()
        
        req.command = cmd

        if(req.command == self.__config_data['BehavExec']['General']['comp_behav_exec_cmd']
            or
           req.command == self.__config_data['BehavExec']['General']['hum_behav_exec_cmd']):

            req.utterance = args['utterance']
            req.lang = self.__config_data['BehavExec']['TTS']['tts_language_code']

            req.expressions = args['expressions']
            req.exp_start = args['exp_start']
            req.exp_end = args['exp_end']
            req.exp_mag = args['exp_mag']

            req.gestures = args['gestures']
            req.ges_start = args['ges_start']
            req.ges_end = args['ges_end']
            req.ges_mag = args['ges_mag']


        return req

    def mainLoop(self):
        #Setup main loop
        rate = rospy.Rate(self.__config_data['TM']['General']['main_freq'])
        it_cnt = 0

        #Initiate dialogue
        if(self.__config_data['TM']['Debug']['enable_prog_part']):
            self.__initiateDialogue()

        #Enter tm main loop
        while True:

            it_cnt = it_cnt + 1
            self.__logger.debug('[Iteration %.6d]' % it_cnt)

            if(it_cnt == 1):
                #Special processing to initialize instantaneous state
                #We initialize after dialogue initiation so that the timestamp in the state is more accurate
                #Note that the instantaneous state monitor assumes that the dialogue starts in robot's turn
                self.__state_monitor_inst.initializeState()
            else:
                #Update instantaneous states
                self.__state_monitor_inst.updateState()

            #Apply policies
            decisions = self.__applyPolicy()

            #Merge and execute actions
            self.__mergeExec(decisions)

            #Sleep by rate
            self.__logger.debug('******************\n')
            rate.sleep()



if __name__ == '__main__':
    grace_config = loadGraceConfigs()
    grace_tm = TurnManager(grace_config)
    grace_tm.mainLoop()