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
from utils.asr_connector import ASR_Word_Stream, ASR_Full_Sentence
from utils.emotion_connector import FE_Connector
# import progressive policy module
from modules.progressive_policy import ProgressivePolicy




class TurnManager:

    def __init__(self, config_data):
        #Miscellaneous
        signal(SIGINT, handle_sigint)
        self.__logger = setupLogger(
                    logging.DEBUG, 
                    logging.INFO, 
                    self.__class__.__name__,
                    "./logs/log_" + datetime.now().strftime("%a_%d_%b_%Y_%I_%M_%S_%p"))
        
        self.__config_data = config_data

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
        self.__gaze_behav_exec = Grace_Behav_Executor.grace_behav_exec.BehavExec(self.__config_data, False)
        self.__nod_behav_exec = Grace_Behav_Executor.grace_behav_exec.BehavExec(self.__config_data, False)
        self.__speak_behav_exec = Grace_Behav_Executor.grace_behav_exec.BehavExec(self.__config_data, False)
        self.__hum_behav_exec = Grace_Behav_Executor.grace_behav_exec.BehavExec(self.__config_data, False)


        #Instantiate critical components of instantaneous parts
        self.__state_monitor_inst = Grace_Pace_Monitor.grace_instantaneous_state_monitor.InstantaneousStateMonitor(self.__config_data, self.__logger, self.__nh)
        self.__policy_instantaneous = Grace_Instantaneous_Policy.grace_instantaneous_policy.InstantaneousPolicy(self.__config_data, self.__logger)
        

        if(self.__config_data['TM']['Debug']['enable_prog_part']):
            #Instantiate critical components of progressive parts
            # self.__state_monitor_prog = None
            # self.__policy_turn = None

            # Start the ASR service
            asr_listener = ASR_Word_Stream(self.__sensor_config_data)
            # Start the vision module
            emotion_listener = FE_Connector(self.__sensor_config_data)
            # Start the policy
            self.__policy_progressive = ProgressivePolicy(
                asr_listener=asr_listener,
                emotion_listener=emotion_listener
            )

    def __initiateDialogue(self):
        #For progressive part, initialize turn and trigger first action

        initial_action = self.__policy_progressive.initialize_conversation()
        # Execute the "start conversation action"
        # Yifan note: SHOULD STILL HAVE TURN-TAKING AND YIELDING ACTION!!
        self.__logger.info(initial_action)
        #self.__mergeExec(initial_action)

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
        self.__logger.info(decisions['inst_act']['gaze_action'])

        #Check if there is speech action from the progressive part
        if('prog_act' in decisions):
            #If there is, apply the speech action from prog part
            self.__logger.info(decisions['prog_act'])
        else:
            #If there isn't, apply actions from the inst part, if any
            self.__logger.info(decisions['inst_act']['bc_action'])

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
            print('[Iteration %d]' % it_cnt)
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
            rate.sleep()



if __name__ == '__main__':
    grace_config = loadGraceConfigs()
    grace_tm = TurnManager(grace_config)
    grace_tm.mainLoop()