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


#Create Logger
def setupLogger(file_log_level, terminal_log_level, logger_name, log_file_name):
    log_formatter = logging.Formatter('%(asctime)s %(msecs)03d %(name)s %(levelname)s %(funcName)s(%(lineno)d) %(message)s', 
                                  datefmt='%d/%m/%Y %H:%M:%S')

    f = open(log_file_name, "a")
    f.close()
    file_handler = logging.FileHandler(log_file_name)
    file_handler.setFormatter(log_formatter)
    file_handler.setLevel(file_log_level)

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(log_formatter)
    stream_handler.setLevel(terminal_log_level)

    logger = logging.getLogger(logger_name)
    logger.addHandler(file_handler)
    logger.addHandler(stream_handler)
    logger.setLevel( min(file_log_level,terminal_log_level) )#set to lowest

    return logger

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

        # # Load sensor config
        # sensor_config_path = os.path.join(
        #     os.path.dirname(os.path.realpath(getsourcefile(lambda:0))), 
        #     "..", 'Grace_Sensor_Interface/config/config.yaml'
        # )
        # self.__sensor_config_data = loadConfig(sensor_config_path)

        #Ros related components for calling the behavior executor
        self.__nh = True
        rospy.init_node(self.__config_data['TM']['Ros']['node_name'])
        self.__grace_behav_client = rospy.ServiceProxy(
                                        self.__config_data['Custom']['Behavior']['grace_behavior_service'], 
                                        grace_attn_msgs.srv.GraceBehavior)



        #Instantiate critical components of instantaneous parts
        self.__state_monitor_inst = Grace_Pace_Monitor.grace_instantaneous_state_monitor.InstantaneousStateMonitor(self.__config_data, self.__nh)
        self.__policy_instantaneous = Grace_Instantaneous_Policy.grace_instantaneous_policy.InstantaneousPolicy(self.__config_data)
        

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

    def __updateStates(self):
        self.__state_monitor_inst.updateState()

        #Progressive part is automatically updating
        # self.__state_monitor_prog.updateState()

    def __applyPolicy(self):
        decisions = {}

        # Apply the instantaneous policy
        state = self.__state_monitor_inst.getState()
        instantaneous_actions = self.__policy_instantaneous.applyPolicy(state)
        decisions['inst_act'] = instantaneous_actions

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
        
        rate = rospy.Rate(self.__config_data['TM']['General']['main_freq'])
        it_cnt = 0
        while True:
            it_cnt = it_cnt + 1
            print('[Iteration %d]' % it_cnt)
            if(it_cnt == 1):
                #Initial state

                #For instantaneous part, initial state is hardcoded, we just reset timestamp
                self.__state_monitor_inst.initializeState()

                if(self.__config_data['TM']['Debug']['enable_prog_part']):
                    # For progressive part, construct the first turn
                    initial_action = self.__policy_progressive.initialize_conversation()
                    # Execute the "start conversation action"
                    self.__logger.info(initial_action)
                    #self.__mergeExec(initial_action)

            else:
                #Update states
                self.__updateStates()

                #Apply policies
                decisions = self.__applyPolicy()

                #Merge and execute actions
                self.__mergeExec(decisions)

            #Sleep by rate
            rate.sleep()



if __name__ == '__main__':
    file_path = os.path.dirname(os.path.realpath(getsourcefile(lambda:0)))
    sys.path.append(os.path.join(file_path, '..'))
    from CommonConfigs.grace_cfg_loader import *
    grace_config = loadGraceConfigs()


    grace_tm = TurnManager(grace_config)
    grace_tm.mainLoop()