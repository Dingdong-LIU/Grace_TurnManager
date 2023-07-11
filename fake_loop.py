# 1. initialize all loggers
import utils.logging_config


def mainloop():


    # 2. instantiate sensors & action execution channel: 
    ## ASR and FE(with attention)
    ## instantiate 4 modules:
    # 2.1 state module1: instantainous state monitor (one line)
    # 2.2 state module2: initialize turn manager module 
    # 2.3 policy module1: instanious policy
    # 2.4 policy module2: engagement and decorators of robot response
    ## 2.4.1 definition of turn talking action & topics [definitions are in Grace_Pace_Monitor/config/config.yaml/line23]

    ## instantiate action execution channel:
    # grace_behavior_client = rospy.ServiceProxy(self.grace_api_configs['Ros']['grace_behavior_service'], grace_attn_msgs.srv.GraceBehavior)


    # 3. construct the first turn object and start initial greetings with a magic string
    ## state changes are listed in Grace_Pace_Monitor/robot_behav_fsm/getState

    ## start from now it will loop
        # State update
            # Call the updateState() in the 2.1
            # 4. receive language from patient
            # 5. monitor state change and construct a turn object
            # 5.1 engagement estimator
        
        # Policy decision
            # apply instantious policy and get an dict of actions
            # turn actions by instantaneous and progressive state
                # apply progressive policy: communicate with DM if turn transition
                    # construct a dict of actions [1.turn taking | 2.speech | 3.turn yielding (check from robot speaking state (natural ending) or barge-in (active ending) )]

        # Action execution
            # In human's turn do not look at progressive actions
            # In robot's turn always check the turn actions
                # speech actions > bc actions
            # Also check the "gracefully end" turn action; the end of conversation action when questionnaire finishes.
            # call a service request for action execution
    ## loop ends
    pass
