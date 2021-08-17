###############################################################################
# This class handles motor communication with the Dynamixel wrist motors, which
# take place over the UART communication protocol.
#
#
# Authors: Levi Hargrove and Kevin Brenner
# Date: June 24, 2021
###############################################################################

import binascii, math, machine, pyb, time, uasyncio, ubinascii, ubluetooth, utime
import crc_calc, motor_control_dynamixel, motor_control_ottobock, motor_control_psyonic
from pyb import DAC, Pin, UART

class MotorLogic:
    # Desired hand velocity
    hand_vel = 0
    prev_hand_vel = 0
    # Desired wrist velocities
    rot_vel = 0
    flex_vel = 0

    # Tracking the present position and current as well as the desired position
    # for each motor
    rot_pos = 0
    rot_pos_des = 0
    rot_curr = 0
    flex_pos = 0
    flex_pos_des = 0
    flex_curr = 0

    # Tracking the present hand grip status of the Psyonic Ability Hand
    hand_grip_status = 0x00

    rot_pos_sign = 0 # 0 for positive, 1 for negative
    rot_angle = 0
    flex_pos_sign = 0 # 0 for positive, 1 for negative
    flex_angle = 0
    last_zero_vel_rot_pos = 0
    last_zero_vel_flex_pos = 0
    find_new_zero_vel_rot_pos = True
    find_new_zero_vel_flex_pos = True

    # Keeping track of the position and current of each motor from the previous
    # time step
    prev_rot_pos = 0
    prev_rot_curr = 0
    prev_flex_pos = 0
    prev_flex_curr = 0
    
    # Hardcoding the number of frames to keep track of
    wrist_history_length = 5
    hand_history_length = 20

    # Lists to track position and current history for each motor for the
    # previous "x" time steps
    rot_pos_history = [0]*wrist_history_length
    flex_pos_history = [0]*wrist_history_length
    rot_curr_history = [0]*wrist_history_length
    flex_curr_history = [0]*wrist_history_length
    hand_grip_stat_history = [0]*hand_history_length

    # Position constraint variables
    max_ccw_pos = 0
    max_cw_pos = 0
    mid_rot_pos = 0
    max_ext_pos = 0
    max_flex_pos = 0
    mid_flex_pos = 0

    # Hardcoded endstop values for smaller and larger cases (these depend on
    # what position the motor last powered down in)
    const_max_ext_pos = [0,0]
    const_max_flex_pos = [0,0]
    const_mid_flex_pos = [0,0]
    const_max_ccw_pos = [0,0]
    const_max_cw_pos = [0,0]
    const_mid_rot_pos = [0,0]

    # Initializing timer variables
    current_time = 0
    rot_time = 0
    flex_time = 0

    # Initializing boolean to say that the motors begin disabled
    rotator_disabled = True
    flexor_disabled = True
    
    # Tracking the enabled status of both motors from the "Motors Enabled"
    # button from the PAT app
    wrist_motors_enabled = False
    
    # The following variables are modified in the engineering parameters panel
    # from the PAT app.
    # Specifically relating to finding endstops
    max_rot_curr_ends = 400 #mA
    rot_vel_ends = 15
    max_flex_curr_ends = 400 #mA
    flex_vel_ends = 9
    # Specifically relating to normal operation
    max_rot_curr = 600 #mA
    pro_end_cush = 57 #encoder counts (encoder count*360/(1*4096) = angle)
    sup_end_cush = 57 #encoder counts (encoder count*360/(1*4096) = angle)
    max_flex_curr = 600 #mA
    flex_end_cush = 57 #encoder counts (encoder count*360/(1*4096) = angle)
    ext_end_cush = 57 #encoder counts (encoder count*360/(1*4096) = angle)
    
    # Booleans to be turned to true once endstops have been found
    rot_endstop_found = False
    flex_endstop_found = False
    endstops_found = False

    # Keeping track of how many incorrect position and current readings there
    # are in a row to determine if the issues is just from normal lag or if
    # there is something else occurring
    bad_readings = 0
    bad_writings = 0
    bad_watchdogs = 0
    bad_error_statuses = 0
    
    # Parameters to check if position or current thresholds have been exceeded
    rot_curr_exceeded = False
    rot_cw_pos_exceeded = False
    rot_ccw_pos_exceeded = False
    flex_curr_exceeded = False
    flex_flex_pos_exceeded = False
    flex_ext_pos_exceeded = False
    
    # Classifier decision variables (class out and prop speed)
    c_out = 1
    p_out = 0
    ramped_gain = 1
    class_decision = 1

    # Hard limits to angles on wrist
    hard_pro_limit = 175
    hard_sup_limit = 175
    hard_flex_limit = 40
    hard_ext_limit = 70

    # Wrist settings
    flex_activated = 0 #0 = deactive, 1 = active
    flex_ramp = 0
    flex_vote = 0
    flex_speed_damp = 0
    max_flex_angle = 0
    max_ext_angle = 0
    flex_pause_angle = 0
    flex_pause_time = 0
    ext_pause_angle = 0
    ext_pause_time = 0
    rot_activated = 0 #0 = deactive, 1 = active
    rot_ramp = 0
    rot_vote = 0
    rot_speed_damp = 0
    max_pro_angle = 0
    max_sup_angle = 0
    pro_pause_angle = 0
    pro_pause_time = 0
    sup_pause_angle = 0
    sup_pause_time = 0

    # Tracking the values for watchdog and hardware errors
    rot_watchdog = 0
    flex_watchdog = 0
    rot_error_status = 0
    flex_error_status = 0
    
    # Terminal device settings
    td_type = 0 #0 = ottobock, 1 = psyonic
    handedness = 0 #0 = left, 1 = right
    hand_activated = 1 #0 = active, 1 = deactive
    grip_percent_change = 1
    grip = [0,0,0,0,0]
    grip_speed = [2,2,2,2,2]
    grip_activated = [0,0,0,0,0]

    # Custom calibration settings
    calibration_time = 2
    adaptation = True
    motor_guided = True
    enabled_on_complete = True

    grip_command_type = 0x00
    
    # Initialize the Pyboard
    def __init__(self):
        self.mc_dyna = motor_control_dynamixel.MotorControlDynamixel()
        self.mc_psy = motor_control_psyonic.MotorControlPsyonic()
        self.mc_otto = motor_control_ottobock.MotorControlOttobock()
        self.current_time = time.ticks_ms()

    # Check to see if position or current thresholds have been exceeded
    #
    # Inputs:
    #   N/A
    #
    # Returns:
    #   N/A
    def safetyCheck(self):
        if math.fabs(self.rot_curr) > self.max_rot_curr:
            print("rot curr: ",math.fabs(self.rot_curr))
            self.rot_curr_exceeded = True 
        else:
            self.rot_curr_exceeded = False
        if math.fabs(self.flex_curr) > self.max_flex_curr:
            self.flex_curr_exceeded = True 
        else:
            self.flex_curr_exceeded = False
        if self.handedness == 0: # Left handed
            if self.rot_pos > (self.max_cw_pos - self.sup_end_cush):
                self.rot_cw_pos_exceeded = True 
            else:
                self.rot_cw_pos_exceeded = False
            if self.rot_pos < (self.max_ccw_pos + self.pro_end_cush):
                self.rot_ccw_pos_exceeded = True 
            else:
                self.rot_ccw_pos_exceeded = False
        elif self.handedness == 1: # Right handed
            if self.rot_pos > (self.max_cw_pos - self.pro_end_cush):
                self.rot_cw_pos_exceeded = True 
            else:
                self.rot_cw_pos_exceeded = False
            if self.rot_pos < (self.max_ccw_pos + self.sup_end_cush):
                self.rot_ccw_pos_exceeded = True 
            else:
                self.rot_ccw_pos_exceeded = False
        if self.flex_pos > (self.max_flex_pos - self.flex_end_cush):
            self.flex_flex_pos_exceeded = True
        else:
            self.flex_flex_pos_exceeded = False
        if self.flex_pos < (self.max_ext_pos + self.ext_end_cush):
            self.flex_ext_pos_exceeded = True
        else:
            self.flex_ext_pos_exceeded = False

    # Ensure that the motors are not passed their endstops or have not exceeded
    # their current thresholds before writing velocities to them.
    #
    # Inputs:
    #   rot_end_found:              flag notifying whether or not the rotator
    #                               endstop has been found
    #   flex_end_found:             flag notifying whether or not the flexor
    #                               endstop has been found
    #
    # Returns:
    #   N/A
    def writeToMotors(self,rot_end_found,flex_end_found):
        # If you have not found the endstops to the rotator, just write the
        # velocity to the motors without worrying about the checks
        if rot_end_found:
            if self.rot_curr_exceeded:
                self.rot_vel = 0
                print("ROT CURR EXCEEDED")
            elif self.rot_cw_pos_exceeded and self.rot_vel > 0:
                self.rot_vel = 0
                print("rot pos: ",self.rot_pos)
                print("max cw pos: ",self.max_cw_pos)
                print("CW Endstop")
            elif self.rot_ccw_pos_exceeded and self.rot_vel < 0:
                self.rot_vel = 0
                print("rot pos: ",self.rot_pos)
                print("max ccw pos: ",self.max_ccw_pos)
                print("CCW Endstop")
        else:
            if self.rot_vel > self.rot_vel_ends:
                self.rot_vel = self.rot_vel_ends
            elif self.rot_vel < -1*self.rot_vel_ends:
                self.rot_vel = -1*self.rot_vel_ends
        
        # If you have not found the endstops to the flexor, just write the
        # velocity to the motors without worrying about the checks
        if flex_end_found:
            if self.flex_curr_exceeded:
                self.flex_vel = 0
                print("FLEX CURR EXCEEDED")
            elif self.flex_flex_pos_exceeded and self.flex_vel > 0:
                self.flex_vel = 0
                print("flex pos: ",self.flex_pos)
                print("max flex pos: ",self.max_flex_pos)
                print("Flex Endstop")
            elif self.flex_ext_pos_exceeded and self.flex_vel < 0:
                self.flex_vel = 0
                print("flex pos: ",self.flex_pos)
                print("max ext pos: ",self.max_ext_pos)
                print("Ext Endstop")
        else:
            if self.flex_vel > self.flex_vel_ends:
                self.flex_vel = self.flex_vel_ends
            elif self.flex_vel < -1*self.flex_vel_ends:
                self.flex_vel = -1*self.flex_vel_ends
        
        if self.rot_vel != 0:
            self.find_new_zero_vel_rot_pos = True
            self.last_zero_vel_rot_pos = self.rot_pos
            self.rot_pos_des = self.rot_pos + self.rot_vel/0.229
        else:
            if self.find_new_zero_vel_rot_pos:
                self.rot_pos_des = self.rot_pos + self.rot_vel/0.229
                self.find_new_zero_vel_rot_pos = False
            else:
                if self.rot_cw_pos_exceeded or self.rot_ccw_pos_exceeded:
                    self.rot_pos_des = self.rot_pos
                else:
                    self.rot_pos_des = self.last_zero_vel_rot_pos

        if self.flex_vel != 0:
            self.find_new_zero_vel_flex_pos = True
            self.last_zero_vel_flex_pos = self.flex_pos
            self.flex_pos_des = self.flex_pos + self.flex_vel/0.229
        else:
            if self.find_new_zero_vel_flex_pos:
                self.flex_pos_des = self.flex_pos + self.flex_vel/0.229
                self.find_new_zero_vel_flex_pos = False
            else:
                if self.flex_flex_pos_exceeded or self.flex_ext_pos_exceeded:
                    self.flex_pos_des = self.flex_pos
                else:
                    self.flex_pos_des = self.last_zero_vel_flex_pos

        if not self.rot_activated:
            self.rot_vel = 0
            self.rot_pos_des = self.rot_pos
        if not self.flex_activated:
            self.flex_vel = 0
            self.flex_pos_des = self.flex_pos

        self.mc_dyna.writeToMotorsInd(self.rot_pos_des,self.flex_pos_des)
    
    # Writing a desired velocity to either hand open or hand close
    #
    # Inputs:
    #   write_psyonic_flag:         flag notifying whether or not to actually
    #                               write a message to the Psyonic Ability Hand
    #                               (should you be using that terminal device)
    #
    # Returns:
    #   N/A
    def writeHand(self,write_psyonic_flag):
        # Ottobock Transcarpal Hand
        if self.td_type == 0:
            self.mc_otto.writeOttobock(self.hand_activated,self.hand_vel)
        # Psyonic Ability Hand
        elif self.td_type == 1:
            try:
                if write_psyonic_flag:
                    self.mc_psy.writePsyonic(self.hand_activated,self.mc_psy.hand_set_grasp,self.grip_command_type,self.hand_vel)
            except Exception as e:
                print(e)
                pass
    
    # Reads in the motion that you are training along with a flag for whether
    # or not this is the first frame that you are training this motion.
    #
    # Inputs:
    #   motion:                     the motion that is being trained (i.e.
    #                               flexion)
    #   train_hand_condition:       a switch case that decides whether or not
    #                               you are training both hand directions or
    #                               just one
    #   train_rotator_condition:    a switch case that decides whether or not
    #                               you are training both rotation directions
    #                               or just one
    #   train_flexor_condition:     a switch case that decides whether or not
    #                               you are training both flexion directions or
    #                               just one
    #   prev_grip_trained:          the last grip that was trained
    #
    # Returns:
    #   N/A
    def setTrainingVelocities(self,motion,train_hand_condition,train_rotator_condition,train_flexor_condition,prev_grip_trained):
        # Assuming that we are driving motors during device based training
        if self.motor_guided:
            # Depending on what the motion is to be trained, and assuming the
            # motors are enabled, set a specific set of hand, rotator, and
            # flexor motor velocities
            if motion in ["No Movement","Prep"]:
                if self.wrist_motors_enabled:
                    self.rot_vel = 0
                    self.flex_vel = 0
                    self.hand_vel = 0
            elif motion == "Open":
                if self.wrist_motors_enabled:
                    self.rot_vel = 0
                    self.flex_vel = 0
                    if self.td_type == 0:
                        if train_hand_condition == 1:
                            self.hand_vel = 35
                        elif train_hand_condition == 3:
                            self.hand_vel = 42
                    elif self.td_type == 1:
                        self.hand_vel = 1
                        if prev_grip_trained == "Power":
                            self.hand_vel = 1
                        elif prev_grip_trained == "Key":
                            self.hand_vel = 1
                        elif prev_grip_trained == "Chuck":
                            self.hand_vel = 1
                        elif prev_grip_trained == "Pinch":
                            self.hand_vel = 1
                        elif prev_grip_trained == "Point":
                            self.hand_vel = 1
                        elif prev_grip_trained == "Rock":
                            self.hand_vel = 1
                        elif prev_grip_trained == "Trigger":
                            self.hand_vel = 1
                        elif prev_grip_trained == "HandShake":
                            self.hand_vel = 1
                        elif prev_grip_trained == "ChuckOk":
                            self.hand_vel = 1
                        self.grip_command_type = self.mc_psy.relax_cmd
            elif motion in ["Close","Power","Key","Chuck","Pinch","Point","Rock","Trigger","HandShake","ChuckOk"]:
                if self.wrist_motors_enabled:
                    self.rot_vel = 0
                    self.flex_vel = 0
                    if self.td_type == 0:
                        if train_hand_condition == 2:
                            self.hand_vel = -35
                        elif train_hand_condition == 3:
                            self.hand_vel = -42
                    elif self.td_type == 1:
                        if motion == "Power":
                            self.hand_vel = 255
                            self.grip_command_type = self.mc_psy.power_grasp_cmd
                        elif motion == "Key":
                            self.hand_vel = 1
                            self.grip_command_type = self.mc_psy.key_grasp_cmd
                        elif motion == "Chuck":
                            self.hand_vel = 1
                            self.grip_command_type = self.mc_psy.chuck_grasp_cmd
                        elif motion == "Pinch":
                            self.hand_vel = 1
                            self.grip_command_type = self.mc_psy.pinch_grasp_cmd
                        elif motion == "Point":
                            self.hand_vel = 175
                            self.grip_command_type = self.mc_psy.point_grasp_cmd
                        elif motion == "Rock":
                            self.hand_vel = 255
                            self.grip_command_type = self.mc_psy.sign_of_the_horns_grasp_cmd
                        elif motion == "Trigger":
                            self.hand_vel = 255
                            self.grip_command_type = self.mc_psy.trigger_grip_cmd
                        elif motion == "HandShake":
                            self.hand_vel = 200
                            self.grip_command_type = self.mc_psy.handshake_grip_cmd
                        elif motion == "ChuckOk":
                            self.hand_vel = 1
                            self.grip_command_type = self.mc_psy.chuck_ok_grasp_cmd
            elif motion == "Pronation":
                if self.wrist_motors_enabled and self.rot_endstop_found:
                    if self.handedness == 1:
                        if train_rotator_condition == 1:
                            self.rot_vel = 10
                        elif train_rotator_condition == 3:
                            self.rot_vel = 20
                    elif self.handedness == 0:
                        if train_rotator_condition == 1:
                            self.rot_vel = -10
                        elif train_rotator_condition == 3:
                            self.rot_vel = -20
                    self.flex_vel = 0
                    self.hand_vel = 0
            elif motion == "Supination":
                if self.wrist_motors_enabled and self.rot_endstop_found:
                    if self.handedness == 1:
                        if train_rotator_condition == 2:
                            self.rot_vel = -10
                        elif train_rotator_condition == 3:
                            self.rot_vel = -20
                    elif self.handedness == 0:
                        if train_rotator_condition == 2:
                            self.rot_vel = 10
                        elif train_rotator_condition == 3:
                            self.rot_vel = 20
                    self.flex_vel = 0
                    self.hand_vel = 0
            elif motion == "Flexion":
                if self.wrist_motors_enabled and self.flex_endstop_found:
                    self.rot_vel = 0
                    if train_flexor_condition == 1:
                        self.flex_vel = 3
                    elif train_flexor_condition == 3:
                        self.flex_vel = 6
                    self.hand_vel = 0
            elif motion == "Extension":
                if self.wrist_motors_enabled and self.flex_endstop_found:
                    self.rot_vel = 0
                    if train_flexor_condition == 2:
                        self.flex_vel = -3
                    elif train_flexor_condition == 3:
                        self.flex_vel = -6
                    self.hand_vel = 0
                    self.grip_command_type = self.mc_psy.general_open_cmd
        else:
            if self.wrist_motors_enabled:
                self.rot_vel = 0
                self.flex_vel = 0
                self.hand_vel = 0
        
        # When you receive the "Done!!!" message, set all motors to zero
        if motion == "Done!!!":
            if self.wrist_motors_enabled:
                self.rot_vel = 0
                self.flex_vel = 0
                self.hand_vel = 0
    
    # Take in clas_out decision and prop_speed and then write to the motors
    #
    # Inputs:
    #   clas_out:                   the class that has been classified
    #   prop_speed:                 the speed of the the classification
    #
    # Returns:
    #   N/A
    def sendClassifierDecision(self,clas_out,prop_speed):
        self.c_out = clas_out
        
        # Track the current time to compare to the last time you
        # received a nonzero speed for the rotator or flexor
        self.current_time = time.ticks_ms()

        # If you have received a clas_out decision for the wrist
        if clas_out in [10,11,12,13]:
            # If clas_out is either supination or pronation
            if clas_out in [10,11]:
                # Added *(1/2) when transmission was reduced to 1 to 1
                prop_speed = 2.25*self.rot_speed_damp*(1/2)*prop_speed
                if clas_out == 11:
                    #print("Wrist Pronation")
                    if self.handedness == 0:
                        # Conversion to a value between -100 and 0
                        self.rot_vel = -1*prop_speed*100/float(255)
                    elif self.handedness == 1:
                        # Conversion to a value between 0 and 100
                        self.rot_vel = prop_speed*100/float(255)
                elif clas_out == 10:
                    #print("Wrist Supination")
                    if self.handedness == 0:
                        # Conversion to a value between 0 and 100
                        self.rot_vel = prop_speed*100/float(255)
                    elif self.handedness == 1:
                        # Conversion to a value between -100 and 0
                        self.rot_vel = -1*prop_speed*100/float(255)
                self.flex_vel = 0
                self.hand_vel = 0

            # If clas_out is either flexion or extension
            elif clas_out in [12,13]:
                # Added *(1/3) when transmission was removed
                prop_speed = self.flex_speed_damp*(1/3)*prop_speed
                # Only write zero to the other DOF's if they recently had a
                # nonzero velocity written to them recently
                if clas_out == 12:
                    #print("Wrist Flexion")
                    # Conversion to a value between -100 and 0
                    self.flex_vel = prop_speed*100/float(255)
                elif clas_out == 13:
                    #print("Wrist Extension")
                    # Conversion to a value between 0 and 100
                    self.flex_vel = -1*prop_speed*100/float(255)         
                self.rot_vel = 0
                self.hand_vel = 0

        # If you have received a clas_out decision for the hand
        elif clas_out in [16,19,17,18,20,22,24,25,26,27]:
            # Ottobock Transcarpal Hand
            if self.td_type == 0:
                # Hand Open
                if clas_out == 16:
                    #print("Hand Open")
                    # Conversion to a value between 0 and 100
                    if self.grip_speed[0] != -1:
                        self.hand_vel = 1.25*self.grip_speed[0]*prop_speed*100/float(255)
                    else:
                        self.hand_vel = 0
                # Hand Close
                elif clas_out == 19:
                    #print("Hand Close")
                    # Conversion to a value between -100 and 0
                    if self.grip_speed[0] != -1:
                        self.hand_vel = -1.25*self.grip_speed[0]*prop_speed*100/float(255)
                    else:
                        self.hand_vel = 0
            # Psyonic Ability Hand
            elif self.td_type == 1:
                self.hand_vel = self.grip_speed[0]*prop_speed*100/float(255)
                # Hand Open
                if clas_out == 16:
                    self.grip_command_type = self.mc_psy.general_open_cmd
                # Power Grasp
                elif clas_out == 19:
                    self.grip_command_type = self.mc_psy.power_grasp_cmd
                # Key Grasp
                elif clas_out == 17:
                    self.grip_command_type = self.mc_psy.key_grasp_cmd
                # Chuck Grasp
                elif clas_out == 18:
                    self.grip_command_type = self.mc_psy.chuck_grasp_cmd
                # Pinch Grasp
                elif clas_out == 20:
                    self.grip_command_type = self.mc_psy.pinch_grasp_cmd
                # Point Grasp
                elif clas_out == 22:
                    self.grip_command_type = self.mc_psy.point_grasp_cmd
                # Rock (Sign of the Horns) Grasp
                elif clas_out == 24:
                    self.grip_command_type = self.mc_psy.sign_of_the_horns_grasp_cmd
                # Chuck Ok Grasp
                elif clas_out == 27:
                    self.grip_command_type = self.mc_psy.chuck_ok_grasp_cmd
            self.rot_vel = 0
            self.flex_vel = 0
            
        # If you have received a clas_out decision for no movement
        else:
            #print("No Movement")
            self.rot_vel = 0
            self.flex_vel = 0
            self.hand_vel = 0
