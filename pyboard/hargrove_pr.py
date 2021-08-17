# This file runs a pattern recognition and data logging test app. This is in
# beta and changing.
# It requires pyboard d-series with ulab installed.
# It also requires an SD on board. Recommended to modify your boot.py file
# to increase clock frequency and mount the SD card for data logging.
#
# Author: Levi Hargrove and Kevin Brenner
# Date: June 25, 2021

import pattern_rec, ble_comms, can_logger, coamp, save_helper, motor_logic, uasyncio, crc_calc, math
import ubluetooth, utime, ubinascii, pyb, os, micropython, gc, sys, machine, utime, time
import ulab as np
from pyb import CAN
from machine import I2C
from pyb import Pin

global sh,b,c,c_log,ml,p_rec
global rot_pos, rot_curr, flex_pos, flex_curr, loop_time, dev_kit
global is_training, collecting_data, class_training
global home_wrist, hand_timer, can_streaming
global rot_end_not_verified,flex_end_not_verified
global manual_motors, can
global need_to_enable, need_to_disable
global train_hand, train_rotator, train_flexor, send_classifier_decision
global emg_gains,td_settings,wrist_settings,settings_dict
global username, settings_file, paused, model_data
global list_of_classes_to_reset, classifier_history
global train_hand_condition, train_rotator_condition, train_flexor_condition
global moving_to_neutral, troubleshooting, pyboard_reset, running_diagnostic
global troubleshooting_description, troubleshooting_step, troubleshooting_step_num
global first_nm, first_flex, first_ext, first_pro, first_sup, first_open, first_close, hand_increment
global mess_id, mess_string, mess_id_list, mess_string_list, dbt_training_start_time, dbt_case
global calibration_counter, dbt_counter, tshoot_folder_name, new_tshoot_sequence
global last_mess, rot_counter, flex_counter
global done_moving_cw, done_moving_ccw, done_moving_ext, done_moving_flex, paused
global endstops_file, endstops_dict, desired_locked_position, locking
global first_prep_frame, prev_frame_train, prev_grip_trained
global cal_count_ottobock, cal_count_psyonic, cal_count_dict, cal_count_file
global first_grip, num_grips_active, active_grips, num_mess, write_psyonic_flag, send_last_mess

micropython.alloc_emergency_exception_buf(100)
error_log_file = open('/sd/error_log.txt', 'w')

###############################################################################

###############################################################################
###INITIALIZE CLASS OBJECTS, FLAGS, AND OTHER LOCAL/STORED VARIABLES###
###############################################################################

try:
    ###########################################################################
    ###INITIALIZING CLASS INSTANCES###
    ###########################################################################

    # Initializing bluetooth (BLE) module on the Pyboard
    ble = ubluetooth.BLE()
    # Initializing CAN module on the Pyboard
    can = CAN(1)
    # Class to help with file I/O on the Pyboard
    sh = save_helper.SaveHelper()
    # Class to help with bluetooth communication
    b = ble_comms.BLEComms(ble)
    # Class to help with CAN communication
    c = coamp.Coamp(can,False)
    # Class to help with logging CAN data to the Pyboard
    c_log = can_logger.CanLogger()
    # Class to help with reading/writing to various motors (rotator, flexor,
    # terminal devices, etc.)
    ml = motor_logic.MotorLogic()
    # Class to help with creating a classifier
    p_rec = pattern_rec.PatternRec(sh)

    # Try to write to the wrist motors
    # If you can without errors, then you are working with a physical device
    # and not a development kit
    # Vice versa if it errors out
    try:
        ml.mc_dyna.disableMotor(ml.mc_dyna.rotator_motor)
        dev_kit = False
        print("PHYSICAL DEVICE")
    except:
        dev_kit = True
        print("DEV KIT")

    ###########################################################################
    ###INITIALIZING SCRIPT VARIABLES###
    ###########################################################################

    # Are you in the process in training a calibration?
    is_training = False
    # Are you specifically collecting data (not just prepping)?
    collecting_data = False
    # The string name for the class that is being trained
    class_training = ""
    # During the homing sequence, has the rotator made it to its homing
    # location?
    rotator_homed = False
    # During the homing sequence, has the flexor made it to its homing
    # location?
    flexor_homed = False
    # During the homing sequence (or the set to neutral/diagnostic check
    # sequence), has the hand made it to its homing location?
    hand_homed = False
    # Is the classifier controlling the device/VR Arm?
    send_classifier_decision = False
    # Are you currently in the process of trying to find end stops?
    home_wrist = False
    # Are you currently in the process of manually driving the motors?
    manual_motors = False
    # Does the rotator physically know its hard end stop? This is set to true
    # when the rotator hits its endstop for the first time, but gets reset to
    # false on a power cycle.
    true_rot_end_found = True
    # Does the flexor physically know its hard end stop? This is set to true
    # when the flexor hits its endstop for the first time, but gets reset to
    # false on a power cycle.
    true_flex_end_found = True
    # Does the rotator need to be rebooted?
    reboot_rotator = False
    # Does the flexor need to be rebooted?
    reboot_flexor = False
    # Have you received a "motor enable" message and thus the motor is waiting
    # to be enabled?
    need_to_enable = False
    # Have you received a "motor disable" message and thus the motor is waiting
    # to be disabled?
    need_to_disable = False
    # Initialized string for the username
    username = ""
    # Initialized string for the user settings file name
    settings_file = "settings.json"
    # Are we pausing the calibration because the user pressed the pause button?
    paused = False
    # Are we in the process of moving to the neutral position?
    moving_to_neutral = False
    # Are we in the process of running a diagnostic check?
    running_diagnostic = False
    # Are we in the process of troubleshooting?
    troubleshooting = False
    # A list of the classes that need to be reset
    list_of_classes_to_reset = []
    # Initializing a list to hold the past "X" number of classifier decisions
    # Set the size of this to be the maximum number in the vote dropdown in the
    # PAT App
    classifier_history = [-1]*10
    # Has the pyboard just been reset?
    pyboard_reset = True
    # Which step are you on in the troubleshooting sequence?
    troubleshooting_step = 0
    # What is the name of the section of troubleshooting that you are in?
    troubleshooting_description = ""
    # What step of the troubleshooting routine are you on?
    troubleshooting_step_num = 0
    # Initialized dictionary for the wrist endstops
    endstops_dict = {}
    # Initialized string for the wrist endstops file
    endstops_file = "endstops.json"
    # Initialized string for the calibration counter file
    cal_count_file = "cal_count.json"
    # Booleans for knowing if this is the first or second time you are
    # collecting data for a given motion
    first_nm = True
    first_flex = True
    first_ext = True
    first_pro = True
    first_sup = True
    first_open = True
    first_close = True
    hand_increment = 0
    # Timer used to track training times for device based training
    dbt_training_start_time = 0
    # Integer to track the step in the deviced base training sequence that you
    # are currently on
    dbt_case = 0
    # Boolean to track if the wrist is in a locked or unlocked state
    locking = False
    # A desired position for the flexor which is selected by the user to tell
    # the wrist which position it should try to lock in
    desired_locked_position = 0
    # Message ID and string to be sent back to the PAT App in the ParseBT
    # async function
    mess_id = 0x00
    mess_string = b' '
    # Lists containing a buildup of messaged ID's and strings to be sent to the
    # PAT App
    mess_id_list = [0x00]*30
    mess_string_list = [b' ']*30
    # Tracking whether or not this is an entirely new troubleshooting sequence
    # or still part of a previous one
    new_tshoot_sequence = False
    # Noting the name of the foldre that the troubleshooting sequence should be
    # saved to
    tshoot_folder_name = ""
    # Tracking the previously received Bluetooth message (used for comparing
    # the current message to the previous one to check to see if there are any
    # messages being repeated over and over)
    last_mess = bytearray(32)
    # Logical flags used to determine what frame we should be reading in and
    # writing individual columns of the covariance matrix for a given class
    first_prep_frame = False
    prev_frame_train = False
    # Keeping track of the previously trained grip type when using the Psyonic
    # hand (this is useful because the hand will end in different locations
    # depending on which grip was just trained, and thus will require different
    # speeds/times to return to a normal/relaxed position)
    prev_grip_trained = ""
    # Keeping track of which grip will be trained first when using the Psyonic
    # hand (need to knwo this because we need to provide a specific speed to
    # the hand initially to get it to that grip's final position in the desired
    # amount of time)
    first_grip = 0
    # Keeping track of how many grips the user settings determines will be
    # trained and calibrated (needed in order to creating the correct order of
    # events for device based training [since there is no command dictating the
    # order from the PAT App])
    num_grips_active = 0
    # Keeping track of which grips are active (and their order) so that if the
    # user starts removing grips and then adding them back in, but in a
    # different order, the knowledge of which grip is which is preserved
    active_grips = []
    # Keeping track of how many CAN messages are in the buffer (if this value
    # exceeds a certain threshold, we are stopping the motors from driving the
    # wrist since the motion will be quite choppy as a result)
    num_mess = 0
    # Flags for deciding whether or not you currently need to train the various
    # degrees of freedom
    train_hand = False
    train_rotator = False
    train_flexor = False
    # Counters for assisting with moving the motros off of their endstops when
    # they are homing (if the motors get too close to their endstops, sometimes
    # the logic will get them stuck there, so we spin them in the opposite
    # direction for 5 frames, and these counters help track those frames)
    rot_counter = 0
    flex_counter = 0
    # Flags to keep track of whether or not each motion has been completed in
    # its process to get the wrist into its position for training (i.e. has it
    # reached its furthest extended position so that it can now begin its
    # flexion training)
    done_moving_cw = False
    done_moving_ccw = False
    done_moving_ext = False
    done_moving_flex = False
    # Flag to notify the hand whether or not to actually write a message to the
    # Psyonic hand (assuming you have the Psyonic hand attached)
    write_psyonic_flag = True
    # Flag to notify whether or not we need to send the "final message" BLE
    # message, which is true if we have sent other messages
    send_last_mess = False

    # Initializing the connection status of the Bluetooth class
    con_status = bytearray(3)
    con_status[0] = 0x0a

    # Update the sample frequency to a desired integer value (hex conversion is
    # handled in the updateSampleFreq function)
    c.updateSampleFreq(500)
    c.sendStart()

    # #For now, leave this as not actually doing anything but
    # #just modify the display so the clinicians are happy
    # #Update gains via CAN messages to the myoIMU
    # for i in range(len(c._emg_gains)):
    #     c.updateGain(i+1,c._emg_gains[i])

    ###########################################################################
    ###FILE I/O OF VARIOUS VARIABLES STORED ON THE SD CARD###
    ###########################################################################

    # Change directions and read in the hardcoded endstops of the motors (these
    # values will change if you mechanically dissassemble the rotator or
    # flexor, so keep that in mind in case the wrist doesn't appear to be
    # moving to its expected end range of motion)
    os.chdir('/sd')
    [ml.const_max_ccw_pos,ml.const_max_cw_pos,ml.const_mid_rot_pos,ml.const_max_ext_pos,ml.const_max_flex_pos,ml.const_mid_flex_pos,dict] = sh.readEndstops(endstops_file)

    # The counter that corresponds to the counter of the present terminal
    # device (there are stored values that correspond to how many classifier
    # models there are for each terminal device, so this is a local variable
    # that tracks whichever one is attached to the wrist)
    calibration_counter = 0

    # Read in the counte values for how many classifier models have been
    # trained for each terminal device
    [cal_count_ottobock,cal_count_psyonic,cal_count_dict] = sh.readCalCount(cal_count_file)

    # Read in the device settings for the wrist, terminal device, and emg gains 
    [emg_gains,td_settings,wrist_settings,settings_dict] = sh.readSettings(settings_file)

    # order of data: emg gain0, emg gain1, emg gain2, emg gain3, emg gain4,
    # emg gain5, emg gain6, emg gain7
    # Emg gains based on last known settings list
    for i in range(len(c._emg_gains)):
        c._emg_gains[i] = emg_gains[i]

    # order of data: num_dof/enable/handedness, % hand open on grip change,
    # grip0, grip1, grip2, grip3, grip4, grip0 speed, grip1 speed, grip2
    # speed, grip3 speed, grip4 speed
    # Setting terminal device settings based on default settings list
    if td_settings[0] < 0:
        ml.hand_activate = 0
    # Determine which terminal device is attached (assuming you're using a
    # development kit)
    if dev_kit:
        if math.fabs(td_settings[0]) in [2,4]:
            ml.td_type = 1
            p_rec._td_type = "Psyonic"
            calibration_counter = cal_count_psyonic
        else:
            ml.td_type = 0
            p_rec._td_type = "Ottobock"
            calibration_counter = cal_count_ottobock
    # Otherwise, make this determination by attempting to detect the Psyonic
    # hand, and depending on its error status, you'll know which terminal
    # device is attached
    else:
        psyonic_attached = ml.mc_psy.detectPsyonicHand()
        # If the Psyonic hand is attached but the last known settings think the
        # Ottobock hand is attached, then overwrite the local classifier model
        # for the terminal device classes and update the local terminal device
        if psyonic_attached:
            print("PSYONIC HAND")
            ml.td_type = 1
            p_rec._td_type = "Psyonic"
            calibration_counter = cal_count_psyonic
            f = open('td_type.txt','r')
            td_type_string = f.read()
            print("td_type_string: ",td_type_string)
            f.close()
            if td_type_string == "Ottobock":
                p_rec.resetSpecificClasses([5,6,7,8,9])
                g = open('td_type.txt','w')
                g.write('Psyonic')
                g.close()
        # If the Ottobock hand is attached but the last known settings think
        # the Psyonic hand is attached, then overwrite the local classifier
        # model for the terminal device classes and update the local terminal
        # device
        else:
            print("OTTOBOCK HAND")
            ml.td_type = 0
            p_rec._td_type = "Ottobock"
            calibration_counter = cal_count_ottobock
            f = open('td_type.txt','r')
            td_type_string = f.read()
            print("td_type_string: ",td_type_string)
            f.close()
            if td_type_string == "Psyonic":
                p_rec.resetSpecificClasses([5,6,7,8,9])
                g = open('td_type.txt','w')
                g.write('Ottobock')
                g.close()
    
    # If you are using a development kit, check for the handedness value in the
    # settings file
    if dev_kit:
        if math.fabs(td_settings[0]) in [1,2]:
            ml.handedness = 0
        else:
            ml.handedness = 1
    # Otherwise, you need to hardcode the handedness for each user (we don't
    # have a way to detect which terminal device is attached between right and
    # left)
    else:
        ml.handedness = 0

    # Continue update local variables pertaining to the terminal device based
    # on what the settings say
    ml.grip_percent_change = td_settings[1]/100
    for i in range(len(ml.grip)):
        if td_settings[2+i] == 255:
            ml.grip[i] = -1
        else:
            ml.grip[i] = td_settings[2+i]
    for i in range(len(ml.grip)):
        if td_settings[7+i] == 255:
            ml.grip_speed[i] = -1
        else:
            ml.grip_speed[i] = td_settings[7+i]/25
    for i in range(5):
        if td_settings[2+i] != 255 and td_settings[2+i] != -1:
            num_grips_active = num_grips_active + 1
            string_name = "Grip" + str(i)
            active_grips.append(string_name)

    p_rec._grip_mapper["Grip0"] = ml.grip[0]
    p_rec._grip_mapper["Grip1"] = ml.grip[1]
    p_rec._grip_mapper["Grip2"] = ml.grip[2]
    p_rec._grip_mapper["Grip3"] = ml.grip[3]

    # order of data: FE_enable, FE_ramp, FE_vote, FE_speed_damping, max flexion
    # angle, max extension angle, FE_pause_angle, FE_pause_time, PS_enable,
    # PS_ramp, PS_vote, PS_speed_damping, max pronation angle, max supination
    # angle, PS_pause_angle, PS_pause_timengle
    # setting wrist settings based on default settings list
    ml.flex_activated = wrist_settings[0]
    ml.flex_ramp = wrist_settings[1]
    ml.flex_vote = wrist_settings[2]
    ml.flex_speed_damp = wrist_settings[3]/100
    ml.max_flex_angle = wrist_settings[4]*(ml.hard_flex_limit/100)*(1*4096)/360
    ml.max_ext_angle = wrist_settings[5]*(ml.hard_ext_limit/100)*(1*4096)/360
    # Set both flexion and extension to the same values to start
    ml.flex_pause_angle = wrist_settings[6]/100
    ml.flex_pause_time = wrist_settings[7]/1000 #in milliseconds
    ml.ext_pause_angle = wrist_settings[6]/100
    ml.ext_pause_time = wrist_settings[7]/1000 #in milliseconds

    ml.rot_activated = wrist_settings[8]
    ml.rot_ramp = wrist_settings[9]
    ml.rot_vote = wrist_settings[10]
    ml.rot_speed_damp = wrist_settings[11]/100
    ml.max_pro_angle = wrist_settings[12]*(ml.hard_pro_limit/100)*(1*4096)/360
    ml.max_sup_angle = wrist_settings[13]*(ml.hard_sup_limit/100)*(1*4096)/360
    # Set both pronation and supination to the same values to start
    ml.pro_pause_angle = wrist_settings[14]/100
    ml.pro_pause_time = wrist_settings[15]/1000 #in milliseconds
    ml.sup_pause_angle = wrist_settings[14]/100
    ml.sup_pause_time = wrist_settings[15]/1000 #in milliseconds

    # Detecting whether the max angles for each wrist DOF exceed the endstop
    # cushion, and if so, setting the endstop cushion to be equal to the max
    # angle for that DOF
    if 180-ml.max_pro_angle > ml.pro_end_cush*360/(1*4096):
        ml.pro_end_cush = (180-ml.max_pro_angle)*(1*4096)/360
    if 180-ml.max_sup_angle > ml.sup_end_cush*360/(1*4096):
        ml.sup_end_cush = (180-ml.max_sup_angle)*(1*4096)/360
    if 70-ml.max_ext_angle > ml.ext_end_cush*360/(1*4096):
        ml.max_ext_cush = (70-ml.max_ext_angle)*(1*4096)/360
    if 49-ml.max_flex_angle > ml.flex_end_cush*(360)/(1*4096):
        ml.max_flex_cush = (49-ml.max_flex_angle)*(1*4096)/360

except Exception as e:
    print("Recording error from initialization")
    print(Exception)
    sys.print_exception(e, error_log_file)
    error_log_file.flush()
    raise

###############################################################################

###############################################################################
###ASYNCHRONOUS FUNCTIONS###
###############################################################################

# Function to handle incoming CAN messages
#
# Inputs:
#   N/A
#
# Returns:
#   N/A
async def handleMessages():
    global c,error_log_file, can, num_mess
    while True:
        try:
            # Check to see how many new messages are available on the CAN bus
            cntrs = c.getCounters()
            num_mess = cntrs[0] - cntrs[1]
            # Loop through each message in the buffer and "handle" the data,
            # which effectively just means unpacking the data and depending on
            # which filtered channel it comes thorugh, act accordingly
            for x in range(num_mess):
                c.handleData2()
            # Keep this value very small (if not 0) since we have CAN messages
            # coming through at a sampling rate of 500Hz (every 2ms)
            await uasyncio.sleep_ms(0)
        except Exception as e:
            # If there is an error in this specific asynchronous function, then
            # log it accordingly
            print("Recording error from handle messages loop")
            sys.print_exception(e, error_log_file)
            error_log_file.flush()
            raise

# Funciton to handle Bluetooth communication
#
# Inputs:
#   N/A
#
# Returns:
#   N/A
async def parseBT():
    global c, p_rec, b, c_log, error_log_file, last_mess, mess_string_list, mess_id_list, con_status
    while True:
        try:
            # Checks to see if there is a bluetooth message awaiting being
            # handled
            if b.checkMessageFlag():
                # Returns the contents of the message
                new_mess = b.getMessage()
                # This line of logic is to prevent multiple calibration models
                # of the exact same message from being handled multiple times
                # in a row, which specifically disrupts the logic used for
                # storing troubleshooting models
                # It also works for accidentally sending the same manual motors
                # message repeatedly
                if last_mess == new_mess:
                    # If the message is a calibration message or a manual
                    # motors message, immediately send a response message
                    # but don't parse it (since an identical message was
                    # just parsed)
                    if last_mess[0] == 22 or last_mess[0] == 33:
                        resp_mess = bytearray(len(new_mess))
                        resp_mess[0] = 0x0b
                        resp_mess[1:] = new_mess
                        b.sendMessage(resp_mess)
                    # Otherwise, parse it again because it truly is a new
                    # message
                    else:
                        parseMessage(new_mess)
                # If two successive messages aren't the same, then absolutely
                # parse both of them
                else:
                    parseMessage(new_mess)
                # If we are connected over Bluetooth, send a confirmation
                # message that lets the app know if we are streaming data over
                # CAN and if we are logging EMG data
                if b.checkSendControllerStatus():
                    con_status[1] = c.checkStreaming()
                    con_status[2] = c_log.checkWriting()
                    b.sendMessage(con_status)
                    b.setSendControllerStatus(False)
                # We have a buffer system for loading up bluetooth messages to
                # be sent one by one to the app. This bit of logic checks to
                # see if there are any knew messages to be sent out. If there
                # are, send the next message in the buffer to the corresponding
                # next available address
                if mess_id_list[0] != 0x00 and mess_string_list[0] != b' ':
                    response_message = bytearray(len(mess_string_list[0])+1)
                    response_message[0] = mess_id_list[0]
                    response_message[1:] = mess_string_list[0]
                    b.sendMessage(response_message)
            # We want to prioritze CAN messages over bluetooth messages, which
            # is why we have a 30ms sleep for this asynchronous function
            await uasyncio.sleep_ms(30)
        except Exception as e:
            print("Recording error from parse BT loop")
            sys.print_exception(e, error_log_file)
            error_log_file.flush()
            raise

# Function to handle all motor communication (choice was made to combine wrist
# motor communication and hand motor communication in same asynchronous
# function rather than separating them so that there is no concern of
# misalignment of frames where two classifications might accidentally occur at
# the same time)
#
# Inputs:
#   N/A
#
# Returns:
#   N/A
async def motorCommands():
    global ml, p_rec, c, b
    global dev_kit, manual_motors, need_to_enable, need_to_disable, collecting_data, class_training
    global true_rot_end_found, true_flex_end_found, home_wrist, train_hand, train_rotator
    global train_flexor, send_classifier_decision, true_rot_end_found, true_flex_end_found
    global loop_time,home_wrist,dev_kit,hand_homed,hand_timer,rotator_homed,flexor_homed
    global error_log_file, reboot_rotator, reboot_flexor
    global emg_gains,td_settings,wrist_settings,settings_dict, paused, is_training
    global train_hand_condition, train_rotator_condition, train_flexor_condition
    global moving_to_neutral, running_diagnostic, pyboard_reset
    global mess_id, mess_string, mess_id_list, mess_string_list, dbt_training_start_time, dbt_case
    global last_mess, rot_counter, flex_counter
    global done_moving_cw, done_moving_ccw, done_moving_ext, done_moving_flex, paused
    global endstops_file, endstops_dict, desired_locked_position, locking, prev_grip_trained
    global first_grip, num_grips_active, active_grips, num_mess, write_psyonic_flag, first_prep_frame

    # Initializing local variables specific to the motorCommands async function
    # Counting the number of frames since the pyboard turned on (used to avoid
    # issues with positional readings from the Dynamixel motors upon bootup)
    frame_counter = 0

    # A flag to signify if the motors have just been rebooted on the previous
    # frame
    just_rebooted = False
    
    # The motor ID of the motor that needed to reboot (should that occur)
    reboot_id = ""

    # A flag signifying whether or not the rotator has returned to a neutral
    # position
    rot_neutralized = False

    # A flag signifying whether or not the flexor has returned to a neutral
    # position
    flex_neutralized = False

    # A flag to determine whether or not the motors were enabled on the
    # previous frame
    just_enabled = False
    
    # Counter for how many frames it has been since you starting homing the
    # Psyonic hand
    psyonic_homing_counter = 0
    
    # Keeping track of the time it takes for the motorCommands function to run
    motor_command_time = 0

    # Setting booleans to initial states for whether or not you have hit the
    # physical endstops on this frame.
    # They start as false, and then might become true if triggered
    cw_end_found = False
    ccw_end_found = False
    ext_end_found = False
    flex_end_found = False

    # Run motor commands if you are not using a Development Kit
    if not dev_kit:
        # Run the initialization/boot up of the Dynamixel motors
        bootDynamixel()
        
        # Writing zero velocity to the hand/wrist to start since we don't want
        # any of the DOF's to be moving on boot up
        ml.hand_vel = 0
        ml.rot_vel = 0
        ml.flex_vel = 0
    
    while True:
        try:
            # Clocking the start time for each frame of the motorCommands loop
            motor_command_time_start = utime.ticks_us()
            
            # Assuming you are using a physical device
            if not dev_kit:
                # Handle Dynamixel enables, disables, and reboots
                [just_enabled,reboot_id,just_rebooted] = enableDisableRebootDynamixel(just_enabled,reboot_id,just_rebooted)
                
                ##############################################################
                ## THIS SECTION IS FOR WHEN THE USER SELECTS MOTORS ENABLED ##
                ##############################################################

                # Only perform the following commands like writing and reading
                # to and from the motor if torque is enabled
                # We are saying that if a user clicks the button "motors
                # enabled", then it signifies that they want all motors to be
                # enabled, not just the wrist motors (kind of confusing and
                # could probably use some better logic)
                if ml.wrist_motors_enabled:
                    # If you have just rebooted, then disable the motor,
                    # reset the indirect addresses (need to do this since they
                    # are wiped on reboot), and re-enable the motor.
                    if just_rebooted:
                        [just_rebooted,reboot_id] = actOnRebootedDyanmixel(just_rebooted,reboot_id)

                    # Otherwise, continue with the normal order of events
                    # (reading sensor data from the motors, writing positions,
                    # handling logic to do one of the following [manual motors,
                    # homing, normal classifier control, etc.])
                    else:
                        # Reading in both position and current values from both
                        # motors
                        try:
                            [just_enabled, frame_counter] = readDynamixel(just_enabled,frame_counter)
                            # Reset the number of bad readings in a row to 0 if
                            # you don't fail during the syncReadInd() call
                            ml.bad_readings = 0

                        # If there are any bad readings from syncReadInd(),
                        # set the present position/current values to their
                        # previous ones and increment the number of bad
                        # readings in a row
                        except Exception as e:
                            print(e)
                            print("BAD READINGS")
                            ml.bad_readings = ml.bad_readings + 1
                            ml.rot_pos = ml.prev_rot_pos
                            ml.flex_pos = ml.prev_flex_pos
                            ml.rot_curr = ml.prev_rot_curr
                            ml.flex_curr = ml.prev_flex_curr
                        await uasyncio.sleep_ms(0)

                        # If the number of bad readings, bad watchdogs, or
                        # errors exceeds a certain number of frames in a row,
                        # power down the wrist
                        [just_rebooted,reboot_id] = handleBadReadingsDyanmixel(just_rebooted,reboot_id)

                        # Keep a log of previous position/current values for
                        # Dynamixel motors as well as the grips of the Psyonic
                        # hand (should it be attached)
                        trackMotorHistory()

                        # Resetting these variables on every frame
                        cw_end_found = False
                        ccw_end_found = False
                        ext_end_found = False
                        flex_end_found = False

                        # No matter what state you are in (training, homing,
                        # normal operation, or manual motors), always check to
                        # see if you have hit an endstop
                        [cw_end_found,ccw_end_found,ext_end_found,flex_end_found] = checkEndstopsDynamixel(cw_end_found,ccw_end_found,ext_end_found,flex_end_found)

                        # Call the safety check function which will monitor if
                        # the maximum current is exceeded or if the position of
                        # the motor is beyond its virtual endstops
                        ml.safetyCheck()

                        # If you receive a button toggle from the socket switch
                        # that signifies the device based training to begin,
                        # reset all of the flags and variables
                        if c._dbt_toggle:
                            c._dbt_toggle = False
                            dbt_case = 0
                            home_wrist = True
                            train_hand = True
                            train_rotator = True
                            train_flexor = True
                            train_hand_condition = 3
                            train_rotator_condition = 3
                            train_flexor_condition = 3
                            first_nm = True
                            first_flex = True
                            first_ext = True
                            first_pro = True
                            first_sup = True
                            first_open = True
                            first_close = True
                            hand_increment = 0
                            c.sendStartTrain()
                            # Stop sending CAN while the device is homing and
                            # then start it again once data collection begins
                            c.sendStop()
                            hand_timer = utime.ticks_ms()
                        # If you receive a socket switch trigger that says to
                        # stop device based training, make sure to reset all of
                        # the relevant flags and variables
                        if c._stop_dbt_toggle:
                            c._stop_dbt_toggle = False
                            home_wrist = False
                            is_training = False
                            c.sendCancelTrain()
                            # Reset flags for next time you home
                            hand_homed = False
                            rotator_homed = False
                            flexor_homed = False
                            train_hand = False
                            train_rotator = False
                            train_flexor = False
                            rot_counter = 0
                            flex_counter = 0
                            done_moving_cw = False
                            done_moving_ccw = False
                            done_moving_ext = False
                            paused = False

                        # If the homing button has been selected, you are in
                        # the endstops routine
                        if home_wrist:
                            [psyonic_homing_counter,cw_end_found,ccw_end_found,ext_end_found,flex_end_found] = homeWrist(psyonic_homing_counter,cw_end_found,ccw_end_found,ext_end_found,flex_end_found)

                        # If you are in the process of training and haven't
                        # paused the calibration, first reset all velocities to
                        # be zero
                        elif is_training and not paused:
                            isTrainingAndNotPaused()

                        # If manual motors is selected, then don't reset any
                        # velocities to be zero and simply allow the motors to
                        # be driven with the prescribed velocity
                        elif manual_motors:
                            # Need this statement so that it doesn't fall into
                            # the "else" statement and reset all the velocities
                            # to be zero
                            pass

                        # If the classifier is on and you wish to run the wrist
                        # in normal operation, first reset the velocities to 0
                        # and then calculate the desired output velocities
                        # based on the class decision and prop speed.
                        elif send_classifier_decision:
                            ml.rot_vel = 0
                            ml.flex_vel = 0
                            ml.hand_vel = 0
                            # Assuming there is a classifier to be sent, send
                            # a classifier decision
                            if p_rec._classifier_available:
                                ml.sendClassifierDecision(ml.class_decision,ml.p_out)

                        # If you are trying to send the wrist manually to a
                        # neutral position or running a diagnostic check
                        elif moving_to_neutral or running_diagnostic:
                            movingNeutralRunningDiagnostic(rot_neutralized, flex_neutralized)

                        # If you are moving to a locked position for the flexor
                        elif locking:
                            lockingFlexor()
                            
                        # If none of these conditions are true, set the
                        # velocity of each motor to be 0 (i.e. no movement)
                        else:
                            ml.rot_vel = 0
                            ml.flex_vel = 0
                            ml.hand_vel = 0

                        # Try to write the velocities to the motors while
                        # passing in the information on whether or not each
                        # wrist motor has found its end stops
                        try:
                            ml.writeToMotors(true_rot_end_found,true_flex_end_found)
                            # If this uart.write is successful, then reset bad
                            # writings to 0
                            ml.bad_writings = 0
                        # If this fails, set velocities to be zero and
                        # increment the number of bad writings
                        except:
                            print("BAD WRITINGS")
                            ml.rot_vel = 0
                            ml.flex_vel = 0
                            ml.rot_pos_des = ml.rot_pos
                            ml.flex_pos_des = ml.flex_pos
                            ml.bad_writings = ml.bad_writings + 1
                        
                        # If the number of bad writings exceeds 10, disable the
                        # motors, write zero velocity to the hand, and raise an
                        # error
                        if ml.bad_writings > 10:
                            # Disable the motors to start to set initialize motor parameters
                            ml.mc_dyna.disableMotor(ml.mc_dyna.rotator_motor)
                            ml.rotator_disabled = True
                            ml.mc_dyna.disableMotor(ml.mc_dyna.flexor_motor)
                            ml.flexor_disabled = True
                            ml.wrist_motors_enabled = False
                            ml.hand_vel = 0
                            ml.writeHand(write_psyonic_flag)
                            a = 1/0

                        try:
                            ml.writeHand(write_psyonic_flag)
                        except:
                            print("unable to write to the psyonic hand")

                        # Don't write to the Psyonic hand if you are in the
                        # process of collecting data and reading in the local
                        # covariance matrix (in an attempt to limit how much
                        # computing power the Pyboard needs during that frame)
                        if first_prep_frame and collecting_data:
                            write_psyonic_flag = False
                        # Resetting the class out decision to be no movement
                        ml.c_out = 1
                        # Calculate the angle that the rotator and flexor are
                        # located at
                        mid_rot_pos = (ml.max_cw_pos + ml.max_ccw_pos)/2
                        ml.rot_angle = (ml.rot_pos-mid_rot_pos)*360/(1*4096)
                        mid_flex_pos = (ml.max_flex_pos + ml.max_ext_pos)/2
                        ml.flex_angle = (ml.flex_pos-mid_flex_pos)*360/(1*4096)

            # Tracking the motor command loop time
            motor_command_time = utime.ticks_us()-motor_command_time_start

            # if motor_command_time/1000 > 40:
            #print("motor command time: ",int(motor_command_time/1000))

            # Handles the sleep time for asynchronous functions
            if 40000-motor_command_time > 0 and motor_command_time >= 0:
                await uasyncio.sleep_ms(int((40000-motor_command_time)/1000))
            else:
                await uasyncio.sleep_ms(0)
        except Exception as e:
            print("Recording error from motor commands loop")
            sys.print_exception(e, error_log_file)
            error_log_file.flush()
            raise
           
# Function to handle all incoming CAN signals and handle calls to pattern
# recognition class
#
# Inputs:
#   N/A
#
# Returns:
#   N/A
async def handleFrames():
    global c, p_rec, b, c_log, ml
    global loop_time,home_wrist,dev_kit,hand_homed,hand_timer,rotator_homed,flexor_homed
    global collecting_data, class_training, send_classifier_decision, true_rot_end_found, true_flex_end_found
    global error_log_file, emg_gains,td_settings,wrist_settings,settings_dict
    global model_data, classifier_history, troubleshooting, pyboard_reset
    global mess_id, mess_string, mess_id_list, mess_string_list, paused
    global first_prep_frame, prev_frame_train, send_last_mess

    # A flag to allow us to use the ramps/votes or not
    use_ramps_and_votes = True
    # Counter to track every time a new frame of data is collected
    counter = 0
    # A byte array containing the class decision and prop speed for a frame
    ep_mess = bytearray(2)
    # A byte array containing the unsigned rotator position, a value for the
    # rotator sign, the unsigned flexor position, and a value for the flexor
    # sign
    motor_data = bytearray(5)
    # A byte array containing the step number of the troubleshooting process
    troubleshooting_data = bytearray(1)
    
    while True:
        try:
            # Keeping track of the timing of this aysnchronous function
            handle_frames_start_time = utime.ticks_us()

            ##################################################################
            ### THIS SECTION IS FOR SENDING INFO BETWEEN THE APP AND WRIST ###
            ##################################################################
            # Check to see if you are connected to Bluetooth
            if b._connected:
                # If you just reconnected to the Pyboard over Bluetooth, send
                # critical information so that the app is aware of the state of
                # the wrist/development kit (ex: are the motors enabled, has a
                # classifier been trained, etc.)
                if b._first_time_connected:
                    send_last_mess = True
                    if p_rec._classifier_available:
                        mess_id = 0xac
                        mess_string = b' Class Avail'
                        addMessageToList()
                    else:
                        mess_id = 0xac
                        mess_string = b' Class Unavail'
                        addMessageToList()
                    if send_classifier_decision:
                        mess_id = 0xbb
                        mess_string = b' Class Enabled'
                        addMessageToList()
                    else:
                        mess_id = 0xbb
                        mess_string = b' Class Disabled'
                        addMessageToList()
                    if true_rot_end_found:
                        mess_id = 0xae
                        mess_string = b' Rot True'
                        addMessageToList()
                    else:
                        mess_id = 0xae
                        mess_string = b' Rot False'
                        addMessageToList()
                    if true_flex_end_found:
                        mess_id = 0xae
                        mess_string = b' Flex True'
                        addMessageToList()
                    else:
                        mess_id = 0xae
                        mess_string = b' Flex False'
                        addMessageToList()
                    if ml.wrist_motors_enabled:
                        mess_id = 0xbc
                        mess_string = b' Mot Enabled'
                        addMessageToList()
                    else:
                        mess_id = 0xbc
                        mess_string = b' Mot Disabled'
                        addMessageToList()
                    if p_rec._use_raw_data:
                        mess_id = 0xbd
                        mess_string = b' Use raw'
                        addMessageToList()
                    else:
                        mess_id = 0xbd
                        mess_string = b' Do not use raw'
                        addMessageToList()
                    # If you just connected, you are not troubleshooting, so
                    # reset those variables
                    troubleshooting = False
                    troubleshooting_step_num = 0
                    # Send the available EMG channels
                    mess_id = 0xbf
                    mess_string = bytearray(p_rec._emg_chan_list)
                    addMessageToList()
                    # If an actual hand is attached, send handedness/td_type
                    if not dev_kit:
                        mess_id = 0xcb
                        if ml.td_type == 0:
                            if ml.handedness == 0:
                                mess_string = b' SingleLeft'
                                print("Left Ottobock")
                            elif ml.handedness == 1:
                                mess_string = b' SingleRight'
                                print("Right Ottobock")
                        elif ml.td_type == 1:
                            if ml.handedness == 0:
                                mess_string = b' MultiLeft'
                                print("Left Psyonic")
                            elif ml.handedness == 1:
                                mess_string = b' MultiRight'
                                print("Right Psyonic")
                        addMessageToList()
                    if dev_kit:
                        mess_id = 0xcc
                        mess_string = b' Dev Kit'
                        addMessageToList()
                    else:
                        mess_id = 0xcc
                        mess_string = b' Not Dev Kit'
                        addMessageToList()
                    b._first_time_connected = False
                # If the flag to send the saved classifier folders is on, then
                # send them. This will likely occur whenever we reset the
                # Pyboard, switch terminal devices, or save a new classifier.
                if b._send_saved_folders:
                    send_last_mess = True
                    work_dir = os.getcwd()
                    try:
                        if p_rec._td_type == "Ottobock":
                            os.chdir('/sd/ControllerData/ControllerDataSubFolder/Ottobock')
                        elif p_rec._td_type == "Psyonic":
                            os.chdir('/sd/ControllerData/ControllerDataSubFolder/Psyonic')
                        num_files = 0
                        tmp = os.listdir()
                        tmp.sort()
                        tmp.reverse()
                        # Sifting through the subdirectories to find the
                        # folders to send over (sends them one by one)
                        for x in tmp:
                            num_files = num_files + 1
                            full_file_name = x
                            no_prefix_file_name = full_file_name[4:]
                            mess_id = 0xaa
                            mess_string = bytearray(full_file_name)
                            print(full_file_name)
                            addMessageToList()
                            if num_files == 15:
                                num_files = 0
                                break
                        os.chdir(work_dir)
                        tmp.reverse()
                    except Exception as e:
                        print(e)
                        pass
                    b._send_saved_folders = False
                if send_last_mess:
                    print("should only print once (handleFrames)")
                    mess_id = 0xce
                    mess_string = b' No more mess'
                    addMessageToList()
                    send_last_mess = False
            
            # If the Bluetooth heartbeat doesn't occur rapidly enough, then
            # assume that we have lost connection and just disconnect it
            hb_time = b.getHeartbeatTime()
            if (utime.time() - hb_time) > 13 and hb_time != -1:
                b.disconnectBLE()

            ##################################################################
            #####  THIS SECTION IS FOR ACTING ON NEW FRAMES OF EMG DATA  #####
            ##################################################################
            # Check to see if we have new data
            flgV = c.checkFrameFlag()
            if flgV:
                # Handle the extraction of feature data from the incoming frame
                # of EMG data
                counter = counter + 1
                s_data = c.getFullFrame2()
                c.resetFrameFlag()
                p_rec.extractTDFeats(s_data)
                mav_bytes = p_rec.getMAVBytes()

                # Check if a classifier is available and that you are not in
                # the process of collecting data/training
                if p_rec.classifierAvailable() and not collecting_data:
                    # Get the class decision and prop speed
                    ml.c_out = p_rec.performClassification()
                    ml.p_out = p_rec.getPropSpeed()

                    # This section is only run if you want to use the ramps and
                    # votes post-processing. Otherwise, just take your class
                    # decision and prop speed as is
                    if use_ramps_and_votes:
                        # Applying the "votes" for different classifier
                        # decisions
                        for i in range(len(classifier_history)-1):
                            classifier_history[i] = classifier_history[i+1]
                        classifier_history[9] = ml.c_out

                        vote_list = [0,0,0,0,0,0,0]

                        # If the class decision is flexion/extension
                        if ml.c_out in [12,13]:
                            new_classification_count = 0
                            # Check the last "X" number of frames where X is
                            # the votes number
                            # Update a list that counts the number of
                            # instances of each class being classified in the
                            # last "X" frames
                            for i in range(len(classifier_history)-ml.flex_vote,len(classifier_history)):
                                classifier_value = classifier_history[i]
                                classifier_index = p_rec._class_mapper.index(classifier_value)
                                vote_list[classifier_index] += 1
                            # Choose the classification that has the highest
                            # number of instances as your class decision while
                            # still preserving the raw class decision as c_out
                            ml.class_decision = p_rec._class_mapper[np.argmax(vote_list)]
                            # Track the number of times the current decision
                            # has shown up in the last "Y" number of frames
                            # where Y is the ramps number
                            for i in range(len(classifier_history)-ml.flex_ramp,len(classifier_history)):
                                if classifier_history[i] == ml.c_out:
                                    new_classification_count = new_classification_count + 1
                            # Apply the ramps gain
                            ml.ramped_gain = new_classification_count/ml.flex_ramp
                            ml.p_out = int(ml.p_out*ml.ramped_gain)
                        # Do the same for rotation
                        elif ml.c_out in [10,11]:
                            new_classification_count = 0
                            for i in range(len(classifier_history) - ml.rot_vote, len(classifier_history)):
                                classifier_value = classifier_history[i]
                                classifier_index = p_rec._class_mapper.index(classifier_value)
                                vote_list[classifier_index] += 1
                            ml.class_decision = p_rec._class_mapper[np.argmax(vote_list)]
                            for i in range(len(classifier_history)-ml.rot_ramp,len(classifier_history)):
                                if classifier_history[i] == ml.c_out:
                                    new_classification_count = new_classification_count + 1
                            ml.ramped_gain = new_classification_count/ml.flex_ramp
                            ml.p_out = int(ml.p_out*ml.ramped_gain)
                        # Potentially add a similar section for the hand
                        else:
                            ml.class_decision = ml.c_out
                    else:
                        ml.class_decision = ml.c_out

                    # Assign the class decision and prop speed to the bytearray
                    ep_mess[0] = ml.class_decision
                    ep_mess[1] = ml.p_out

                # If there is no classifier available, or if you are collecting
                # new data, send -1 for each entry (we will handle these -1
                # entries in the app)
                else:
                    ep_mess[0] = 0xff
                    ep_mess[1] = 0xff

                # Update the motor positions/signs so that we can keep track of
                # the live location of each motor
                motor_data[0] = 0xaf
                motor_data[1] = int(math.fabs(ml.rot_angle))
                motor_data[2] = ml.rot_pos_sign
                motor_data[3] = int(math.fabs(ml.flex_angle))
                motor_data[4] = ml.flex_pos_sign

                # If you are connected to Bluetooth and it's every other frame,
                # send the EMG data, the class decision/prop speed, and the
                # motor position information
                if b.checkConnectionStatus() and counter%2==0 and not b.checkSendControllerStatus():
                    #print("sending EMG signals")
                    b.writeMessage(mav_bytes+ep_mess)
                    b.writeMessage(motor_data)        

                # Check to see if there is a new calibration entry, and if so,
                # and if you aren't troubleshooting, update the means and
                # covariances of the classifier model
                cal_status = p_rec.checkCalibrationStatus()
                # Only do this during the prep stage (cal_status == -1)
                if collecting_data and cal_status == -1:
                    start_time = time.ticks_ms()
                    # During your first frame of "prepping", read in the
                    # covariance matrix for the next class that will be trained
                    if first_prep_frame:
                        first_prep_frame = False
                        c.sendStop()
                        cwd = os.getcwd()
                        os.chdir('/sd/ControllerData')
                        [tmpCOV, tmpN] = p_rec._save_helper.readMN_COV(('COV' + str(p_rec._cov_index) + '.DAP'))
                        p_rec._cov = tmpCOV
                        c.sendStart()
                        os.chdir(cwd)
                    # If you are "prepping" and you just came from a state of
                    # having trained (i.e. not your first "prepping" message),
                    # then write the updated covariance matrix of the class
                    # that you just trained
                    if prev_frame_train:
                        c.sendStop()
                        prev_frame_train = False
                        print("p_rec._prev_cov_index: ",p_rec._prev_cov_index)
                        # This should never be -1 becuase you're previous
                        # covariance index should be one of the actual motions
                        if (p_rec._prev_cov_index == -1):
                            print("INDEX IS -1")
                        else:
                            p_rec._save_helper.writeMN_COV(p_rec._cov,p_rec._N[p_rec._prev_cov_index],('COV'+str(p_rec._prev_cov_index)+'.DAP'))
                            print("SAVED COV TO SD")
                        c.sendStart()
                # Assuming that you are not troubleshooting, you are not
                # paused during calibration, and that you are actually
                # collecting data and not just prepping, try to update the
                # means and covariance values
                if cal_status != -1 and not troubleshooting and not paused:
                    unable_to_collect = True
                    count = 0
                    # This while loop was put in to avoid any memory space
                    # issues, but it might not be necessary anymore
                    while(unable_to_collect):
                        try:
                            count = count + 1
                            start_time = time.ticks_ms()
                            # if cal_status == 6:
                            #     print("cal status 6")
                            start_time = time.ticks_ms()
                            await uasyncio.sleep_ms(0)
                            p_rec.updateMeanAndCov(cal_status)
                            await uasyncio.sleep_ms(0)
                            #print("time in updateMeanAndCov: ",(time.ticks_ms()-start_time))
                            # Resetting boolean to notify that the first frame
                            # of this motion is done being updated (this is
                            # used so that we only allocate covariance matrix
                            # heap memory once and not every frame of the
                            # training session)
                            unable_to_collect = False  
                        except Exception as e:
                            print(count)
                            print(e)
                            gc.collect() 
                            # print(count)
                            # print("allocated bytes: ",gc.mem_alloc())
                            # print("free bytes: ",gc.mem_free())
                            if count > 15:
                                print(e)
                                raise
                # If you are logging data (happens during calibration), then
                # log the data
                if c_log.checkWriting() and cal_status != -1:
                    c_log.writeData(s_data)
            else:
                pass
            
            # Clock the amount of time spent in this asynschronous function
            handle_frames_time = utime.ticks_us()-handle_frames_start_time
            #print("time in handle frame: ",int(handle_frames_time/1000))
            # Handles the sleep time for asynchronous functions
            if 40000-handle_frames_time > 0 and handle_frames_time >= 0:
                await uasyncio.sleep_ms(int((40000-handle_frames_time)/1000))
            else:
                await uasyncio.sleep_ms(0)
        except Exception as e:
            print("Recording error from handle frames loop")
            sys.print_exception(e, error_log_file)
            error_log_file.flush()
            raise            

###############################################################################

###############################################################################
###STANDARD FUNCTIONS###
###############################################################################

# This function will add a given message ID and string to the list of ID's and
# strings that need to be sent to the PAT App
#
# Inputs:
#   N/A
#
# Returns:
#   N/A
def addMessageToList():
    global mess_id, mess_string, mess_id_list, mess_string_list
    # Loop through the list of built up messages, and append the newest message
    # to the first one that is empty
    for i in range(len(mess_id_list)):
        if mess_id_list[i] == 0x00:
            mess_id_list[i] = mess_id
            mess_string_list[i] = mess_string
            break

# Handle device based training commands
#
# Inputs:
#   N/A
#
# Returns:
#   N/A
def deviceBasedTraining(mess):
    global collecting_data,class_training,c_log,p_rec,is_training,b,c, send_classifier_decision, calibration_counter
    global file_name_class, prev_grip_trained, first_prep_frame, prev_frame_train, valid_folder,sh,ml
    global cal_count_ottobock, cal_count_psyonic, cal_count_dict, cal_count_file, moving_to_neutral, hand_timer

    # The incoming message will contain a word that corresponds with a
    # specific motion, the waiting/prepping time prior to a motion, or
    # the completion of all motions ("Done").
    if "Prep" in mess:
        c.sendStop()
        class_training = "Prep"
        print(mess)
        
        # Do not log any EMG data during the prepping stage
        if c_log._file_open:
            c_log.closeFile()
        else:
            os.chdir('/sd')
        # Reset the flag to "collecting data" so that on the next
        # message (which will contain an actual motion), we know that
        # we are collecting data
        collecting_data = True
        if "No Movement" in mess:
            p_rec._cov_index = 0
            file_name_class = "_NO_MOVEMENT_"
        elif "Flexion" in mess:
            p_rec._cov_index = 1
            file_name_class = "_FLEXION_"
        elif "Extension" in mess:
            p_rec._cov_index = 2
            file_name_class = "_EXTENSION_"
        elif "Pronation" in mess:
            p_rec._cov_index = 3
            file_name_class = "_PRONATION_"
        elif "Supination" in mess:
            p_rec._cov_index = 4
            file_name_class = "_SUPINATION_"
        elif "Open" in mess:
            p_rec._cov_index = 5
            file_name_class = "_HAND_OPEN_"
        elif "Close" in mess:
            p_rec._cov_index = 6
            file_name_class = "_HAND_CLOSE_"
        elif "Grip0" in mess:
            p_rec._cov_index = 6
            file_name_class = p_rec._grip_dict[p_rec._grip_mapper["Grip0"]]
            prev_grip_trained = file_name_class
        elif "Grip1" in mess:
            p_rec._cov_index = 7
            file_name_class = p_rec._grip_dict[p_rec._grip_mapper["Grip1"]]
            prev_grip_trained = file_name_class
        elif "Grip2" in mess:
            p_rec._cov_index = 8
            file_name_class = p_rec._grip_dict[p_rec._grip_mapper["Grip2"]]
            prev_grip_trained = file_name_class
        elif "Grip3" in mess:
            p_rec._cov_index = 9
            file_name_class = p_rec._grip_dict[p_rec._grip_mapper["Grip3"]]
            prev_grip_trained = file_name_class
        elif "Robustness" in mess:
            p_rec._cov_index = 0
        p_rec.setCalibration(-1)
        os.chdir('/sd')
        c.sendStart()

    elif "No Movement" in mess:
        c.sendStop()
        p_rec._prev_cov_index = p_rec._cov_index
        class_training = "No Movement"
        print("No Movement")
        # Create a file name based on the motion and the date/time, and
        # begin logging the EMG data during this motion
        date_time = utime.localtime()
        c_log._file_name = "C_NO_MOVEMENT_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP" 
        c_log.openFile(c_log._file_name,"Calibrations","","")
        p_rec.setCalibration(0)
        first_prep_frame = True
        prev_frame_train = True
        c.sendStart()
    elif "Flexion" in mess:
        c.sendStop()
        p_rec._prev_cov_index = p_rec._cov_index
        class_training = "Flexion"
        print("Flexion")
        # Create a file name based on the motion and the date/time, and
        # begin logging the EMG data during this motion
        date_time = utime.localtime()
        c_log._file_name = "C_FLEXION_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP"
        c_log.openFile(c_log._file_name,"Calibrations","","")
        p_rec.setCalibration(1)
        first_prep_frame = True
        prev_frame_train = True
        c.sendStart()
    elif "Extension" in mess:
        c.sendStop()
        p_rec._prev_cov_index = p_rec._cov_index
        class_training = "Extension"
        print("Extension")
        # Create a file name based on the motion and the date/time, and
        # begin logging the EMG data during this motion
        date_time = utime.localtime()
        c_log._file_name = "C_EXTENSION_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP"
        c_log.openFile(c_log._file_name,"Calibrations","","")
        p_rec.setCalibration(2)
        first_prep_frame = True
        prev_frame_train = True
        c.sendStart()
    elif "Pronation" in mess:
        c.sendStop()
        p_rec._prev_cov_index = p_rec._cov_index
        class_training = "Pronation"
        print("Pronation")
        # Create a file name based on the motion and the date/time, and
        # begin logging the EMG data during this motion
        date_time = utime.localtime()
        c_log._file_name = "C_PRONATION_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP"
        c_log.openFile(c_log._file_name,"Calibrations","","")
        p_rec.setCalibration(3)
        first_prep_frame = True
        prev_frame_train = True
        c.sendStart()
    elif "Supination" in mess:
        c.sendStop()
        p_rec._prev_cov_index = p_rec._cov_index
        class_training = "Supination"
        print("Supination")
        # Create a file name based on the motion and the date/time, and
        # begin logging the EMG data during this motion
        date_time = utime.localtime()
        c_log._file_name = "C_SUPINATION_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP"
        c_log.openFile(c_log._file_name,"Calibrations","","")
        p_rec.setCalibration(4)
        first_prep_frame = True
        prev_frame_train = True
        c.sendStart()
    elif "Open" in mess:
        c.sendStop()
        p_rec._prev_cov_index = p_rec._cov_index
        class_training = "Open"
        print("Open")
        # Create a file name based on the motion and the date/time, and
        # begin logging the EMG data during this motion
        date_time = utime.localtime()
        c_log._file_name = "C_HAND_OPEN_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP"
        c_log.openFile(c_log._file_name,"Calibrations","","")
        p_rec.setCalibration(5)
        first_prep_frame = True
        prev_frame_train = True
        c.sendStart()
    elif "Close" in mess:
        c.sendStop()
        p_rec._prev_cov_index = p_rec._cov_index
        class_training = "Close"
        print("Close")
        # Create a file name based on the motion and the date/time, and
        # begin logging the EMG data during this motion
        date_time = utime.localtime()
        c_log._file_name = "C_HAND_CLOSE_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP"
        c_log.openFile(c_log._file_name,"Calibrations","","")
        p_rec.setCalibration(6)
        first_prep_frame = True
        prev_frame_train = True
        c.sendStart()
    elif "Grip0" in mess:
        c.sendStop()
        p_rec._prev_cov_index = p_rec._cov_index
        print("Grip0")
        class_training = p_rec._grip_dict[p_rec._grip_mapper["Grip0"]]
        print("class_training: ",class_training)
        # Create a file name based on the motion and the date/time, and
        # begin logging the EMG data during this motion
        date_time = utime.localtime()
        c_log._file_name = "C_" + class_training + "_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP"
        c_log.openFile(c_log._file_name,"Calibrations","","")
        p_rec.setCalibration(6)
        first_prep_frame = True
        prev_frame_train = True
        c.sendStart()
    elif "Grip1" in mess:
        c.sendStop()
        p_rec._prev_cov_index = p_rec._cov_index
        print("Grip1")
        class_training = p_rec._grip_dict[p_rec._grip_mapper["Grip1"]]
        print("class_training: ",class_training)
        # Create a file name based on the motion and the date/time, and
        # begin logging the EMG data during this motion
        date_time = utime.localtime()
        c_log._file_name = "C_" + class_training + "_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP"
        c_log.openFile(c_log._file_name,"Calibrations","","")
        p_rec.setCalibration(7)
        first_prep_frame = True
        prev_frame_train = True
        c.sendStart()
    elif "Grip2" in mess:
        c.sendStop()
        p_rec._prev_cov_index = p_rec._cov_index
        print("Grip2")
        class_training = p_rec._grip_dict[p_rec._grip_mapper["Grip2"]]
        print("class_training: ",class_training)
        # Create a file name based on the motion and the date/time, and
        # begin logging the EMG data during this motion
        date_time = utime.localtime()
        c_log._file_name = "C_" + class_training + "_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP"
        c_log.openFile(c_log._file_name,"Calibrations","","")
        p_rec.setCalibration(8)
        first_prep_frame = True
        prev_frame_train = True
        c.sendStart()
    elif "Grip3" in mess:
        c.sendStop()
        p_rec._prev_cov_index = p_rec._cov_index
        print("Grip3")
        class_training = p_rec._grip_dict[p_rec._grip_mapper["Grip3"]]
        print("class_training: ",class_training)
        # Create a file name based on the motion and the date/time, and
        # begin logging the EMG data during this motion
        date_time = utime.localtime()
        c_log._file_name = "C_" + class_training + "_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP"
        c_log.openFile(c_log._file_name,"Calibrations","","")
        p_rec.setCalibration(9)
        first_prep_frame = True
        prev_frame_train = True
        c.sendStart()
    elif "Robustness" in mess:
        c.sendStop()
        p_rec._prev_cov_index = p_rec._cov_index
        class_training = "Robustness"
        print("Robustness")
        p_rec.setCalibration(10)
        first_prep_frame = True
        prev_frame_train = True
        c.sendStart()
    elif "Done!!!" in mess:
        # Turn off all the booleans that let us know that we are collecting
        # data and training since we are done with the calibrations now.
        collecting_data = False
        is_training = False
        p_rec._is_training = False
        class_training = "Done!!!"
        print("Done!!!")
        prev_grip_trained = ""
        c.sendStop()
        os.chdir('/sd/ControllerData')
        if (p_rec.checkCalibrationStatus == -1):
            print("INDEX IS -1 IN CALIBRATION")
        else:
            p_rec._save_helper.writeMN_COV(p_rec._cov,p_rec._N[p_rec.checkCalibrationStatus()],('COV'+str(p_rec.checkCalibrationStatus())+'.DAP'))
        # Stop logging EMG data
        c_log.closeFile()
        
        p_rec.setCalibration(-1)
        first_prep_frame = True
        prev_frame_train = False
        # Since you are not in the troubleshooting process, stop CAN (to avoid
        # a buildup of messages), calculate the inverse covariance matrix,
        # and make the LDA classifier. Then start CAN again.
        gc.collect()
        unable_to_collect = True
        count = 0
        # This while loop was put in to avoid any memory space issues, but it
        # might not be necessary anymore
        while(unable_to_collect):
            try:
                count = count + 1
                p_rec.calculateInverseCovariance()
                p_rec.makeLDAClassifier()
                unable_to_collect = False
            except Exception as e:
                gc.collect() 
                print(count)
                if count > 10:
                    print(e)
                    raise
        print("just made the classifier")

        os.chdir('/sd')

        # Increment the calibration counter and write them to file
        calibration_counter = calibration_counter + 1
        if p_rec._td_type == "Ottobock":
            cal_count_ottobock = calibration_counter
            cal_count_dict["cal_count_ottobock"] = cal_count_ottobock
        elif p_rec._td_type == "Psyonic":
            cal_count_psyonic = calibration_counter
            cal_count_dict["cal_count_psyonic"] = cal_count_psyonic

        sh.writeDict(cal_count_file,cal_count_dict)

        [cal_count_ottobock,cal_count_psyonic,cal_count_dict] = sh.readCalCount(cal_count_file)

        # Read in the folder name from the Bluetooth message
        if calibration_counter < 10:
            folder_name = "00" + str(calibration_counter) + "_" + "DBT"
        elif calibration_counter < 100:
            folder_name = "0" + str(calibration_counter) + "_" + "DBT"
        else:
            folder_name = str(calibration_counter) + "_" + "DBT"

        # # Increment the dbt counter and write them to file
        # dbt_counter = dbt_counter + 1
        # dbt_counter_file = open('dbt_counter.txt','w')
        # dbt_counter_file.write(str(dbt_counter))
        # dbt_counter_file.close()

        # Try to save the controller data with this folder name. If the folder
        # name has already been used, it will return False, otherwise it will
        # return True.
        valid_folder = p_rec.saveController(folder_name)
        moving_to_neutral = True
        hand_timer = utime.ticks_ms()
        c.sendStart()
        
# This function handles the messages that get recieved over Bluetooth from the
# PAT App and filters them by the address they get sent with (e.g. 0x16 is a
# message that gives instructions on performing a calibration)
#
# Inputs:
#   N/A
#
# Returns:
#   N/A
def parseMessage(mess):
    global sh,b,c,c_log,p_rec,ml,rot_pos, rot_curr, flex_pos, flex_curr, loop_time
    global is_training, collecting_data, class_training, home_wrist, hand_timer, can_streaming
    global manual_motors, need_to_enable, need_to_disable, train_hand, train_rotator, train_flexor
    global send_classifier_decision, reboot_rotator, reboot_flexor, dev_kit, emg_gains
    global td_settings,wrist_settings,settings_dict, username, settings_file, paused
    global list_of_classes_to_reset, train_hand_condition, train_rotator_condition, train_flexor_condition
    global moving_to_neutral, troubleshooting, running_diagnostic
    global troubleshooting_description, troubleshooting_step, troubleshooting_step_num
    global first_nm, first_flex, first_ext, first_pro, first_sup, first_open, first_close, hand_increment
    global mess_id, mess_string, mess_id_list, mess_string_list
    global calibration_counter, tshoot_folder_name, new_tshoot_sequence
    global last_mess, train_hand, train_rotator, train_flexor, rot_counter, flex_counter
    global done_moving_cw, done_moving_ccw, done_moving_ext, done_moving_flex, paused
    global desired_locked_position, locking, true_rot_end_found, true_flex_end_found
    global first_prep_frame, prev_frame_train, prev_grip_trained
    global cal_count_ottobock, cal_count_psyonic, cal_count_dict, cal_count_file
    global first_grip, num_grips_active, active_grips, write_psyonic_flag

    # Heartbeet messages (messages with address 0x01) will often times be the
    # same message one after another, so don't include these messages when
    # tracking if two successive messages are identical
    if mess[0] != 0x01:
        last_mess = mess

    # A message that is sent on a regularly scheduled interval and is then
    # responded to in order to update the app with how long the heartbeat
    # message is taking to work
    if mess[0] == 0x01:
        b.updateHeartbeatTime()

    # Message containing a string to start recording the EMG data
    elif mess[0] == 0x10:
        save_name = mess[1:].decode() + '_' + str(utime.localtime()[0]) + '_' + str(utime.localtime()[1]) + '_' + str(utime.localtime()[2]) + '_' +  str(utime.localtime()[3]) + '_' + str(utime.localtime()[4]) + '_' + str(utime.localtime()[5]) + '.DAP' 
        resp_mess = bytearray(9)
        resp_mess[0] = 0x0b
        resp_mess[1] = 0x46
        resp_mess[2] = 0x69
        resp_mess[3] = 0x6c
        resp_mess[4] = 0x65
        resp_mess[5] = 0x53
        resp_mess[6] = 0x74
        resp_mess[7] = 0x61
        resp_mess[8] = 0x72
        b.sendMessage(resp_mess)
        print(resp_mess)
        c_log.openFile(save_name)

    # Message containing a string to stop recording the EMG data
    elif mess[0] == 0x11:
        resp_mess = bytearray(9)
        resp_mess[0] = 0x0b
        resp_mess[1] = 0x46
        resp_mess[2] = 0x69
        resp_mess[3] = 0x6c
        resp_mess[4] = 0x65
        resp_mess[5] = 0x53
        resp_mess[6] = 0x74
        resp_mess[7] = 0x6f
        resp_mess[8] = 0x70 

        b.sendMessage(resp_mess)
        print(resp_mess)
        c_log.closeFile()


    # Reserve for starting CAN
    # Byte format for message received:
    #       mess[0] = 0x12
    #       The rest of the message is not relevant, but a string
    #       saying "Start Can" is sent.
    #
    # There are no additional BLE messages that need to be sent back
    # to the app.
    elif mess[0] == 0x12:
        # Telling the CAN class to start reading in data from the CAN buffer
        c.sendStart()

        # This block of code is present in many places in this script. It is
        # used to send a response message to the PAT App to let it know that
        # the message was received.
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)

    # Reserve for stopping CAN
    # Byte format for message received:
    #       mess[0] = 0x13
    #       The rest of the message is not relevant, but a string
    #       saying "Stop Can" is sent.
    #
    # There are no additional BLE messages that need to be sent back
    # to the app.
    elif mess[0] == 0x13:
        # Telling the CAN class to stop reading in data from the CAN buffer
        c.sendStop()

        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)
    
    # Reserve for motors enable
    # Byte format for message received:
    #       mess[0] = 0x14
    #       The rest of the message is not relevant, but a string
    #       saying "Motors Disabled" is sent.
    #
    # There are no additional BLE messages that need to be sent back
    # to the app.
    elif mess[0] == 0x14:
        print("Enable Motors")

        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)
        
        # Check to see if you are operating the wrist or just a Dev Kit
        # If it is the wrist, set some flags on for needing to enable or
        # disable which will be referenced in the motorCommands function
        if not dev_kit:
            need_to_enable = True
            need_to_disable = False
        
    # Reserve for motors disable
    # Byte format for message received:
    #       mess[0] = 0x15
    #       The rest of the message is not relevant, but a string
    #       saying "Motors Disabled" is sent.
    #
    # There are no additional BLE messages that need to be sent back
    # to the app.
    elif mess[0] == 0x15:
        print("Motors are Disabled")

        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)
        
        # Check to see if you are operating the wrist or just a Dev Kit
        # If it is the wrist, set some flags on for needing to enable or
        # disable which will be referenced in the motorCommands function
        if not dev_kit:
            need_to_disable = True
            need_to_enable = False

        # If you receive a motor disable message while in the middle of any of
        # the following sequences (finding endstops, moving to neutral, or
        # running diagnostics), then send a message in response saying that
        # those sequences are completed, so that the PAT App isn't stuck on
        # those screens.
        if home_wrist:
            home_wrist = False
            mess_id = 0x0e
            mess_string = b' Done Homing'
            addMessageToList()
            # Reset flags for next time you home
            hand_homed = False
            rotator_homed = False
            flexor_homed = False
            train_hand = False
            train_rotator = False
            train_flexor = False
            rot_counter = 0
            flex_counter = 0
            done_moving_cw = False
            done_moving_ccw = False
            done_moving_ext = False
            done_moving_flex = False
            paused = False
        elif moving_to_neutral:
            moving_to_neutral = False
            mess_id = 0xad
            mess_string = b' Done Neutral'
            addMessageToList()
        elif running_diagnostic:
            running_diagnostic = False
            mess_id = 0xba
            mess_string = b' Done Diagnosing'
            addMessageToList()
      
    # Reserve for Calibrate
    # Byte format for message received:
    #       mess[0] = 0x16
    #       The rest of the message contains a string that corresponds
    #       to which part of the training session the user is in. This
    #       could be prepping, relaxing, pronating, etc.
    #
    # Response messages are sent after a contracting phase is completed.
    # This means that for Relaxing, Hand Opening, Hand Closing, Pronating,
    # Supinating, Flexing, and Extending, a response message is sent. It
    # has the following byte message array:
    #       mess[0] = 0xaa
    #       mess[1:] = b' Finished Collecting Data'  
    elif mess[0] == 0x16:
        # The incoming message will contain a word that corresponds with a
        # specific motion, the waiting/prepping time prior to a motion, or
        # the completion of all motions ("Done"). When it says "Prep" prior
        # to a data collection, it will also include the name of the DOF that
        # is coming up so that the Pyboard can make certain preparations for
        # said data collection
        if "Prep" in mess:
            # Stop listening to EMG data during the first frame of either the
            # prepping or data collection sessions because we need this frame
            # to do other computationally taxing processes, and if CAN is
            # streaming during this time, it could cause delays 
            c.sendStop()
            class_training = "Prep"
            write_psyonic_flag = False
            print("Prep")
            resp_mess = bytearray(len(mess))
            resp_mess[0] = 0x0b
            resp_mess[1:] = mess
            b.sendMessage(resp_mess)
            
            # Do not log any EMG data during the prepping stage
            if c_log._file_open:
                c_log.closeFile()
            else:
                os.chdir('/sd')
            # Reset the flag to "collecting data" so that on the next
            # message (which will contain an actual motion), we know that
            # we are collecting data
            # The name of the next class depends on what message was also
            # included in the prep message. For the different grips, we use
            # the device settings to map which number grip aligns with which
            # specific grip
            collecting_data = True
            if "No Movement" in mess:
                p_rec._cov_index = 0
                file_name_class = "_NO_MOVEMENT_"
            elif "Flexion" in mess:
                p_rec._cov_index = 1
                file_name_class = "_FLEXION_"
            elif "Extension" in mess:
                p_rec._cov_index = 2
                file_name_class = "_EXTENSION_"
            elif "Pronation" in mess:
                p_rec._cov_index = 3
                file_name_class = "_PRONATION_"
            elif "Supination" in mess:
                p_rec._cov_index = 4
                file_name_class = "_SUPINATION_"
            elif "Open" in mess:
                p_rec._cov_index = 5
                file_name_class = "_HAND_OPEN_"
            elif "Close" in mess:
                p_rec._cov_index = 6
                file_name_class = "_HAND_CLOSE_"
            elif "Grip0" in mess:
                p_rec._cov_index = 6
                file_name_class = p_rec._grip_dict[p_rec._grip_mapper["Grip0"]]
                prev_grip_trained = file_name_class
            elif "Grip1" in mess:
                p_rec._cov_index = 7
                file_name_class = p_rec._grip_dict[p_rec._grip_mapper["Grip1"]]
                prev_grip_trained = file_name_class
            elif "Grip2" in mess:
                p_rec._cov_index = 8
                file_name_class = p_rec._grip_dict[p_rec._grip_mapper["Grip2"]]
                prev_grip_trained = file_name_class
            elif "  Grip3" in mess:
                p_rec._cov_index = 9
                file_name_class = p_rec._grip_dict[p_rec._grip_mapper["Grip3"]]
                prev_grip_trained = file_name_class
            elif "Robustness" in mess:
                p_rec._cov_index = 0
            # Set the calibration value so that the pattern recognition script
            # knows that this "Prepping" class shouldn't have any impact on the
            # classifier
            p_rec.setCalibration(-1)
            # Set the variable first_prep_frame to be true so we know that we
            # have just started prepping (which is the best time to start
            # loading in the covariances for the upcoming class to be trained)
            first_prep_frame = True
            os.chdir('/sd')
            # Start the EMG streaming again once all of this logic has been
            # completed
            c.sendStart()
        elif "No Movement" in mess:
            c.sendStop()
            # We keep track of which class was just collected by using a time
            # history of the index that is used for selecting the correct
            # column of the covariance matrix (each column corresponds to a
            # different class)
            p_rec._prev_cov_index = p_rec._cov_index
            class_training = "No Movement"
            print("No Movement")
            resp_mess = bytearray(len(mess))
            resp_mess[0] = 0x0b
            resp_mess[1:] = mess
            b.sendMessage(resp_mess)

            # Don't log data or set calibration status if you are simply
            # troubleshooting
            if not troubleshooting:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                date_time = utime.localtime()
                c_log._file_name = "C_NO_MOVEMENT_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP" 
                c_log.openFile(c_log._file_name,"Calibrations","","")
                p_rec.setCalibration(0)
            else:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                if first_nm:
                    increment = "000"
                    first_nm = False
                else:
                    increment = "001"
                    first_nm = True
                date_time = utime.localtime()
                c_log._file_name = "TSHOOT_C_" + increment + "_NO_MOVEMENT_" + troubleshooting_description + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP" 
                c_log.openFile(c_log._file_name,"Troubleshooting",tshoot_folder_name,troubleshooting_step)
            # Keep track of the fact that the next time we enter the "Prepping"
            # or "Done" stages of calibration, we will have already collected
            # some data, meaning we should be writing these updated means and
            # covariances
            prev_frame_train = True
            c.sendStart()
        elif "Flexion" in mess:
            c.sendStop()
            p_rec._prev_cov_index = p_rec._cov_index
            class_training = "Flexion"
            write_psyonic_flag = False
            print("Flexion")
            resp_mess = bytearray(len(mess))
            resp_mess[0] = 0x0b
            resp_mess[1:] = mess
            b.sendMessage(resp_mess)

            # Don't log data or set calibration status if you are simply
            # troubleshooting
            if not troubleshooting:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                date_time = utime.localtime()
                c_log._file_name = "C_FLEXION_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP"
                c_log.openFile(c_log._file_name,"Calibrations","","")
                p_rec.setCalibration(1)
            else:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                if first_flex:
                    increment = "003"
                    first_flex = False
                else:
                    increment = "005"
                    first_flex = True
                date_time = utime.localtime()
                c_log._file_name = "TSHOOT_C_" + increment + "_FLEXION_" + troubleshooting_description + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP" 
                c_log.openFile(c_log._file_name,"Troubleshooting",tshoot_folder_name,troubleshooting_step)
            prev_frame_train = True
            c.sendStart()
        elif "Extension" in mess:
            c.sendStop()
            p_rec._prev_cov_index = p_rec._cov_index
            class_training = "Extension"
            write_psyonic_flag = False
            print("Extension")
            resp_mess = bytearray(len(mess))
            resp_mess[0] = 0x0b
            resp_mess[1:] = mess
            b.sendMessage(resp_mess)

            # Don't log data or set calibration status if you are simply
            # troubleshooting
            if not troubleshooting:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                date_time = utime.localtime()
                c_log._file_name = "C_EXTENSION_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP"
                c_log.openFile(c_log._file_name,"Calibrations","","")
                p_rec.setCalibration(2)
            else:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                if first_ext:
                    increment = "002"
                    first_ext = False
                else:
                    increment = "004"
                    first_ext = True
                date_time = utime.localtime()
                c_log._file_name = "TSHOOT_C_" + increment + "_EXTENSION_" + troubleshooting_description + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP" 
                c_log.openFile(c_log._file_name,"Troubleshooting",tshoot_folder_name,troubleshooting_step)
            prev_frame_train = True
            c.sendStart()
        elif "Pronation" in mess:
            c.sendStop()
            p_rec._prev_cov_index = p_rec._cov_index
            class_training = "Pronation"
            write_psyonic_flag = False
            print("Pronation")
            resp_mess = bytearray(len(mess))
            resp_mess[0] = 0x0b
            resp_mess[1:] = mess
            b.sendMessage(resp_mess)

            # Don't log data or set calibration status if you are simply
            # troubleshooting
            if not troubleshooting:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                date_time = utime.localtime()
                c_log._file_name = "C_PRONATION_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP"
                c_log.openFile(c_log._file_name,"Calibrations","","")
                p_rec.setCalibration(3)
            else:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                if first_pro:
                    increment = "006"
                    first_pro = False
                else:
                    increment = "008"
                    first_pro = True
                date_time = utime.localtime()
                c_log._file_name = "TSHOOT_C_" + increment + "_PRONATION_" + troubleshooting_description + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP" 
                c_log.openFile(c_log._file_name,"Troubleshooting",tshoot_folder_name,troubleshooting_step)
            prev_frame_train = True
            c.sendStart()
        elif "Supination" in mess:
            c.sendStop()
            p_rec._prev_cov_index = p_rec._cov_index
            class_training = "Supination"
            write_psyonic_flag = False
            print("Supination")
            resp_mess = bytearray(len(mess))
            resp_mess[0] = 0x0b
            resp_mess[1:] = mess
            b.sendMessage(resp_mess)

            # Don't log data or set calibration status if you are simply
            # troubleshooting
            if not troubleshooting:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                date_time = utime.localtime()
                c_log._file_name = "C_SUPINATION_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP"
                c_log.openFile(c_log._file_name,"Calibrations","","")
                p_rec.setCalibration(4)
            else:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                if first_sup:
                    increment = "007"
                    first_sup = False
                else:
                    increment = "009"
                    first_sup = True
                date_time = utime.localtime()
                c_log._file_name = "TSHOOT_C_" + increment + "_SUPINATION_" + troubleshooting_description + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP" 
                c_log.openFile(c_log._file_name,"Troubleshooting",tshoot_folder_name,troubleshooting_step)
            prev_frame_train = True
            c.sendStart()
        elif "Open" in mess:
            c.sendStop()
            p_rec._prev_cov_index = p_rec._cov_index
            class_training = "Open"
            if ml.td_type == 1:
                write_psyonic_flag = True
            else:
                write_psyonic_flag = False
            print("Open")
            resp_mess = bytearray(len(mess))
            resp_mess[0] = 0x0b
            resp_mess[1:] = mess
            b.sendMessage(resp_mess)

            # Don't log data or set calibration status if you are simply
            # troubleshooting
            if not troubleshooting:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                date_time = utime.localtime()
                c_log._file_name = "C_HAND_OPEN_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP"
                c_log.openFile(c_log._file_name,"Calibrations","","")
                p_rec.setCalibration(5)
            else:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                increment = "0" + str(hand_increment+10)
                hand_increment = hand_increment + 1
                date_time = utime.localtime()
                c_log._file_name = "TSHOOT_C_" + increment + "_HAND_OPEN_" + troubleshooting_description + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP" 
                c_log.openFile(c_log._file_name,"Troubleshooting",tshoot_folder_name,troubleshooting_step)
            prev_frame_train = True
            c.sendStart()
        elif "Close" in mess:
            c.sendStop()
            p_rec._prev_cov_index = p_rec._cov_index
            class_training = "Close"
            write_psyonic_flag = False
            print("Close")
            resp_mess = bytearray(len(mess))
            resp_mess[0] = 0x0b
            resp_mess[1:] = mess
            b.sendMessage(resp_mess)

            # Don't log data or set calibration status if you are simply
            # troubleshooting
            if not troubleshooting:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                date_time = utime.localtime()
                c_log._file_name = "C_HAND_CLOSE_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP"
                c_log.openFile(c_log._file_name,"Calibrations","","")
                p_rec.setCalibration(6)
            else:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                increment = "0" + str(hand_increment+10)
                hand_increment = hand_increment + 1
                date_time = utime.localtime()
                c_log._file_name = "TSHOOT_C_" + increment + "_HAND_CLOSE_" + troubleshooting_description + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP" 
                c_log.openFile(c_log._file_name,"Troubleshooting",tshoot_folder_name,troubleshooting_step)
            prev_frame_train = True
            c.sendStart()
        elif "Grip0" in mess:
            c.sendStop()
            p_rec._prev_cov_index = p_rec._cov_index
            print("Grip0")
            class_training = p_rec._grip_dict[p_rec._grip_mapper["Grip0"]]
            write_psyonic_flag = True
            print("class_training: ",class_training)
            resp_mess = bytearray(len(mess))
            resp_mess[0] = 0x0b
            resp_mess[1:] = mess
            b.sendMessage(resp_mess)

            # Don't log data or set calibration status if you are simply
            # troubleshooting
            if not troubleshooting:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                date_time = utime.localtime()
                c_log._file_name = "C_" + class_training + "_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP"
                c_log.openFile(c_log._file_name,"Calibrations","","")
                p_rec.setCalibration(6)
            else:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                increment = "0" + str(hand_increment+10)
                hand_increment = hand_increment + 1
                date_time = utime.localtime()
                c_log._file_name = "TSHOOT_C_" + increment + "_" + class_training + "_" + troubleshooting_description + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP" 
                c_log.openFile(c_log._file_name,"Troubleshooting",tshoot_folder_name,troubleshooting_step)
            prev_frame_train = True
            c.sendStart()
        elif "Grip1" in mess:
            c.sendStop()
            p_rec._prev_cov_index = p_rec._cov_index
            print("Grip1")
            class_training = p_rec._grip_dict[p_rec._grip_mapper["Grip1"]]
            write_psyonic_flag = True
            print("class_training: ",class_training)
            resp_mess = bytearray(len(mess))
            resp_mess[0] = 0x0b
            resp_mess[1:] = mess
            b.sendMessage(resp_mess)

            # Don't log data or set calibration status if you are simply
            # troubleshooting
            if not troubleshooting:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                date_time = utime.localtime()
                c_log._file_name = "C_" + class_training + "_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP"
                c_log.openFile(c_log._file_name,"Calibrations","","")
                p_rec.setCalibration(7)
            else:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                increment = "0" + str(hand_increment+10)
                hand_increment = hand_increment + 1
                date_time = utime.localtime()
                c_log._file_name = "TSHOOT_C_" + increment + "_" + class_training + "_" + troubleshooting_description + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP" 
                c_log.openFile(c_log._file_name,"Troubleshooting",tshoot_folder_name,troubleshooting_step)
            prev_frame_train = True
            c.sendStart()
        elif "Grip2" in mess:
            c.sendStop()
            p_rec._prev_cov_index = p_rec._cov_index
            print("Grip2")
            class_training = p_rec._grip_dict[p_rec._grip_mapper["Grip2"]]
            write_psyonic_flag = True
            print("class_training: ",class_training)
            resp_mess = bytearray(len(mess))
            resp_mess[0] = 0x0b
            resp_mess[1:] = mess
            b.sendMessage(resp_mess)

            # Don't log data or set calibration status if you are simply
            # troubleshooting
            if not troubleshooting:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                date_time = utime.localtime()
                c_log._file_name = "C_" + class_training + "_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP"
                c_log.openFile(c_log._file_name,"Calibrations","","")
                p_rec.setCalibration(8)
            else:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                increment = "0" + str(hand_increment+10)
                hand_increment = hand_increment + 1
                date_time = utime.localtime()
                c_log._file_name = "TSHOOT_C_" + increment + "_" + class_training + "_" + troubleshooting_description + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP" 
                c_log.openFile(c_log._file_name,"Troubleshooting",tshoot_folder_name,troubleshooting_step)
            prev_frame_train = True
            c.sendStart()
        elif "Grip3" in mess:
            c.sendStop()
            p_rec._prev_cov_index = p_rec._cov_index
            print("Grip3")
            class_training = p_rec._grip_dict[p_rec._grip_mapper["Grip3"]]
            write_psyonic_flag = True
            print("class_training: ",class_training)
            resp_mess = bytearray(len(mess))
            resp_mess[0] = 0x0b
            resp_mess[1:] = mess
            b.sendMessage(resp_mess)

            # Don't log data or set calibration status if you are simply
            # troubleshooting
            if not troubleshooting:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                date_time = utime.localtime()
                c_log._file_name = "C_" + class_training + "_" + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP"
                c_log.openFile(c_log._file_name,"Calibrations","","")
                p_rec.setCalibration(9)
            else:
                # Create a file name based on the motion and the date/time, and
                # begin logging the EMG data during this motion
                increment = "0" + str(hand_increment+10)
                hand_increment = hand_increment + 1
                date_time = utime.localtime()
                c_log._file_name = "TSHOOT_C_" + increment + "_" + class_training + "_" + troubleshooting_description + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP" 
                c_log.openFile(c_log._file_name,"Troubleshooting",tshoot_folder_name,troubleshooting_step)
            prev_frame_train = True
            c.sendStart()
        elif "Robustness" in mess:
            c.sendStop()
            p_rec._prev_cov_index = p_rec._cov_index
            write_psyonic_flag = True
            class_training = "Robustness"
            print("Robustness")
            resp_mess = bytearray(len(mess))
            resp_mess[0] = 0x0b
            resp_mess[1:] = mess
            b.sendMessage(resp_mess)
            p_rec.setCalibration(10)
            prev_frame_train = True
            c.sendStart()
        elif "Done!!!" in mess:
            # Turn off all the booleans that let us know that we are collecting
            # data and training since we are done with the calibrations now.
            collecting_data = False
            is_training = False
            p_rec._is_training = False
            class_training = "Done!!!"
            write_psyonic_flag = True
            hand_increment = 0
            prev_grip_trained = ""
            # Stop CAN since we are about to save the covariance variable
            c.sendStop()
            cwd = os.getcwd()
            os.chdir('/sd/ControllerData')
            # We do the saving here because we want to avoid any issues with
            # some variables being modified in this section and there being a
            # time delay before the saving can occur in the handleFrames
            # function. Basically, the variable collecting_data gets reset
            # here; however, we rely on it being True in order to record data
            # when we record data in the handleFrames function
            # Therefore, we can just do it here for this one frame,
            # (especially since there is no next motion whose covariance we
            # need to read in)
            if (p_rec.checkCalibrationStatus == -1):
                print("INDEX IS -1 IN CALIBRATION")
            else:
                p_rec._save_helper.writeMN_COV(p_rec._cov,p_rec._N[p_rec.checkCalibrationStatus()],('COV'+str(p_rec.checkCalibrationStatus())+'.DAP'))
            # Start CAN again now that saving is done
            c.sendStart()
            print("Done!!!")
            # Send the normal response message letting the app know that we
            # received the "Done!!!" message
            resp_mess = bytearray(len(mess))
            resp_mess[0] = 0x0b
            resp_mess[1:] = mess
            b.sendMessage(resp_mess)
            # Stop logging EMG data
            c_log.closeFile()
            os.chdir(cwd)
            p_rec.setCalibration(-1)
            # Reset these flags so they are both False prior to the next
            # calibration session
            first_prep_frame = False
            prev_frame_train = False
            # If you are not in the troubleshooting process, stop CAN (to avoid
            # a buildup of messages), calculate the inverse covariance matrix,
            # and make the LDA classifier. Then start CAN again.
            if not troubleshooting:
                c.sendStop()

                gc.collect()

                unable_to_collect = True
                count = 0

                # This while loop was put in to avoid any memory space issues, but it
                # might not be necessary anymore
                while(unable_to_collect):
                    try:
                        count = count + 1
                        p_rec.calculateInverseCovariance()
                        p_rec.makeLDAClassifier()
                        unable_to_collect = False
                    except Exception as e:
                        gc.collect() 
                        print(count)
                        if count > 10:
                            print(e)
                            raise
                c.sendStart()
                print("just made the classifier")
            # Send a message that you are done training.
            mess_id = 0x0e
            mess_string = b' Done Training'
            addMessageToList()
        c.sendStart()
        
    # Reserve for disconnecting bluetooth
    # Byte format for message received:
    #       mess[0] = 0x17
    #       The rest of the message is not relevant, but a string
    #       saying "Disconnect" is sent.
    #
    # There are no additional BLE messages that need to be sent back
    # to the app.
    elif mess[0] == 0x17:
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)

        # Disconnect the bluetooth connection
        b.disconnectBLE()

    # Reserve for Saving Pattern Rec Controller
    # Byte format for message received:
    #       mess[0] = 0x18
    #       The rest of the message is not relevant, but a string
    #       saying "Save Control" is sent.
    #
    # There are no additional BLE messages that need to be sent back
    # to the app.
    elif mess[0] == 0x18:
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)

        # Get the folder name out of the byte message
        string_mess = str(mess[1:])

        # Stop CAN so that messages don't get built up 
        c.sendStop()

        cwd = os.getcwd()
        os.chdir('/sd')

        # Read in the folder name from the Bluetooth message
        if calibration_counter < 10:
            folder_name = "00" + str(calibration_counter) + "_" + string_mess[2:len(string_mess)-1]
        elif calibration_counter < 100:
            folder_name = "0" + str(calibration_counter) + "_" + string_mess[2:len(string_mess)-1]
        else:
            folder_name = str(calibration_counter) + "_" + string_mess[2:len(string_mess)-1]
            
        # Increment the calibration counter and write them to file
        calibration_counter = calibration_counter + 1
        if p_rec._td_type == "Ottobock":
            cal_count_ottobock = calibration_counter
            cal_count_dict["cal_count_ottobock"] = cal_count_ottobock
        elif p_rec._td_type == "Psyonic":
            cal_count_psyonic = calibration_counter
            cal_count_dict["cal_count_psyonic"] = cal_count_psyonic

        # Update the dictionary with the new counter values
        sh.writeDict(cal_count_file,cal_count_dict)

        # Read them back in as a sanity check
        [cal_count_ottobock,cal_count_psyonic,cal_count_dict] = sh.readCalCount(cal_count_file)

        os.chdir(cwd)

        # Try to save the controller data with this folder name. If the folder
        # name has already been used, it will return False, otherwise it will
        # return True.
        valid_folder = p_rec.saveController(folder_name)

        # Since we have just saved a new folder, we should update the phone
        # with the new list of saved classifier folders
        b._send_saved_folders = True

        # Send a response message that tells the PAT App whether or not the
        # folder was valid.
        mess_id = 0x0f
        if not valid_folder:
            mess_string = b' Invalid folder'
        else:
            mess_string = b' Valid folder'
        addMessageToList()

        # Start CAN again
        c.sendStart()

    # Reserve for Resetting Pattern Rec
    # Byte format for message received:
    #       mess[0] = 0x19
    #       The rest of the message is not relevant, but a string
    #       saying "Reset Control" is sent.
    #
    # There are no additional BLE messages that need to be sent back
    # to the app.
    elif mess[0] == 0x19:
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)
        
        # Reset the controller and set the variable that signifies if the
        # classifier is "on" to False.
        p_rec.resetController()
        send_classifier_decision = False

    # Reserve for sending velocity to motors
    # Byte format for message received:
    #       mess[0] = 0x21
    #       mess[3] = "Vel: "
    #       mess[8:9] is the joint DOF (0 for rotator, 1 for flexor, 2 for
    #       hand)
    #       The rest of the message is reserved for the velocity of the
    #       joint DOF that has been selected.
    #
    # There are no additional BLE messages that need to be sent back
    # to the app.
    elif mess[0] == 0x21:
        str_mess = str(mess)

        # If the message contains the words "NOT WRITING VEL", and assuming
        # you are not using a Development Kit, then set all the motor
        # velocities to zero and set the manual motors flag to False.
        if "NOT WRITING VEL" in str_mess:
            resp_mess = bytearray(len(mess))
            resp_mess[0] = 0x0b
            resp_mess[1:] = mess
            b.sendMessage(resp_mess)
            if not dev_kit:
                print("DONE WITH MANUAL MOTORS")
                ml.rot_vel = 0
                ml.flex_vel = 0
                ml.hand_vel = 0
                manual_motors = False
        # Otherwise, assuming that you are not using a Development Kit, and
        # that the motors are enabled decode the message to determine which
        # degree of freedom is being referenced and what velocity you are to
        # send to it.
        else:
            print("MANUAL MOTORS")
            split_mess = str_mess[8:len(str_mess)-1].split(",")
            first_mess = split_mess[0]
            joint_dof = int(first_mess[0:1])
            velocity = float(first_mess[1:])
            resp_mess = bytearray(len(mess))
            resp_mess[0] = 0x0b
            resp_mess[1:] = mess
            b.sendMessage(resp_mess)
            
            if not dev_kit:                
                if ml.wrist_motors_enabled:
                    manual_motors = True
                    # The individual gain values for the various motors was
                    # determined experimentally
                    if joint_dof == 0:
                        # Account for right vs. left handedness when sending
                        # velocities to the rotator
                        if ml.handedness == 0:
                            velocity = velocity
                        else:
                            velocity = -1*velocity
                        # Scaling the velocity based on experimentation
                        ml.rot_vel = 3*velocity
                        ml.flex_vel = 0
                        ml.hand_vel = 0
                    elif joint_dof == 1:
                        # Scaling the velocity based on experimentation
                        ml.flex_vel = 1.5*velocity
                        ml.rot_vel = 0
                        ml.hand_vel = 0
                    elif joint_dof == 2:
                        ml.rot_vel = 0
                        ml.flex_vel = 0
                        if ml.td_type == 1:
                            ml.hand_vel = int(velocity*(255/100))
                            if velocity < 0:
                                grip_type_index = int(split_mess[1])
                                # Depending on which grip is being manually
                                # driven, update the current grip command
                                if grip_type_index == 0:
                                    ml.grip_command_type = ml.mc_psy.power_grasp_cmd
                                elif grip_type_index == 1:
                                    ml.grip_command_type = ml.mc_psy.key_grasp_cmd
                                elif grip_type_index == 2:
                                    ml.grip_command_type = ml.mc_psy.chuck_grasp_cmd
                                elif grip_type_index == 3:
                                    ml.grip_command_type = ml.mc_psy.pinch_grasp_cmd
                                elif grip_type_index == 4:
                                    ml.grip_command_type = ml.mc_psy.point_grasp_cmd
                                elif grip_type_index == 5:
                                    ml.grip_command_type = ml.mc_psy.sign_of_the_horns_grasp_cmd
                                elif grip_type_index == 6:
                                    ml.grip_command_type = ml.mc_psy.handshake_grip_cmd
                                elif grip_type_index == 7:
                                    ml.grip_command_type = ml.mc_psy.chuck_ok_grasp_cmd
                                elif grip_type_index == 8:
                                    ml.grip_command_type = ml.mc_psy.trigger_grip_cmd
                                elif grip_type_index == 9:
                                    ml.grip_command_type = ml.mc_psy.relax_cmd
                            else:
                                ml.grip_command_type = ml.mc_psy.general_open_cmd
                        else:
                            ml.hand_vel = velocity

    # Reserve for choosing which DOF's to train
    # Byte format for message received:
    #       mess[0] = 0x28
    #       mess[3:4] = 0 or 1 depending on if you want to train hand open
    #       mess[4:5] = 0 or 1 depending on if you want to train hand close
    #       mess[5:6] = 0 or 1 depending on if you want to train pronation
    #       mess[6:7] = 0 or 1 depending on if you want to train supination
    #       mess[7:8] = 0 or 1 depending on if you want to train flexion
    #       mess[8:9] = 0 or 1 depending on if you want to train extension
    #
    # After the wrist completes its homing sequence, it will send a response
    # message to the app with the following byte array:
    #
    #       mess[0] = 0x0d
    #
    # If both the rotator and the flexor were trained send the following
    # byte array message:
    #       mess[0] = 0x0c
    #       mess[1:] = b' Found Enstops'
    elif mess[0] == 0x28:        
        str_mess = str(mess)
        # Parse the message to determine which DOF's you are training
        training_nm = int(str_mess[3:4])
        training_hand_open = int(str_mess[4:5])
        training_hand_close = int(str_mess[5:6])
        training_pronation = int(str_mess[6:7])
        training_supination = int(str_mess[7:8])
        training_flexion = int(str_mess[8:9])
        training_extension = int(str_mess[9:10])
        first_grip = int(str_mess[10:len(str_mess)-1])

        # Quite a few booleans here, mainly because we need to be able to train
        # individual motions for a given DOF (e.g. supination but not
        # pronation)
        if training_nm == 1:
            p_rec._cov_index = 0
            file_name_class = "_NO_MOVEMENT_"
        elif training_flexion == 1:
            p_rec._cov_index = 1
            file_name_class = "_FLEXION_"
        elif training_extension == 1:
            p_rec._cov_index = 2
            file_name_class = "_EXTENSION_"
        elif training_pronation == 1:
            p_rec._cov_index = 3
            file_name_class = "_PRONATION_"
        elif training_supination == 1:
            p_rec._cov_index = 4
            file_name_class = "_SUPINATION_"
        elif training_hand_open == 1: 
            p_rec._cov_index = 5
            file_name_class = "_HAND_OPEN_"
        elif training_hand_close == 1:
            p_rec._cov_index = 6
            file_name_class = "_HAND_CLOSE_"
        # Map the value that came in with the correct grip to train
        elif first_grip != -1:
            if p_rec._grip_mapper["Grip0"] == first_grip:
                p_rec._cov_index = 6
                file_name_class = p_rec._grip_dict[p_rec._grip_mapper["Grip0"]]
                prev_grip_trained = file_name_class
            elif p_rec._grip_mapper["Grip1"] == first_grip:
                p_rec._cov_index = 7
                file_name_class = p_rec._grip_dict[p_rec._grip_mapper["Grip1"]]
                prev_grip_trained = file_name_class
            elif p_rec._grip_mapper["Gri2"] == first_grip:
                p_rec._cov_index = 8
                file_name_class = p_rec._grip_dict[p_rec._grip_mapper["Grip2"]]
                prev_grip_trained = file_name_class
            elif p_rec._grip_mapper["Grip3"] == first_grip:
                p_rec._cov_index = 9
                file_name_class = p_rec._grip_dict[p_rec._grip_mapper["Grip3"]]
                prev_grip_trained = file_name_class
        else:
            p_rec._cov_index = 0

        # If you are training either of the motions for a given DOF, set the
        # overall DOF as a being trained. If neither are motions are being
        # trained, mark that boolean as false.
        if training_hand_open == 1:
            train_hand_open = True
            train_hand = True
        else:
            train_hand_open = False
        if training_hand_close == 1:
            train_hand_close = True
            train_hand = True
        else:
            train_hand_close = False
        if not train_hand_open and not train_hand_close:
            train_hand = False


        if training_pronation == 1:
            train_pronation = True
            train_rotator = True
        else:
            train_pronation = False
        if training_supination == 1:
            train_supination = True
            train_rotator = True
        else:
            train_supination = False
        if not train_pronation and not train_supination:
            train_rotator = False


        if training_flexion == 1:
            train_flexion = True
            train_flexor = True
        else:
            train_flexion = False
        if training_extension == 1:
            train_extension = True
            train_flexor = True
        else:
            train_extension = False
        if not train_flexion and not train_extension:
            train_flexor = False
        
        # Just initializing these booleans. They represent whether you are
        # training one motion, the other motion, or both for a given DOF.
        train_hand_condition = 0
        train_rotator_condition = 0
        train_flexor_condition = 0

        # Setting all of these conditions given which motions are being
        # trained.
        if train_hand_open and not train_hand_close:
            train_hand_condition = 1
        elif train_hand_close and not train_hand_open:
            train_hand_condition = 2
        elif train_hand_open and train_hand_close:
            train_hand_condition = 3

        if train_pronation and not training_supination:
            train_rotator_condition = 1
        elif training_supination and not train_pronation:
            train_rotator_condition = 2
        elif train_pronation and training_supination:
            train_rotator_condition = 3

        if train_flexion and not training_extension:
            train_flexor_condition = 1
        elif training_extension and not train_flexion:
            train_flexor_condition = 2
        elif train_flexion and training_extension:
            train_flexor_condition = 3

        first_nm = True
        first_flex = True
        first_ext = True
        first_pro = True
        first_sup = True
        first_open = True
        first_close = True
        hand_increment = 0

        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)
        
        # Stop CAN to avoid message buildup
        c.sendStop()
        
        home_wrist = True

        # If you are using a Development Kit, if you are not using motor
        # guided mode (a custom setting to be set by clinicians that allows
        # you to not run the motors during training), or if none of the motions
        # are being trained, then just send a message in response to the PAT
        # App saying that you are done homing the device prior to training.
        # Otherwise, start the training by initializing the timer to tell the
        # hand to start moving.
        if dev_kit or not ml.motor_guided or (not train_flexion and not train_extension and not train_pronation and not train_supination and not train_hand_open and not train_hand_close) or (not train_flexor and not train_rotator and not train_hand):
            mess_id = 0x0d
            mess_string = b' Done Homing'
            addMessageToList()
            home_wrist = False
            is_training = True
            p_rec._is_training = True
            paused = False
            c.sendStart()
        else:
            hand_timer = utime.ticks_ms()
            if ml.td_type == 1:
                # Check to see if you are training hand open, hand close, or
                # both, and if the terminal device is multi DOF, update the
                # grip command
                if train_hand_condition in [1,3]:
                    if first_grip == 0:
                        ml.grip_command_type = ml.mc_psy.power_grasp_cmd
                        ml.hand_vel = 50
                    elif first_grip == 1:
                        ml.grip_command_type = ml.mc_psy.key_grasp_cmd
                        ml.hand_vel = 50
                    elif first_grip == 2:
                        ml.grip_command_type = ml.mc_psy.chuck_grasp_cmd
                        ml.hand_vel = 50
                    elif first_grip == 3:
                        ml.grip_command_type = ml.mc_psy.pinch_grasp_cmd
                        ml.hand_vel = 50
                    elif first_grip == 4:
                        ml.grip_command_type = ml.mc_psy.point_grasp_cmd
                        ml.hand_vel = 50
                    elif first_grip == 5:
                        ml.grip_command_type = ml.mc_psy.sign_of_the_horns_grasp_cmd
                        ml.hand_vel = 50
                    elif first_grip == 6:
                        ml.grip_command_type = ml.mc_psy.trigger_grip_cmd
                        ml.hand_vel = 50
                    elif first_grip == 7:
                        ml.grip_command_type = ml.mc_psy.handshake_grip_cmd
                        ml.hand_vel = 50
                    elif first_grip == 8:
                        ml.grip_command_type = ml.mc_psy.chuck_ok_grasp_cmd
                        ml.hand_vel = 50
                elif train_hand_condition == 2:
                    ml.grip_command_type = ml.mc_psy.general_open_cmd
                    ml.hand_vel = 125
       
    # Reserve for determining whether or not to disable any of the DOFs 
    #       mess[0] = 0x30
    #       mess[3:4] = either 0 or 1 (OFF or ON for rotator)
    #       mess[4:5] = either 0 or 1 (OFF or ON for flexor)
    #       mess[5:6] = either 0 or 1 (OFF or ON for hand)
    #
    # There are no additional BLE messages that need to be sent back
    # to the app.     
    elif mess[0] == 0x30:
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)

        str_mess = str(mess)

        # Decode the messages to confirm whether or not we want to activate or
        # deactivate a given DOF
        flex_activity = int(str_mess[3:4])
        rot_activity = int(str_mess[4:5])
        grip0_activity = int(str_mess[5:6])
        grip1_activity = int(str_mess[6:7])
        grip2_activity = int(str_mess[7:8])
        grip3_activity = int(str_mess[8:9])

        # Set booleans for whether or each DOF is enabled based on the decoded
        # message
        if not dev_kit:
            if flex_activity:
                ml.flex_activated = True
            else:
                ml.flex_activated = False
            if rot_activity:
                ml.rot_activated = True
            else:
                ml.rot_activated = False
            if grip0_activity:
                ml.hand_activated = True
            else:
                ml.hand_activated = False
    # Reserve for classifier enable
    # Byte format for message received:
    #       mess[0] = 0x31
    #       The rest of the message is not relevant, but a string
    #       saying "Classifier Enable" is sent.
    #
    # There are no additional BLE messages that need to be sent back
    # to the app.  
    elif mess[0] == 0x31:
        print("Classifier is Enabled")
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)
        
        # Setting the boolean on that represents whether or not the classifier
        # will be acting on the wrist (assuming a classifier is available)
        send_classifier_decision = True
        
    # Reserve for classifier disable
    # Byte format for message received:
    #       mess[0] = 0x32
    #       The rest of the message is not relevant, but a string
    #       saying "Classifier Disable" is sent.
    #
    # There are no additional BLE messages that need to be sent back
    # to the app.
    elif mess[0] == 0x32:
        print("Classifier is Disabled")
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)
        
        # Setting the boolean off that represents whether or not the classifier
        # will be acting on the wrist (assuming a classifier is available)
        send_classifier_decision = False

    # Reserve for selecting username
    # Byte format for message received:
    #       mess[0] = 0x36
    #       The rest of the message is a string that contains
    #       the name of the user.
    #
    # There are no additional BLE messages that need to be sent back
    # to the app. 
    elif mess[0] == 0x36:
        
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)
        
        # get the username out of the byte message
        username = str(mess[1:])[2:-1]
        print("Username: " + str(username))

    # Reserve for pausing/playing data collection
    # Byte format for message received:
    #       mess[0] = 0x37
    #       The rest of the message is a string that contains
    #       the word "Pause" or "Play".
    #
    # There are no additional BLE messages that need to be sent back
    # to the app.
    elif mess[0] == 0x37:
        
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)
        
        string_mess = str(mess)

        # Set a "paused" boolean on or off so that the wrist will stop if need
        # be during a training
        if "Pause" in string_mess:
            paused = True
        elif "Play" in string_mess:
            paused = False

    # Reserve for sending custom calibration settings
    # Byte format for message received:
    #       mess[0] = 0x38
    #       The rest of the message is a string that contains
    #       individual settings values in each byte.
    #
    # There are no additional BLE messages that need to be sent back
    # to the app.
    elif mess[0] == 0x38:
        
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)

        string_mess = str(mess[1:])

        # Set the calibration pace variable to either slow, medium, or fast.
        # This variable determines how long the prepping stage lasts during
        # calibration.
        if string_mess[2:3] == "0":
            ml.calibration_time = 1
        elif string_mess[2:3] == "1":
            ml.calibration_time = 2
        elif string_mess[2:3] == "2":
            ml.calibration_time == 4

        # Setting on/off the adaptation variable for calibration (not sure
        # what this variable actually does, and it is not implemented 
        # anywhere)
        if string_mess[3:4] == "3":
            ml.adaptation = True
        elif string_mess[3:4] == "4":
            ml.adaptation = False
        if string_mess[3:4] == "5":
            print("Not sure what 'reset' means")

        # Setting on or off a flag that tells the wrist whether or not to move
        # during calibration
        if string_mess[4:5] == "0":
            ml.motor_guided = True
            print("Guide motors")
        elif string_mess[4:5] == "1":
            ml.motor_guided = False
            print("Do not guide motors")

        # Setting on or off a flag that tells the wrist whether or not to be
        # enabled once the calibration has been completed.
        if string_mess[5:6] == "0":
            ml.enabled_on_complete = True
        elif string_mess[5:6] == "1":
            ml.enabled_on_complete = False

    # Reserve for sending reset calibration message
    # Byte format for message received:
    #       mess[0] = 0x39
    #       The rest of the message is a string that contains
    #       comma separated classes to reset.
    #
    # There are no additional BLE messages that need to be sent back
    # to the app.
    elif mess[0] == 0x39:
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)

        # Stop CAN to avoid message buildup
        c.sendStop()

        # This set of logic will read in the message, which is an array
        # containing which classes need to be reset. If a class number exists
        # in this array, we add it to a list containing the classes that we
        # need to reset. This is basically just code that reads in a big string
        # and reformats it so it's useful.
        list_of_classes_to_reset = []
        string_mess = str(mess[1:])
        res = [i for i in range(len(string_mess)) if string_mess.startswith(",", i)]
        #print(res)
        if res == []:
            list_of_classes_to_reset.append(int(string_mess[2]))
            pass
        else:
            for i in range(len(res)):
                if i == 0:
                    start_index = 2
                else:
                    start_index = res[i-1]+1
                end_index = res[i]
                mess = string_mess[start_index:end_index]
                #print(mess)
                list_of_classes_to_reset.append(int(mess))
                if i == len(res)-1:
                    start_index = res[i]+1
                    end_index = len(string_mess)-1
                    mess = string_mess[start_index:end_index]
                    #print(mess)
                    list_of_classes_to_reset.append(int(mess))

        print("list of classes to reset: ",list_of_classes_to_reset)

        if len(list_of_classes_to_reset) == p_rec._num_classes:
            # Reset the controller and set the variable that signifies if the
            # classifier is on to False.
            p_rec.resetController()
            send_classifier_decision = False
        else:
            # Calling the function from the patternRec class to actually reset the
            # specific classes of interest.
            p_rec.resetSpecificClasses(list_of_classes_to_reset)

        # Turn on CAN
        c.sendStart()

    # Reserve for alerting the wrist to use a specific classifier.
    # Byte format for message received:
    #       mess[0] = 0x43
    #       The rest of the message is a string that contains the
    #       name of the folder of interest.
    #
    # There are no additional BLE messages that need to be sent back
    # to the app.
    elif mess[0] == 0x43:
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)
        
        string_mess = str(mess[1:])

        # Decode the string name for the folder from the bluetooth message
        folder_name = string_mess[2:len(string_mess)-1]

        # Stop CAN to avoid message buildup
        c.sendStop()
        # Reset the controller since we need to load in a different, saved
        # calibration
        p_rec.resetController()

        # Reset the classifier variables and specify the sub-directory to look
        # into in for pulling the classifier variables in from
        p_rec._cov = []
        p_rec._means = []
        p_rec._inv_pooled_cov = np.zeros((p_rec._total_num_feats,p_rec._total_num_feats))
        work_dir = os.getcwd()
        if p_rec._td_type == 'Ottobock':
            path = '/sd/ControllerData/ControllerDataSubFolder/Ottobock/' + folder_name
        elif p_rec._td_type == 'Psyonic':
            path = '/sd/ControllerData/ControllerDataSubFolder/Psyonic/' + folder_name
        #print("path: " + path)
        os.chdir(path)

        gc.collect()

        unable_to_collect = True
        count = 0

        # This while loop was put in to avoid any memory space issues, but it
        # might not be necessary anymore
        while(unable_to_collect):
            try:
                count = count + 1
                # Load in the classifier model variables
                for x in range(p_rec._num_classes):
                  #[tmpCOV,tmpN] = sh.readMN_COV(('COV'+str(x)+'.DAP'))
                  [tmpMN,tmpN] = sh.readMN_COV(('MN'+str(x)+'.DAP'))
                  #print(tmpCOV.shape())
                  #p_rec._cov.append(tmpCOV)
                  p_rec._means.append(tmpMN)
                  p_rec._N.append(tmpN[0])
                unable_to_collect = False  
            except Exception as e:
                gc.collect() 
                print(count)
                if count > 10:
                    print(e)
                    raise

        # Read in the inverse pooled covriance matrix
        [tmpINVCOV,tmpN]  = sh.readMN_COV(('INV_COV.DAP'))
        p_rec._inv_pooled_cov = tmpINVCOV

        # Read in the weights and offsets
        p_rec._wg = sh.readFloatMat('WG.DAP')
        p_rec._cg = sh.readFloatMat('CG.DAP')

        # Read in and set the threshold data
        NM_thresh_data = sh.readFloatMat('NM_Thresh.txt')
        p_rec._avg_NM_threshold = NM_thresh_data[0][0]
        p_rec._num_NM_threshold_examples = NM_thresh_data[0][1]

        # Read in and set the classes that have been trained
        class_list_data = sh.readFloatMat('Class_List.DAP')
        p_rec._class_list = []
        for x in range(p_rec._num_classes):
            p_rec._class_list.append(int(class_list_data[0,x]))

        # Read in the saved grip information
        grip_data = sh.readFloatMat('Grip_Data.DAP')
        p_rec._class_grip_data = []
        for x in range(10):
            p_rec._class_grip_data.append(int(grip_data[0,x]))
            if x < 5:
                ml.grip[x] = int(grip_data[0,x])
            else:
                ml.grip_speed[x] = int(grip_data[0,x])

        # Send the loaded grip information back to the wrist
        mess_id = 0xcd
        mess_string = bytearray(p_rec._class_grip_data)
        addMessageToList()

        p_rec._classifier_available = True

        os.chdir(work_dir)
        # Start CAN again
        c.sendStart()


    # Reserve for choosing which EMG channels to use
    # Byte format for message received:
    #       mess[0] = 0x44
    #       mess[1:end] = individual int's in a row that correspond to which
    #                     EMG channels to use
    elif mess[0] == 0x44:
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)

        str_mess = str(mess)

        emg_list = []

        for i in range(3,len(str_mess)-1):
            emg_chan = int(str_mess[i:i+1])
            emg_list.append(emg_chan)

        emg_list.sort()
        # Update the list of EMG channels to be used in creating a classifier
        p_rec._emg_chan_list = emg_list
        print("p_rec._emg_chan_list: ",p_rec._emg_chan_list)

        # Stop CAN to avoid message buildup
        c.sendStop()
        # Remake the classifier (not the inverse pooled covariance matrix) now
        # that you have a new set of channels to use
        p_rec.makeLDAClassifier()
        # Start CAN again
        c.sendStart()

        # Update the wrist that this process has been completed
        mess_id = 0xab
        mess_string = b' EMGs Updated'
        addMessageToList()

    # Reserve for setting use_raw_data filter on/off
    # Byte format for message received:
    #       mess[0] = 0x45
    #       mess[1:end] = either "True" or "False"
    elif mess[0] == 0x45:
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)

        str_mess = str(mess)

        # Toggle on and off the user_raw_data flag
        if "True" in str_mess:
            p_rec._use_raw_data = True
        elif "False" in str_mess:
            p_rec._use_raw_data = False

    # Reserve for setting robustness threshold gain
    # Byte format for message received:
    #       mess[0] = 0x46
    #       mess[1:end] = robustness threshold gain
    elif mess[0] == 0x46:
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)

        str_mess = str(mess)

        # Update the robustness threshold gain
        p_rec._thresh_gain = float(str_mess[3:len(str_mess)-1])
        print("thresh gain: " + str(p_rec._thresh_gain))

    # Reserve for sending wrist to neutral position
    # Byte format for message received:
    #       mess[0] = 0x48
    #       mess[1:end] = "Send Neutral"
    elif mess[0] == 0x48:
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)

        moving_to_neutral = True

        # If you are using a development kit or don't want to drive motors,
        # then just send a message saying that you're done setting to neutral.
        if dev_kit or not ml.motor_guided:
            mess_id = 0xad
            mess_string = b' Done Neutral'
            moving_to_neutral = False
            addMessageToList()
        # Otherwise, begin the process by closing the hand.
        else:
            hand_timer = utime.ticks_ms()
            if ml.td_type == 1:
                ml.grip_command_type = ml.mc_psy.relax_cmd
                ml.hand_vel = 125

    # Reserve for notifying if you are troubleshooting or not
    # Byte format for message recieved:
    #       mess[0] = 0x49
    #       mess[1:2] = 0 or 1 for True or False
    elif mess[0] == 0x49:
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)

        string_mess = str(mess[1:])
        troubleshooting_val = string_mess[2:3]

        # Update the troubleshooting flag
        if troubleshooting_val == "1":
            troubleshooting = True
        elif troubleshooting_val == "0":
            troubleshooting = False

    # Reserve for notifying if you are doing a diganostic check
    # Byte format for message received:
    #       mess[0] = 0x50
    #       mess[1:end] = b' Diagnostic'
    elif mess[0] == 0x50:
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)

        running_diagnostic = True
        true_rot_end_found = False
        true_flex_end_found = False

        # If you are using a development kit or don't want to drive motors,
        # then just send a message saying that you're done running a
        # diagnostic check. 
        if dev_kit or not ml.motor_guided:
            mess_id = 0xba
            mess_string = b' Done Diagnosing'
            addMessageToList()
        # Otherwise, begin the process by closing the hand.    
        else:
            hand_timer = utime.ticks_ms()

    # Reserve for notifying you which troubleshooting step you are on
    # Byte format for message received:
    #       mess[0] = 0x51
    #       mess[1:2] = an integer that represents which step of the
    #                   troubleshooting protocol you are on
    elif mess[0] == 0x51:
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)

        troubleshooting = True

        string_mess = str(mess[1:])
        troubleshooting_step = string_mess[2:len(string_mess)-1]
        troubleshooting_step_num = int(troubleshooting_step)

        # Decode the string that comes through the message and then create a
        # file name that is specific to what step of the troubleshooting
        # sequence you are on
        if troubleshooting_step in ["0","1","2"]:
            if troubleshooting_step == "0":
                troubleshooting_description = "CAL_ARM_SUPPORTED_"
                if new_tshoot_sequence:
                    date_time = utime.localtime()
                    year = str(date_time[0])
                    if date_time[1] < 10:
                        month = "0"+str(date_time[1])
                    else:
                        month = str(date_time[1])
                    if date_time[2] < 10:
                        day = "0" + str(date_time[2])
                    else:
                        day = str(date_time[2])
                    if date_time[3] < 10:
                        hour = "0" + str(date_time[3])
                    else:
                        hour = str(date_time[3])
                    if date_time[4] < 10:
                        minute = "0" + str(date_time[4])
                    else:
                        minute = str(date_time[4])
                    if date_time[5] < 10:
                        second = "0" + str(date_time[5])
                    else:
                        second = str(date_time[5])
                    tshoot_folder_name = year + month + day + "_" + hour + minute + second
                    new_tshoot_sequence = False
            elif troubleshooting_step == "1":
                troubleshooting_description = "CAL_ARM_DOWN_SIDE_"
            elif troubleshooting_step == "2":
                troubleshooting_description = "CAL_ARM_FRONT_"
        elif troubleshooting_step in ["3","4","5","6","7","8","9"]:
            if troubleshooting_step == "3":
                troubleshooting_description = "000_REC_ARM_DOWN_SIDE_"
            elif troubleshooting_step == "4":
                troubleshooting_description = "001_REC_ARM_FRONT_"
            elif troubleshooting_step == "5":
                troubleshooting_description = "002_REC_ARM_SIDE_"
            elif troubleshooting_step == "6":
                troubleshooting_description = "003_REC_DIAGNOAL_SWEEPS_"
            elif troubleshooting_step == "7":
                troubleshooting_description = "004_REC_SIDE_SWEEPS_"
            elif troubleshooting_step == "8":
                troubleshooting_description = "005_REC_PUSH_SOCKET_"
            elif troubleshooting_step == "9":
                troubleshooting_description = "006_REC_PULL_SOCKET_"
            # Create a file name based on the motion and the date/time, and
            # begin logging the EMG data during this motion
            date_time = utime.localtime()
            c_log._file_name = "TSHOOT_" + troubleshooting_description + str(date_time[0]) + "_" + str(date_time[1]) + "_" + str(date_time[2]) + "_" + str(date_time[3]) + "_" + str(date_time[4]) + "_" + str(date_time[5]) + ".DAP" 
            c_log.openFile(c_log._file_name,"Troubleshooting",tshoot_folder_name,troubleshooting_step)

    # Reserve for notifying if you are done troubleshooting
    # Byte format for message received:
    #       mess[0] = 0x52
    #       mess[1:] = b' Done'
    elif mess[0] == 0x52:
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)
    
        troubleshooting = False

        # Stop logging EMG data
        c_log.closeFile()
        troubleshooting_step = 0
        troubleshooting_description = ""

    # Reserve for acknowledgement of message received by the PAT App
    # Byte format for message received:
    #       mess[0] = 0x53
    #       mess[1:] = b' Received'
    elif mess[0] == 0x53:
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)

        # # Create new message to respond to phone saying that message 0x53 was
        # # received
        # mess_id = 0x53
        # mess_string = b' '
        # If you have received this acknolwedgmenet message, then the phone has
        # received your previous message, so you can remove it from the list of
        # messages that need to be sent out
        for i in range(len(mess_id_list)-1):
            mess_id_list[i] = mess_id_list[i+1]
            mess_string_list[i] = mess_string_list[i+1]
        mess_id_list[len(mess_id_list)-1] = 0x00
        mess_string_list[len(mess_string_list)-1] = b' '

    # Reserve for reading in date time to set RTC
    # Byte format for message received:
    #       mess[0] = 0x54
    #       The rest of the message is a string that contains comma
    #       separated datetime entries.
    elif mess[0] == 0x54:
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)

        # This set of logic will read in the message, which is an array
        # containing information related to the date and time. It will be used
        # to set the real time clock, since the real time clock gets reset on
        # every power cycleThis is basically just code that reads in a big
        # string and reformats it so it's useful.
        date_time = []
        string_mess = str(mess[1:])
        res = [i for i in range(len(string_mess)) if string_mess.startswith(",", i)]

        for i in range(len(res)):
            if i == 0:
                start_index = 2
            else:
                start_index = res[i-1]+1
            end_index = res[i]
            mess = string_mess[start_index:end_index]
            if i == 0:
                mess = "20"+mess;
            date_time.append(int(mess))
            if i == len(res)-1:
                start_index = res[i]+1
                end_index = len(string_mess)-1
                mess = string_mess[start_index:end_index]
                date_time.append(int(mess))

        date_time.append(0)
        machine.RTC().datetime(date_time)
        print(date_time)

    # Reserve for querying and setting general settings
    # Byte format for message received:
    #   mess[0] = 0xMT
    #   mess[1] = [0xMD,0xP0,0xP1,..]
    #
    # MT is a unique address (will be 55)
    # MD is a descriptor that will designate a specific setting.
    # P0...PN are the N-number of settings values to change for a
    # specific setting.
    #
    # For querying a setting value, P0...PN will be empty
    # For setting a setting value, P0...PN will be filled
    elif mess[0] == 0x55:
        # Avoid error of looking for index that doesn't exist
        if len(mess[1:]) > 0:
            # Access the byte array of data
            data = mess[1:]
            
            # Check for each descriptor
            # This condition accounts for EMG gain settings
            if data[0] == 0x00:
                if len(data) == 1:
                    # Put in code to read settings .txt file and then send a
                    # response message containing the gain values
                    [emg_gains,td_settings,wrist_settings,settings_dict] = sh.readSettings(settings_file)
                else:
                    # Put in code to set the EMG gains in the settings file and
                    # then send a response message containing those gain values
                    for i in range(len(data)-1):
                        emg_gains[i] = data[i+1]
                    print("new EMG gains are: " + str(emg_gains))
                    settings_dict["EMG"] = emg_gains

                    # Emg gains based on default settings list
                    for i in range(len(c._emg_gains)):
                        c._emg_gains[i] = emg_gains[i]

                    # For now, leave this as not actually doing anything but
                    # just modify the display so the clinicians are happy
                    # #Update gains via CAN messages to the myoIMU
                    # for i in range(8):
                    #     c.updateGain(i+1,emg_gains[i])

                    sh.writeDict(settings_file,settings_dict)

                # Regardless of whether or not it's a query or setting message,
                # send a response with the updated values
                resp_mess = bytearray(len(mess))
                resp_mess[0] = 0x55
                message = [0] + emg_gains
                message = bytearray(message)
                resp_mess[1:] = message
                b.sendMessage(resp_mess)

                # mess_id = 0x55
                # message = [0] + emg_gains
                # message = bytearray(message)
                # mess_string = message
                # addMessageToList()

            # This condition accounts for terminal device settings
            elif data[0] == 0x01:
                if len(data) == 1:
                    # Put in code to read settings .txt file and then send a
                    # response message containing the TD settings
                    [emg_gains,td_settings,wrist_settings,settings_dict] = sh.readSettings(settings_file)
                else:
                    # Put in code to set the TD settings in the settings .txt
                    # file and then send a response message containing those
                    # values
                    for i in range(len(data)-1):
                        td_settings[i] = data[i+1]
                    print("new terminal device settings are: " + str(td_settings))
                    settings_dict["TD"] = td_settings
                    
                    # Setting terminal device settings based on updated
                    # settings list
                    if td_settings[0] < 0:
                        ml.hand_activate = 0
                    os.chdir('/sd')
                    # Determine which terminal device was used last and establish number of
                    # classes along with the class mapper with this information
                    #try:
                    f = open('td_type.txt','r')
                    td_type_string = f.read()
                    print("td_type_string: ",td_type_string)
                    f.close()
                    if int(math.fabs(td_settings[0])) in [1,3]:
                        if td_type_string == "Psyonic":
                            ml.td_type = 0
                            print("changing from psyonic to ottobock")
                            p_rec.resetSpecificClasses([5,6,7,8,9])
                            p_rec._td_type = "Ottobock"
                            calibration_counter = cal_count_ottobock
                            f = open('td_type.txt','w')
                            f.write('Ottobock')
                            f.close()
                            b._send_saved_folders = True
                    else:
                        if td_type_string == "Ottobock":
                            ml.td_type = 1
                            print("changing from ottobock to psyonic")
                            p_rec.resetSpecificClasses([5,6,7,8,9])
                            p_rec._td_type = "Psyonic"
                            calibration_counter = cal_count_psyonic
                            f = open('td_type.txt','w')
                            f.write('Psyonic')
                            f.close()
                            b._send_saved_folders = True
                    if math.fabs(td_settings[0]) in [1,2]:
                        ml.handedness = 0
                    else:
                        ml.handedness = 1
                    ml.grip_percent_change = td_settings[1]/100

                    prev_grip_classes = ml.grip.copy()

                    p_rec._class_grip_data = []

                    for i in range(len(ml.grip)):
                        if td_settings[2+i] == 255:
                            ml.grip[i] = -1
                        else:
                            ml.grip[i] = td_settings[2+i]
                        p_rec._class_grip_data.append(ml.grip[i])
                    for i in range(len(ml.grip_speed)):
                        if td_settings[7+i] == 255:
                            ml.grip_speed[i] = -1
                        else:
                            ml.grip_speed[i] = td_settings[7+i]/25
                        p_rec._class_grip_data.append(ml.grip_speed[i])

                    list_of_classes_to_reset = []

                    for i in range(len(ml.grip)):
                        if not ml.grip[i] == prev_grip_classes[i]:
                            list_of_classes_to_reset.append(6+i)

                    p_rec.resetSpecificClasses(list_of_classes_to_reset)

                    active_grips = []
                    num_grips_active = 0
                    for i in range(5):
                        if td_settings[7+i] != 255 and td_settings[7+i] != 0:
                            num_grips_active = num_grips_active + 1
                            string_name = "Grip" + str(i)
                            active_grips.append(string_name)

                    p_rec._grip_mapper["Grip0"] = ml.grip[0]
                    p_rec._grip_mapper["Grip1"] = ml.grip[1]
                    p_rec._grip_mapper["Grip2"] = ml.grip[2]
                    p_rec._grip_mapper["Grip3"] = ml.grip[3]
                    sh.writeDict(settings_file,settings_dict)
                # Regardless of whether or not it's a query or setting message,
                # send a response with the updated values
                resp_mess = bytearray(len(mess))
                resp_mess[0] = 0x55
                message = [0] + td_settings
                message = bytearray(message)
                resp_mess[1:] = message
                b.sendMessage(resp_mess)

                # mess_id = 0x55
                # message = [1] + td_settings
                # message = bytearray(message)
                # mess_string = message
                # addMessageToList()

            # This condition accounts for wrist device settings
            elif data[0] == 0x02:
                if len(data) == 1:
                    # Put in code to read settings .txt file and then send a
                    # response message containing the wrist device settings
                    [emg_gains,td_settings,wrist_settings,settings_dict] = sh.readSettings(settings_file)
                else:
                    # Put in code to set the wrist device settings in the 
                    # settings file and then send a response message 
                    # containing those values
                    for i in range(len(data)-1):
                        wrist_settings[i] = data[i+1]
                    print("new wrist settings are: " + str(wrist_settings))
                    settings_dict["WD"] = wrist_settings
                    
                    ## setting wrist settings based on updated settings list
                    ml.flex_activated = wrist_settings[0]
                    ml.flex_ramp = wrist_settings[1]
                    ml.flex_vote = wrist_settings[2]
                    ml.flex_speed_damp = wrist_settings[3]/100
                    ml.max_flex_angle = wrist_settings[4]*(ml.hard_flex_limit/100)*(1*4096)/360
                    ml.max_ext_angle = wrist_settings[5]*(ml.hard_ext_limit/100)*(1*4096)/360
                    if wrist_settings[6]>0:
                        ml.flex_pause_angle = wrist_settings[6]/100
                        ml.flex_pause_time = wrist_settings[7]/1000 #in milliseconds
                    else:
                        ml.ext_pause_angle = wrist_settings[6]/100
                        ml.ext_pause_time = wrist_settings[7]/1000 #in milliseconds
                    ml.rot_activated = wrist_settings[8]
                    ml.rot_ramp = wrist_settings[9]
                    ml.rot_vote = wrist_settings[10]
                    ml.rot_speed_damp = wrist_settings[11]/100
                    ml.max_pro_angle = wrist_settings[12]*(ml.hard_pro_limit/100)*(1*4096)/360
                    ml.max_sup_angle = wrist_settings[13]*(ml.hard_sup_limit/100)*(1*4096)/360
                    if wrist_settings[14]>0:
                        ml.pro_pause_angle = wrist_settings[14]/100
                        ml.pro_pause_time = wrist_settings[15]/1000 #in milliseconds
                    else:
                        ml.sup_pause_angle = wrist_settings[14]/100
                        ml.sup_pause_time = wrist_settings[15]/1000 #in milliseconds
                    
                    # Detecting whether the max angles for each wrist DOF exceed the endstop
                    # cushion, and if so, setting the endstop cushion to be equal to the max
                    # angle for that DOF
                    if 180-ml.max_pro_angle > ml.pro_end_cush*360/(1*4096):
                        ml.pro_end_cush = (180-ml.max_pro_angle)*(1*4096)/360
                    if 180-ml.max_sup_angle > ml.sup_end_cush*360/(1*4096):
                        ml.sup_end_cush = (180-ml.max_sup_angle)*(1*4096)/360
                    if 70-ml.max_ext_angle > ml.ext_end_cush*360/(1*4096):
                        ml.max_ext_cush = (70-ml.max_ext_angle)*(1*4096)/360
                    if 49-ml.max_flex_angle > ml.flex_end_cush*(360)/(1*4096):
                        ml.max_flex_cush = (49-ml.max_flex_angle)*(1*4096)/360

                    sh.writeDict(settings_file,settings_dict)
                # Regardless of whether or not it's a query or setting message,
                # send a response with the updated values
                resp_mess = bytearray(len(mess))
                resp_mess[0] = 0x55
                message = [0] + wrist_settings
                message = bytearray(message)
                resp_mess[1:] = message
                b.sendMessage(resp_mess)

                # The wrist settings get sent last in the order (first emg
                # gains, then terminal device settings, and finally wrist
                # settings).
                # If there was a switch in terminal device types, then the
                # "send saved folders" flag will be set to True. If that's
                # the case, don't send the "finished sending saved folders"
                # message yet. Otherwise, send it now.
                if not b._send_saved_folders and not b._first_time_connected:
                    print("should only print once (settings)")
                    mess_id = 0xce
                    mess_string = b' No more messages'
                    addMessageToList()

                # mess_id = 0x55
                # message = [2] + wrist_settings
                # message = bytearray(message)
                # mess_string = message
                # addMessageToList()

    # Reserve for starting new Troubleshooting sequence
    # Byte format for message received:
    #       mess[0] = 0x56
    #       mess[1:] = b' New sequence'
    elif mess[0] == 0x56:        
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)

        new_tshoot_sequence = True
        print("New Troubleshooting Sequence")

    # Reserve for notifying the wrist to move into its locking position
    # Byte format for message received:
    #       mess[0] = 0x57
    #       mess[3:4] = an integer corresponding to which position to move
    #                   to when locking
    elif mess[0] == 0x57:
        resp_mess = bytearray(len(mess))
        resp_mess[0] = 0x0b
        resp_mess[1:] = mess
        b.sendMessage(resp_mess)

        string_mess = str(mess)
        position = string_mess[3:4]
        desired_locked_position = 0

        if position == "0":
            desired_locked_position = ml.max_ext_pos + ml.ext_end_cush
        elif position == "1":
            desired_locked_position = ml.max_ext_pos + ml.ext_end_cush + int((40/360)*4096)
        elif position == "2":
            desired_locked_position = ml.max_flex_pos - ml.flex_end_cush - int((25/360)*4096)
        elif position == "3":
            desired_locked_position = ml.max_flex_pos - ml.flex_end_cush

        locking = True

# This function initializes/boots up the Dynamixel Motors on the first frame
#
# Inputs:
#   N/A
#
# Returns:
#   N/A
def bootDynamixel():
    global ml

    # Disable the motors to start to set initialize motor parameters
    ml.mc_dyna.disableMotor(ml.mc_dyna.rotator_motor)
    ml.rotator_disabled = True
    ml.mc_dyna.disableMotor(ml.mc_dyna.flexor_motor)
    ml.flexor_disabled = True
    ml.wrist_motors_enabled = False
    
    # Set the shutdown conditions to check for in the hardware error status
    ml.mc_dyna.writeShutdown(ml.mc_dyna.rotator_motor)
    ml.mc_dyna.writeShutdown(ml.mc_dyna.flexor_motor)

    # Set the PWM limit
    ml.mc_dyna.writePWMLimit(ml.mc_dyna.rotator_motor,ml.mc_dyna.init_pwm_limit)
    ml.mc_dyna.writePWMLimit(ml.mc_dyna.flexor_motor,ml.mc_dyna.init_pwm_limit)
    
    # Writing a baud rate of 0 will clear any errors prior to setting it to
    # a nonzero value (we chose 50 here)
    ml.mc_dyna.writeBusWatchdog(ml.mc_dyna.rotator_motor,0)
    ml.mc_dyna.writeBusWatchdog(ml.mc_dyna.rotator_motor,50)
    ml.mc_dyna.writeBusWatchdog(ml.mc_dyna.flexor_motor,0)
    ml.mc_dyna.writeBusWatchdog(ml.mc_dyna.flexor_motor,50)
    
    # Set indirect addresses
    ml.mc_dyna.initIndirect(ml.mc_dyna.flexor_motor,ml.mc_dyna.init_ind1)
    ml.mc_dyna.initIndirect(ml.mc_dyna.flexor_motor,ml.mc_dyna.init_ind2)
    ml.mc_dyna.initIndirect(ml.mc_dyna.flexor_motor,ml.mc_dyna.init_ind3)
    ml.mc_dyna.initIndirect(ml.mc_dyna.flexor_motor,ml.mc_dyna.init_ind4)
    ml.mc_dyna.initIndirect(ml.mc_dyna.flexor_motor,ml.mc_dyna.init_ind5)
    ml.mc_dyna.initIndirect(ml.mc_dyna.flexor_motor,ml.mc_dyna.init_ind6)
    ml.mc_dyna.initIndirect(ml.mc_dyna.flexor_motor,ml.mc_dyna.init_ind7)
    ml.mc_dyna.initIndirect(ml.mc_dyna.flexor_motor,ml.mc_dyna.init_ind8)
    ml.mc_dyna.initIndirect(ml.mc_dyna.rotator_motor,ml.mc_dyna.init_ind1)
    ml.mc_dyna.initIndirect(ml.mc_dyna.rotator_motor,ml.mc_dyna.init_ind2)
    ml.mc_dyna.initIndirect(ml.mc_dyna.rotator_motor,ml.mc_dyna.init_ind3)
    ml.mc_dyna.initIndirect(ml.mc_dyna.rotator_motor,ml.mc_dyna.init_ind4)
    ml.mc_dyna.initIndirect(ml.mc_dyna.rotator_motor,ml.mc_dyna.init_ind5)
    ml.mc_dyna.initIndirect(ml.mc_dyna.rotator_motor,ml.mc_dyna.init_ind6)
    ml.mc_dyna.initIndirect(ml.mc_dyna.rotator_motor,ml.mc_dyna.init_ind7)
    ml.mc_dyna.initIndirect(ml.mc_dyna.rotator_motor,ml.mc_dyna.init_ind8)
    
    # Start with motors enabled
    ml.mc_dyna.enableMotor(ml.mc_dyna.rotator_motor)
    ml.rotator_disabled = False
    ml.mc_dyna.enableMotor(ml.mc_dyna.flexor_motor)
    ml.flexor_disabled = False
    ml.wrist_motors_enabled = True

# This function will handle enabling, disabling, and rebooting Dyanmixel motors
# MIGHT NEED TO GO BACK AND ADD BACK IN THE UASYNCIO CALLS
#
# Inputs:
#   just_enabled:               flag notifying whether or not the motors have
#                               just enabled torque mode
#   reboot_id:                  the id of the motor that was just rebooted
#   just_rebooted:              flag notifying whether or not the motor of
#                               interest has just rebooted or not
#
# Returns:
#   just_enabled:               flag notifying whether or not the motors have
#                               just enabled torque mode
#   reboot_id:                  the id of the motor that was just rebooted
#   just_rebooted:              flag notifying whether or not the motor of
#                               interest has just rebooted or not
def enableDisableRebootDynamixel(just_enabled,reboot_id,just_rebooted):
    global ml, need_to_enable, need_to_disable, reboot_rotator,reboot_flexor

    # A flag is set when the "ENABLE MOTORS" button is pressed
    # that alerts this section of the code to physically enable
    # the motors
    if need_to_enable and not ml.wrist_motors_enabled:
        ml.mc_dyna.enableMotor(ml.mc_dyna.rotator_motor)
        ml.rotator_disabled = False
        #await uasyncio.sleep_ms(0)
        ml.mc_dyna.enableMotor(ml.mc_dyna.flexor_motor)
        ml.flexor_disabled = False
        #await uasyncio.sleep_ms(0)

        # Mark that they are enabled and that they don't need to be
        # enabled until another message comes through
        ml.wrist_motors_enabled = True
        need_to_enable = False
        just_enabled = True
    # A flag is set when the "DISABLE MOTORS" button is pressed
    # that alerts this section of the code to physically disable
    # the motors
    elif need_to_disable and ml.wrist_motors_enabled:
        # Update position/profile velocity values to ensure last
        # command does not send the motors in either direction
        ml.rot_pos_des = ml.rot_pos
        ml.flex_pos_des = ml.flex_pos
        ml.rot_vel = 0
        ml.flex_vel = 0
        ml.hand_vel = 0
        ml.writeHand(write_psyonic_flag)
        #await uasyncio.sleep_ms(0)
        ml.mc_dyna.disableMotor(ml.mc_dyna.rotator_motor)
        ml.rotator_disabled = True
        #await uasyncio.sleep_ms(0)
        ml.mc_dyna.disableMotor(ml.mc_dyna.flexor_motor)
        ml.flexor_disabled = True
        #await uasyncio.sleep_ms(0)
        # Mark that they are disabled and that they don't need to
        # be disabled until another message comes through
        ml.wrist_motors_enabled = False
        need_to_disable = False
    
    # If the flag for rebooting either motor is enabled (which will
    # occur on a hardware error status flag, reboot the motor, note
    # which motor was rebooted, disable the flag that says it needs
    # to be rebooted, and enable the flag saying that it was just
    # rebooted
    if reboot_rotator:
        ml.mc_dyna.rebootMotor(ml.mc_dyna.rotator_motor)
        reboot_id = ml.mc_dyna.rotator_motor
        reboot_rotator = False
        just_rebooted = True
    if reboot_flexor:
        ml.mc_dyna.rebootMotor(ml.mc_dyna.flexor_motor)
        reboot_id = ml.mc_dyna.flexor_motor
        reboot_flexor = False
        just_rebooted = True

    return [just_enabled,reboot_id,just_rebooted]

# This function disables the recently rebooted motor, reinitializes the
# indirect data addresses, and then re-enables the rebooted motor
#
# Inputs:
#   just_rebooted:              flag notifying whether or not the motor of
#                               interest has just rebooted or not
#   reboot_id:                  the id of the motor that was just rebooted
#
# Returns:
#   just_rebooted:              flag notifying whether or not the motor of
#                               interest has just rebooted or not
#   reboot_id:                  the id of the motor that was just rebooted
def actOnRebootedDyanmixel(just_rebooted,reboot_id):
    global ml

    ml.mc_dyna.disableMotor(reboot_id)
    if reboot_id == ml.mc_dyna.rotator_motor:
        ml.rotator_disabled = True
    elif reboot_id == ml.mc_dyna.flexor_motor:
        ml.flexor_disabled = True
    ml.mc_dyna.initIndirect(reboot_id,ml.mc_dyna.init_ind1)
    ml.mc_dyna.initIndirect(reboot_id,ml.mc_dyna.init_ind2)
    ml.mc_dyna.initIndirect(reboot_id,ml.mc_dyna.init_ind3)
    ml.mc_dyna.initIndirect(reboot_id,ml.mc_dyna.init_ind4)
    ml.mc_dyna.initIndirect(reboot_id,ml.mc_dyna.init_ind5)
    ml.mc_dyna.initIndirect(reboot_id,ml.mc_dyna.init_ind6)
    ml.mc_dyna.enableMotor(reboot_id)
    if reboot_id == ml.mc_dyna.rotator_motor:
        ml.rotator_disabled = False
    elif reboot_id == ml.mc_dyna.flexor_motor:
        ml.flexor_disabled = False
    just_rebooted = False

    return [just_rebooted,reboot_id]

# This function reads in Dynamixel sensor data and handles it
#
# Inputs:
#   just_enabled:               flag notifying whether or not the motors have
#                               just enabled torque mode
#   frame_counter:              the count of the frame that the loop is on
#
# Returns:
#   just_enabled:               flag notifying whether or not the motors have
#                               just enabled torque mode
#   frame_counter:              the count of the frame that the loop is on
def readDynamixel(just_enabled,frame_counter):
    global ml

    # Call the syncReadInd() function to update the
    # present values of position, current, bus 
    # watchdog, and hardware error status for each
    # motor
    [ml.rot_pos,ml.flex_pos,ml.rot_curr,ml.flex_curr,ml.rot_watchdog,ml.flex_watchdog,ml.rot_error_status,ml.flex_error_status] = ml.mc_dyna.syncReadInd()
    
    # Depending on where the wrist motors are located
    # with respect to their neutral positions (0
    # degrees in either direction), update the sign of
    # their positional angles.
    # We use this information for informing the PAT App
    # where the wrist motors are located in order for
    # the VR arm to be as accurate as possible of the
    # physical device's true position
    if (ml.rot_angle >= 0):
        ml.rot_pos_sign = 0
    else:
        ml.rot_pos_sign = 1
    if (ml.flex_angle >= 0):
        ml.flex_pos_sign = 0
    else:
        ml.flex_pos_sign = 1                                

    # On the first frame, we want to figure out where
    # the Dynamixel motors think they are located.
    # We need to do this because based on where they
    # were located before they were power cycled, they
    # may be off by one rotation in either direction.
    # Reading in the position values and then comparing
    # them with the hardcoded endstops will allow us to
    # know how we need to adjust the endstop values.
    if frame_counter == 0:
        # Determine which end stop condition the motors
        # are in, based on what position the motors
        # powered back on in
        if ml.flex_pos <= ml.const_max_flex_pos[0]:
            ml.max_ext_pos = ml.const_max_ext_pos[0]
            ml.max_flex_pos = ml.const_max_flex_pos[0]
            ml.mid_flex_pos = ml.const_mid_flex_pos[0]
        else:
            ml.max_ext_pos = ml.const_max_ext_pos[1]
            ml.max_flex_pos = ml.const_max_flex_pos[1]
            ml.mid_flex_pos = ml.const_mid_flex_pos[1]
        if ml.rot_pos <= ml.const_max_cw_pos[0]:
            ml.max_ccw_pos = ml.const_max_ccw_pos[0]
            ml.max_cw_pos = ml.const_max_cw_pos[0]
            ml.mid_rot_pos = ml.const_mid_rot_pos[0]
        else:
            ml.max_ccw_pos = ml.const_max_ccw_pos[1]
            ml.max_cw_pos = ml.const_max_cw_pos[1]
            ml.mid_rot_pos = ml.const_mid_rot_pos[1]

        # The rotator and flexor will occasionally
        # drift upon boot up.
        # These variables help with avoiding this
        # problem.
        ml.last_zero_vel_rot_pos = ml.rot_pos
        ml.last_zero_vel_flex_pos = ml.flex_pos
        ml.find_new_zero_vel_rot_pos = True
        ml.find_new_zero_vel_flex_pos = True

    # Occasionally, positions are coming back extremely
    # off, but they don't get flagged because the
    # uart.read() and crc checks are accurate. We
    # ignore the first 2 frames since the positions
    # start at 0 and then get initialized
    if frame_counter > 2:
        # If the positions on back to back readings are
        # different by more than 200 encoder counts, it
        # is likely that one of the readings came back
        # incorrectly. If so, just set the present
        # value to be the previous one.
        if math.fabs(ml.rot_pos-ml.prev_rot_pos) > 100 and (ml.rot_pos == 0):
            ml.rot_pos = ml.prev_rot_pos
        if math.fabs(ml.flex_pos-ml.prev_flex_pos) > 100 and (ml.flex_pos == 0):
            ml.flex_pos = ml.prev_flex_pos

    # The rotator and flexor will occasionally
    # drift upon torque enable.
    # These variables help with avoiding this
    # problem.
    if just_enabled:
        just_enabled = False
        ml.last_zero_vel_rot_pos = ml.rot_pos
        ml.last_zero_vel_flex_pos = ml.flex_pos
        ml.find_new_zero_vel_rot_pos = True
        ml.find_new_zero_vel_flex_pos = True

    # Update the frame counter and set the previous
    # position and current readings to be the present
    # ones
    frame_counter = frame_counter + 1
    ml.prev_rot_pos = ml.rot_pos
    ml.prev_flex_pos = ml.flex_pos
    ml.prev_rot_curr = ml.rot_curr
    ml.prev_flex_curr = ml.flex_curr

    return [just_enabled,frame_counter]

# Handle what happens when the number of bad Dynamixel readings exceeds a
# certain number of frames
#
# Inputs:
#   just_rebooted:              flag notifying whether or not the motor of
#                               interest has just rebooted or not
#   reboot_id:                  the id of the motor that was just rebooted
#
# Returns:
#   just_rebooted:              flag notifying whether or not the motor of
#                               interest has just rebooted or not
#   reboot_id:                  the id of the motor that was just rebooted
def handleBadReadingsDyanmixel(just_rebooted,reboot_id):
    global b, ml, write_psyonic_flag, reboot_rotator,reboot_flexor

    # If the number of bad readings in a row ever exceeds
    # 10, then disable the motors, write zero velocity to
    # the hand, and raise an error
    if ml.bad_readings > 10:
        print("Motors disabling due to reading errors")
        try:
            ml.mc_dyna.disableMotor(ml.mc_dyna.rotator_motor)
            ml.rotator_disabled = True
            ml.mc_dyna.disableMotor(ml.mc_dyna.flexor_motor)
            ml.flexor_disabled = True
            ml.wrist_motors_enabled = False
            b.disconnectBLE()
        except:
            b.disconnectBLE()
        ml.hand_vel = 0
        ml.writeHand(write_psyonic_flag)
        a = 1/0

    # If the bus watchdog ever returns 255 (-1), that means
    # that their was a UART error somewhere, so we need to
    # increment the number of bad watchdog readings in a
    # row, the same way we did with bad uart readings,
    # clear the bus watchdog by setting it to 0, and then
    # reset it to it's original value that we initialized
    # it to
    if ml.rot_watchdog == 255 or ml.flex_watchdog == 255:
        ml.bad_watchdogs = ml.bad_watchdogs + 1
        if ml.rot_watchdog == 255:
            ml.mc_dyna.writeBusWatchdog(ml.mc_dyna.rotator_motor,0)
            ml.mc_dyna.writeBusWatchdog(ml.mc_dyna.rotator_motor,50)
        if ml.flex_watchdog == 255:
            ml.mc_dyna.writeBusWatchdog(ml.mc_dyna.flexor_motor,0)
            ml.mc_dyna.writeBusWatchdog(ml.mc_dyna.flexor_motor,50)
    else:
        ml.bad_watchdogs = 0
    
    # In a similar fashion, if the number of bad watchdog
    # readings in a row exceeds 10, then disable the
    # motors, write zero velocity to the hand, and raise an
    # error
    if ml.bad_watchdogs > 10:
        print("Motors disabling due to watchdog errors")
        try:
            ml.mc_dyna.disableMotor(ml.mc_dyna.rotator_motor)
            ml.rotator_disabled = True
            ml.mc_dyna.disableMotor(ml.mc_dyna.flexor_motor)
            ml.flexor_disabled = True
            ml.wrist_motors_enabled = False
            b.disconnectBLE()
        except:
            b.disconnectBLE()
        pyb.delay(5)
        ml.hand_vel = 0
        ml.writeHand(write_psyonic_flag)
        a = 1/0

    # If the hardware error status for either motors is
    # ever nonzero, that means that there is some form of
    # hardware error occurring. When this occurs, reboot
    # the motor that has failed, and update the reboot
    # status like we did before
    if ml.rot_error_status != 0 or ml.flex_error_status != 0:
        ml.bad_error_statuses = ml.bad_error_statuses + 1
        if ml.rot_error_status != 0:
            ml.mc_dyna.rebootMotor(ml.mc_dyna.rotator_motor)
            reboot_id = ml.mc_dyna.rotator_motor
            reboot_rotator = False
            just_rebooted = True
        elif ml.flex_error_status != 0:
            ml.mc_dyna.rebootMotor(ml.mc_dyna.flexor_motor)
            reboot_id = ml.mc_dyna.flexor_motor
            reboot_flexor = False
            just_rebooted = True
    
    # If there are too many hardware errors in a row,
    # follow the same protocol of disabling motors, writing
    # zero velocity to the hand, and raising an error
    if ml.bad_error_statuses > 10:
        print("Motors disabling due to hardware errors")
        try:
            ml.mc_dyna.disableMotor(ml.mc_dyna.rotator_motor)
            ml.rotator_disabled = True
            ml.mc_dyna.disableMotor(ml.mc_dyna.flexor_motor)
            ml.flexor_disabled = True
            ml.wrist_motors_enabled = False
            b.disconnectBLE()
        except:
            b.disconnectBLE()
        pyb.delay(5)
        ml.hand_vel = 0
        ml.writeHand(write_psyonic_flag)
        a = 1/0

    return [reboot_id,just_rebooted]

# Keep a log of the previous "x" number of frames of position/current data of
# the Dynamixel motors as well as the past "y" number of frames of grip status
# information from the Psyonic hand (should it be attached)
#
# Inputs:
#   N/A
#
# Returns:
#   N/A
def trackMotorHistory():
    global ml

    # Keep track of the past "x" position and current values for each motor
    # with the present value going in the "x"th index
    for i in range(ml.wrist_history_length):
        if i != ml.wrist_history_length-1:
            ml.rot_pos_history[i] = ml.rot_pos_history[i+1]
            ml.flex_pos_history[i] = ml.flex_pos_history[i+1]
            ml.rot_curr_history[i] = ml.rot_curr_history[i+1]
            ml.flex_curr_history[i] = ml.flex_curr_history[i+1]
        else:
            ml.rot_pos_history[i] = ml.rot_pos
            ml.flex_pos_history[i] = ml.flex_pos
            ml.rot_curr_history[i] = ml.rot_curr
            ml.flex_curr_history[i] = ml.flex_curr

    # If the Psyonic hand is attached, keep track of the past "y" grip types
    # with the present value going in the "y"th index
    if ml.td_type == 1:
        hand_resp = ml.mc_psy.readPsyonic()
        ml.hand_grip_status = hand_resp[7]
        for i in range(ml.hand_history_length):
            if i != ml.hand_history_length-1:
                ml.hand_grip_stat_history[i] = ml.hand_grip_stat_history[i+1]
            else:
                ml.hand_grip_stat_history[i] = ml.hand_grip_status

# No matter what state you are in (training, homing, normal operation, or
# manual motors), always check to see if you have hit an endstop
#
# Inputs:
#   cw_end_found:               flag for if the cw endstop has been found
#   ccw_end_found:              flag for if the ccw endstop has been found
#   ext_end_found:              flag for if the ext endstop has been found
#   flex_end_found:             flag for if the flex endstop has been found
#
# Returns:
#   cw_end_found:               flag for if the cw endstop has been found
#   ccw_end_found:              flag for if the ccw endstop has been found
#   ext_end_found:              flag for if the ext endstop has been found
#   flex_end_found:             flag for if the flex endstop has been found
def checkEndstopsDynamixel(cw_end_found,ccw_end_found,ext_end_found,flex_end_found):
    global sh, ml, true_rot_end_found, true_flex_end_found, mess_id, mess_id_list, endstops_file, endstops_dict
    # If you are writing a positive velocity and your
    # position values are not changing very much and
    # your current values are changing a lot and your
    # present current value exceeds a certain threshold, it
    # is likely that you have hit a physical endstop.
    #ROTATOR
    if (math.fabs(ml.rot_vel) > 0 and math.fabs(ml.rot_pos_history[4]-ml.rot_pos_history[0]) < 50 and math.fabs(ml.rot_curr) > ml.max_rot_curr_ends and math.fabs(ml.rot_curr_history[4]-ml.rot_curr_history[0]) > 100):
        print("rot_curr: ",ml.rot_curr)
        print("max rot curr: ",ml.max_rot_curr_ends)
        # Check the CW direction and then back calculate
        # the CCW endstop from this measurement
        if ml.rot_vel > 0:
            cw_end_found = True
            ml.max_cw_pos = ml.rot_pos
            print("max_cw_pos: " + str(ml.max_cw_pos))
            ml.max_ccw_pos = ml.max_cw_pos - 3900
            print("max_ccw_pos: " + str(ml.max_ccw_pos))
        # Check the CCW direction and then back calculate
        # the CW endstop from this measurement
        elif ml.rot_vel < 0:
            ccw_end_found = True
            ml.max_ccw_pos = ml.rot_pos
            print("max_ccw_pos: " + str(ml.max_ccw_pos))
            ml.max_cw_pos = ml.max_ccw_pos + 3900
            print("max_cw_pos: " + str(ml.max_cw_pos))
        ml.mid_rot_pos = (ml.max_cw_pos+ml.max_ccw_pos)/2
        ml.rot_vel = 0
        true_rot_end_found = True
        mess_id = 0xae
        mess_string = b' Rot True'
        addMessageToList()

        # Reset the hard coded endstop positions and write
        # them to the SD card
        if ml.max_ccw_pos < 0:
            ml.const_max_ccw_pos = [ml.max_ccw_pos,ml.max_ccw_pos+4096]
            ml.const_max_cw_pos = [ml.max_cw_pos,ml.max_cw_pos+4096]
            ml.const_mid_rot_pos = [ml.mid_rot_pos,ml.mid_rot_pos+4096]
        else:
            ml.const_max_ccw_pos = [ml.max_ccw_pos-4096,ml.max_ccw_pos]
            ml.const_max_cw_pos = [ml.max_cw_pos-4096,ml.max_cw_pos]
            ml.const_mid_rot_pos = [ml.mid_rot_pos-4096,ml.mid_rot_pos]

        endstops_dict = {"max_ccw_pos": ml.const_max_ccw_pos, "max_cw_pos": ml.const_max_cw_pos, "mid_rot_pos": ml.const_mid_rot_pos, "max_ext_pos": ml.const_max_ext_pos, "max_flex_pos": ml.const_max_flex_pos, "mid_flex_pos": ml.const_mid_flex_pos}

        cwd = os.getcwd()
        os.chdir('/sd')
        sh.writeDict(endstops_file,endstops_dict)
        os.chdir(cwd)

    #FLEXOR
    if (math.fabs(ml.flex_vel) > 0 and math.fabs(ml.flex_pos_history[4]-ml.flex_pos_history[0]) < 40 and math.fabs(ml.flex_curr) > ml.max_flex_curr_ends and math.fabs(ml.flex_curr_history[4]-ml.flex_curr_history[0]) > 20):
        print("flex_curr: ",ml.flex_curr)
        print("max flex curr: ",ml.max_flex_curr_ends)
        # Check the flexion direction and then back
        # calculate the extension endstop from this
        # measurement
        if ml.flex_vel > 0:
            flex_end_found = True
            ml.max_flex_pos = ml.flex_pos
            print("max_flex_pos: " + str(ml.max_flex_pos))
            ml.max_ext_pos = ml.max_flex_pos - 1215
            print("max_ext_pos: " + str(ml.max_ext_pos))
        # Check the extension direction and then back
        # calculate the flexion endstop from this
        # measurement
        elif ml.flex_vel < 0:
            ext_end_found = True
            ml.max_ext_pos = ml.flex_pos
            print("max_ext_pos: " + str(ml.max_ext_pos))
            ml.max_flex_pos = ml.max_ext_pos + 1215
            print("max_flex_pos: " + str(ml.max_flex_pos))
        ml.mid_flex_pos = (ml.max_ext_pos+ml.max_flex_pos)/2
        ml.flex_vel = 0
        true_flex_end_found = True
        mess_id = 0xae
        mess_string = b' Flex True'
        addMessageToList()

        # Reset the hard coded endstop positions and write
        # them to the SD card
        if ml.max_ext_pos < 0:
            ml.const_max_ext_pos = [ml.max_ext_pos,ml.max_ext_pos+4096]
            ml.const_max_flex_pos = [ml.max_flex_pos,ml.max_flex_pos+4096]
            ml.const_mid_flex_pos = [ml.mid_flex_pos,ml.mid_flex_pos+4096]
        else:
            ml.const_max_ext_pos = [ml.max_ext_pos-4096,ml.max_ext_pos]
            ml.const_max_flex_pos = [ml.max_flex_pos-4096,ml.max_flex_pos]
            ml.const_mid_flex_pos = [ml.mid_flex_pos-4096,ml.mid_flex_pos]

        endstops_dict = {"max_ccw_pos": ml.const_max_ccw_pos, "max_cw_pos": ml.const_max_cw_pos, "mid_rot_pos": ml.const_mid_rot_pos, "max_ext_pos": ml.const_max_ext_pos, "max_flex_pos": ml.const_max_flex_pos, "mid_flex_pos": ml.const_mid_flex_pos}

        cwd = os.getcwd()
        os.chdir('/sd')
        sh.writeDict(endstops_file,endstops_dict)
        os.chdir(cwd)

    return [cw_end_found,ccw_end_found,ext_end_found,flex_end_found]

# If you are in the process of training and have not paused the training,
# execute the following code
#
# Inputs:
#   N/A
#
# Returns:
#   N/A
def isTrainingAndNotPaused():
    global c, ml, p_rec, dbt_case, dbt_training_start_time, num_grips_active, active_grips, collecting_data, num_mess
    global class_training,train_hand_condition,train_rotator_condition,train_flexor_condition,prev_grip_trained
    
    # If you are in the process of device based training (DBT) and not training
    # from the PAT App you will follow this logic, which looks to see which
    # step of the DBT process you are on and call the deviceBasedTraining
    # function and handle the logic based on that step
    if c._device_based_training:
        if dbt_case == 0:
            deviceBasedTraining("No Movement:Prep")
            dbt_training_start_time = utime.ticks_ms()
            dbt_case = dbt_case + 1
        elif dbt_case in [2,4,6,8,10,12,14,16,18]:
            if utime.ticks_ms()-dbt_training_start_time > 3000:
                if dbt_case == 2:
                    deviceBasedTraining("No Movement:Prep")
                elif dbt_case in [4,8]:
                    deviceBasedTraining("Flexion:Prep")
                elif dbt_case in [6,10]:
                    deviceBasedTraining("Extension:Prep")
                elif dbt_case in [12,16]:
                    deviceBasedTraining("Pronation:Prep")
                elif dbt_case in [14,18]:
                    deviceBasedTraining("Supination:Prep")
                dbt_training_start_time = utime.ticks_ms()
                dbt_case = dbt_case + 1
        elif dbt_case in [1,3]:
            if utime.ticks_ms()-dbt_training_start_time > ml.calibration_time*1000:
                deviceBasedTraining("No Movement")
                dbt_training_start_time = utime.ticks_ms()
                dbt_case = dbt_case + 1
        elif dbt_case in [5,9]:
            if utime.ticks_ms()-dbt_training_start_time > ml.calibration_time*1000:
                deviceBasedTraining("Flexion")
                dbt_training_start_time = utime.ticks_ms()
                dbt_case = dbt_case + 1
        elif dbt_case in [7,11]:
            if utime.ticks_ms()-dbt_training_start_time > ml.calibration_time*1000:
                deviceBasedTraining("Extension")
                dbt_training_start_time = utime.ticks_ms()
                dbt_case = dbt_case + 1
        elif dbt_case in [13,17]:
            if utime.ticks_ms()-dbt_training_start_time > ml.calibration_time*1000:
                deviceBasedTraining("Pronation")
                dbt_training_start_time = utime.ticks_ms()
                dbt_case = dbt_case + 1
        elif dbt_case in [15,19]:
            if utime.ticks_ms()-dbt_training_start_time > ml.calibration_time*1000:
                deviceBasedTraining("Supination")
                dbt_training_start_time = utime.ticks_ms()
                dbt_case = dbt_case + 1
        if p_rec._td_type == "Ottobock":
            if dbt_case in [20,22,24,26]:
                if utime.ticks_ms()-dbt_training_start_time > 3000:
                    if dbt_case in [20,24]:
                        deviceBasedTraining("Open:Prep")
                    elif dbt_case in [22,26]:
                        deviceBasedTraining("Close:Prep")
                    dbt_training_start_time = utime.ticks_ms()
                    dbt_case = dbt_case + 1
            elif dbt_case in [21,25]:
                if utime.ticks_ms()-dbt_training_start_time > ml.calibration_time*1000:
                    deviceBasedTraining("Open")
                    dbt_training_start_time = utime.ticks_ms()
                    dbt_case = dbt_case + 1
            elif dbt_case in [23,27]:
                if utime.ticks_ms()-dbt_training_start_time > ml.calibration_time*1000:
                    deviceBasedTraining("Close")
                    dbt_training_start_time = utime.ticks_ms()
                    dbt_case = dbt_case + 1
            elif dbt_case == 28:
                if utime.ticks_ms()-dbt_training_start_time > 3000:
                    deviceBasedTraining("Done!!!")
                    dbt_case = 0
                    dbt_training_start_time = 0
        elif p_rec._td_type == "Psyonic":
            if dbt_case == (20+num_grips_active*8):
                deviceBasedTraining("Done!!!")
                dbt_case = 0
                dbt_training_start_time = 0
            elif dbt_case in [22,26]:
                mess = active_grips[0]+":Prep"
                deviceBasedTraining(mess)
            elif dbt_case in [23,27]:
                if utime.ticks_ms()-dbt_training_start_time > ml.calibration_time*1000:
                    deviceBasedTraining(active_grips[0])
                    dbt_training_start_time = utime.ticks_ms()
                    dbt_case = dbt_case + 1
            elif dbt_case in [30,34]:
                mess = active_grips[1]+":Prep"
                deviceBasedTraining(mess)
            elif dbt_case in [31,35]:
                if utime.ticks_ms()-dbt_training_start_time > ml.calibration_time*1000:
                    deviceBasedTraining(active_grips[1])
                    dbt_training_start_time = utime.ticks_ms()
                    dbt_case = dbt_case + 1
            elif dbt_case in [38,42]:
                mess = active_grips[2]+":Prep"
                deviceBasedTraining(mess)
            elif dbt_case in [39,43]:
                if utime.ticks_ms()-dbt_training_start_time > ml.calibration_time*1000:
                    deviceBasedTraining(active_grips[2])
                    dbt_training_start_time = utime.ticks_ms()
                    dbt_case = dbt_case + 1
            elif dbt_case in [46,50]:
                mess = active_grips[3]+":Prep"
                deviceBasedTraining(mess)
            elif dbt_case in [47,51]:
                if utime.ticks_ms()-dbt_training_start_time > ml.calibration_time*1000:
                    deviceBasedTraining(active_grips[3])
                    dbt_training_start_time = utime.ticks_ms()
                    dbt_case = dbt_case + 1
            elif dbt_case in [54,58]:
                mess = active_grips[4]+":Prep"
                deviceBasedTraining(mess)
            elif dbt_case in [55,59]:
                if utime.ticks_ms()-dbt_training_start_time > ml.calibration_time*1000:
                    deviceBasedTraining(active_grips[4])
                    dbt_training_start_time = utime.ticks_ms()
                    dbt_case = dbt_case + 1
            if num_grips_active == 1:
                if utime.ticks_ms()-dbt_training_start_time > 3000:
                    if dbt_case in [20,24]:
                        deviceBasedTraining("Open:Prep")
                    elif dbt_case in [21,25]:
                        deviceBasedTraining("Open")
                    dbt_training_start_time = utime.ticks_ms()
                    dbt_case = dbt_case + 1
            elif num_grips_active == 2:
                if utime.ticks_ms()-dbt_training_start_time > 3000:
                    if dbt_case in [20,24,28,32]:
                        deviceBasedTraining("Open:Prep")
                    elif dbt_case in [21,25,29,33]:
                        deviceBasedTraining("Open")
                    dbt_training_start_time = utime.ticks_ms()
                    dbt_case = dbt_case + 1
            elif num_grips_active == 3:
                if utime.ticks_ms()-dbt_training_start_time > 3000:
                    if dbt_case in [20,24,28,32,36,40]:
                        deviceBasedTraining("Open:Prep")
                    elif dbt_case in [21,25,29,33,37,41]:
                        deviceBasedTraining("Open")
                    dbt_training_start_time = utime.ticks_ms()
                    dbt_case = dbt_case + 1 
            elif num_grips_active == 4:
                if utime.ticks_ms()-dbt_training_start_time > 3000:
                    if dbt_case in [20,24,28,32,36,40,44,48]:
                        deviceBasedTraining("Open:Prep")
                    elif dbt_case in [21,25,29,33,37,41,45,49]:
                        deviceBasedTraining("Open")
                    dbt_training_start_time = utime.ticks_ms()
                    dbt_case = dbt_case + 1
            elif num_grips_active == 5:
                if utime.ticks_ms()-dbt_training_start_time > 3000:
                    if dbt_case in [20,24,28,32,36,40,44,48,52,56]:
                        deviceBasedTraining("Open:Prep")
                    elif dbt_case in [21,25,29,33,37,41,45,49,53,57]:
                        deviceBasedTraining("Open")
                    dbt_training_start_time = utime.ticks_ms()
                    dbt_case = dbt_case + 1

    # Reset the motor velocities to zero prior to setting the dfiferent
    # training velocities (should you be collecting data)
    ml.rot_vel = 0
    ml.flex_vel = 0
    ml.hand_vel = 0

    # Check to see if you are in the process of collecting data
    if collecting_data:
        # If so, send motor commands based on which motion you are training
        if num_mess < 200:
            ml.setTrainingVelocities(class_training,train_hand_condition,train_rotator_condition,train_flexor_condition,prev_grip_trained)
        # If the class that is training is "Done!!!", this means that the
        # motion is done collecting data so we reset collecting_data to be
        # false
        if class_training == "Done!!!":
            collecting_data = False

# If the wrist is either moving to its neutral position or is in the process of
# having a diagnostic check run
#
# Inputs:
#   rot_neutralized:            flag for if the rotator has found neutral
#   flex_neutralized:           flag for if the flexor has found neutral
#
# Returns:
#   rot_neutralized:            flag for if the rotator has found neutral
#   flex_neutralized:           flag for if the flexor has found neutral
def movingNeutralRunningDiagnostic(rot_neutralized,flex_neutralized):
    global b, c, ml, hand_homed, true_rot_end_found, true_flex_end_found, send_classifier_decision
    global moving_to_neutral, running_diagnostic, mess_id, mess_string

    # Start by initalizing all motors to have zero
    # velocity (this will be changed depending on which
    # condition you are in)
    ml.rot_vel = 0
    ml.flex_vel = 0
    ml.hand_vel = 0

    # If the hand has not yet been homed, and it
    # has been less than 2.5 seconds since the 
    # homing button was pressed, close the hand
    if not hand_homed:
        if ml.td_type == 0:
            if utime.ticks_ms() - hand_timer < 2500:
                ml.hand_vel = -50
            # Once this timer elapses, switch the hand
            # homed flag to true and set the motor
            # velocities to be zero
            else:
                hand_homed = True
                ml.hand_vel = 0
                ml.rot_vel = 0
                ml.flex_vel = 0
        elif ml.td_type == 1:
            if ml.hand_grip_status == 0x0c:
                hand_homed = True
                ml.hand_vel = 0
                ml.rot_vel = 0
                ml.flex_vel = 0
    else:
        # If you haven't found the rotator end stop,
        # drive the rotator at the end stops velocity
        if not true_rot_end_found:
            print("found new rot end stop")
            ml.rot_vel = ml.rot_vel_ends
        # Otherwise, move to the middle position at a
        # constant velocity
        else:
            if not rot_neutralized:
                if ml.rot_pos < ((ml.max_cw_pos + ml.max_ccw_pos)/2):
                    ml.rot_vel = ml.rot_vel_ends
                elif ml.rot_pos > ((ml.max_cw_pos + ml.max_ccw_pos)/2):
                    ml.rot_vel = -1*ml.rot_vel_ends
                if math.fabs(ml.rot_pos - ((ml.max_cw_pos + ml.max_ccw_pos)/2)) < 50:
                    print("neutralized rotator")
                    rot_neutralized = True
            else:
                # Same process goes for the flexor
                if not true_flex_end_found:
                    ml.flex_vel = -1*ml.flex_vel_ends
                else:
                    if not flex_neutralized:
                        if ml.flex_pos < (ml.max_flex_pos + ml.max_ext_pos)*(1/3):
                            ml.flex_vel = ml.flex_vel_ends
                        elif ml.flex_pos > (ml.max_flex_pos + ml.max_ext_pos)*(1/3):
                            ml.flex_vel = -1*ml.flex_vel_ends
                        if math.fabs(ml.flex_pos - (ml.max_flex_pos + ml.max_ext_pos)*(1/3)) < 50:
                            flex_neutralized = True
                    # Once you have finished getting to
                    # the neutral location, send a
                    # message back to the PAT App
                    # stating that you've either found
                    # neutral or finished diagnostics
                    else:
                        print("neutralized flexor")
                        if c._device_based_training:
                            c.sendTrainSuccess()
                            c._device_based_training = False
                            c.sendStart()
                            b._send_saved_folders = True
                            print("just made the classifier")
                            send_classifier_decision = True
                        else:
                            if moving_to_neutral:
                                mess_id = 0xad
                                mess_string = b' Done Neutral'
                                addMessageToList()
                            elif running_diagnostic:
                                mess_id = 0xba
                                mess_string = b' Done Diagnosing'
                                addMessageToList()
                        moving_to_neutral = False
                        running_diagnostic = False
                        hand_homed = False
                        rot_neutralized = False
                        flex_neutralized = False

    return [rot_neutralized,flex_neutralized]

# Handle logic for what to do if you are in the process of locking the flexor
#
# Inputs:
#   N/A
#
# Returns:
#   N/A
def lockingFlexor():
    global ml, desired_locked_position, locking, mess_id, mess_string

    if math.fabs(ml.flex_pos-desired_locked_position) > 10:
        if ml.flex_pos < desired_locked_position:
            ml.flex_vel = ml.flex_vel_ends
        else:
            ml.flex_vel = -1*ml.flex_vel_ends
    else:
        locking = False
        mess_id = 0xca
        mess_string = b' Done Locking'
        addMessageToList()

# Handle logic for homing the wrist prior to training
#
# Inputs:
#   psyonic_homing_counter:     a counter for how many frames the psyonic hand
#                               has been homing for
#   cw_end_found:               flag for if the cw endstop has been found
#   ccw_end_found:              flag for if the ccw endstop has been found
#   ext_end_found:              flag for if the ext endstop has been found
#   flex_end_found:             flag for if the flex endstop has been found
#
# Returns:
#   psyonic_homing_counter:     a counter for how many frames the psyonic hand
#                               has been homing for
#   cw_end_found:               flag for if the cw endstop has been found
#   ccw_end_found:              flag for if the ccw endstop has been found
#   ext_end_found:              flag for if the ext endstop has been found
#   flex_end_found:             flag for if the flex endstop has been found
def homeWrist(psyonic_homing_counter,cw_end_found,ccw_end_found,ext_end_found,flex_end_found):
    global c, ml, train_hand, hand_homed, train_hand_condition, hand_timer, first_grip
    global train_rotator, rotator_homed, train_rotator_condition, true_rot_end_found
    global done_moving_cw, rot_counter, done_moving_ccw, train_flexor, flexor_homed
    global train_flexor_condition, true_flex_end_found, done_moving_ext, done_moving_flex
    global flex_counter, home_wrist, mess_id, mess_string, is_training, paused

    # Start by initalizing all motors to have zero
    # velocity (this will be changed depending on which
    # condition you are in)
    ml.rot_vel = 0
    ml.flex_vel = 0
    ml.hand_vel = 0

    # If the hand has been selected to be trained,
    # proceed with moving it
    if train_hand and ml.motor_guided:
        # If the hand has not yet been homed, and it
        # has been less than 2.5 seconds since the 
        # homing button was pressed, close the hand
        if not hand_homed:
            if ml.td_type == 0:
                if utime.ticks_ms() - hand_timer < 2500:
                    # Check to see if you are training hand
                    # open, hand close, or both
                    if train_hand_condition in [1,3]:
                        ml.hand_vel = -50
                    elif train_hand_condition == 2:
                        ml.hand_vel = 50
                # Once this timer elapses, switch the hand
                # homed flag to true and set the motor
                # velocities to be zero
                else:
                    hand_homed = True
                    ml.hand_vel = 0
                    ml.rot_vel = 0
                    ml.flex_vel = 0
                    print("DONE HOMING OTTOBOCK TRANSCARPAL HAND")
            elif ml.td_type == 1:
                if psyonic_homing_counter < 10:
                    # Check to see if you are training hand
                    # open, hand close, or both
                    if train_hand_condition in [1,3]:
                        if first_grip == 0:
                            ml.grip_command_type = ml.mc_psy.power_grasp_cmd
                            ml.hand_vel = 50
                        elif first_grip == 1:
                            ml.grip_command_type = ml.mc_psy.key_grasp_cmd
                            ml.hand_vel = 50
                        elif first_grip == 2:
                            ml.grip_command_type = ml.mc_psy.chuck_grasp_cmd
                            ml.hand_vel = 50
                        elif first_grip == 3:
                            ml.grip_command_type = ml.mc_psy.pinch_grasp_cmd
                            ml.hand_vel = 50
                        elif first_grip == 4:
                            ml.grip_command_type = ml.mc_psy.point_grasp_cmd
                            ml.hand_vel = 50
                        elif first_grip == 5:
                            ml.grip_command_type = ml.mc_psy.sign_of_the_horns_grasp_cmd
                            ml.hand_vel = 50
                        elif first_grip == 6:
                            ml.grip_command_type = ml.mc_psy.trigger_grip_cmd
                            ml.hand_vel = 50
                        elif first_grip == 7:
                            ml.grip_command_type = ml.mc_psy.handshake_grip_cmd
                            ml.hand_vel = 50
                        elif first_grip == 8:
                            ml.grip_command_type = ml.mc_psy.chuck_ok_grasp_cmd
                            ml.hand_vel = 50
                    elif train_hand_condition == 2:
                        ml.grip_command_type = ml.mc_psy.general_open_cmd
                        ml.hand_vel = 125
                    psyonic_homing_counter = psyonic_homing_counter + 1
                else:
                    if 0xff in ml.hand_grip_stat_history:
                        # Check to see if you are training hand
                        # open, hand close, or both
                        if train_hand_condition in [1,3]:
                            if first_grip == 0:
                                ml.grip_command_type = ml.mc_psy.power_grasp_cmd
                                ml.hand_vel = 50
                            elif first_grip == 1:
                                ml.grip_command_type = ml.mc_psy.key_grasp_cmd
                                ml.hand_vel = 50
                            elif first_grip == 2:
                                ml.grip_command_type = ml.mc_psy.chuck_grasp_cmd
                                ml.hand_vel = 50
                            elif first_grip == 3:
                                ml.grip_command_type = ml.mc_psy.pinch_grasp_cmd
                                ml.hand_vel = 50
                            elif first_grip == 4:
                                ml.grip_command_type = ml.mc_psy.point_grasp_cmd
                                ml.hand_vel = 50
                            elif first_grip == 5:
                                ml.grip_command_type = ml.mc_psy.sign_of_the_horns_grasp_cmd
                                ml.hand_vel = 50
                            elif first_grip == 6:
                                ml.grip_command_type = ml.mc_psy.trigger_grip_cmd
                                ml.hand_vel = 50
                            elif first_grip == 7:
                                ml.grip_command_type = ml.mc_psy.handshake_grip_cmd
                                ml.hand_vel = 50
                            elif first_grip == 8:
                                ml.grip_command_type = ml.mc_psy.chuck_ok_grasp_cmd
                                ml.hand_vel = 50
                        elif train_hand_condition == 2:
                            ml.grip_command_type = ml.mc_psy.general_open_cmd
                            ml.hand_vel = 125
                        psyonic_homing_counter = psyonic_homing_counter + 1
                    else:
                        psyonic_homing_counter = 0
                        hand_homed = True
                        ml.hand_vel = 0
                        ml.rot_vel = 0
                        ml.flex_vel = 0
                        print("DONE HOMING PSYONIC ABILITY HAND")

    # Otherwise, just mark that it has been homed so
    # that you can move onto the next DOF (the rotator)
    else:
        hand_homed = True
    # If the hand has been homed, move on to the
    # rotator
    if hand_homed:
        # If you have selected the rotator as a DOF to
        # train, then home it
        if train_rotator and ml.motor_guided:
            # Keep running these commands until you
            # have homed the rotator
            if not rotator_homed:
                # Check to see if you are training
                # pronation, supination, or both
                if train_rotator_condition in [1,3]:
                    # Follow the following commands if the
                    # terminal device is left handed
                    if ml.handedness == 0:
                        # If you have already found the 
                        # physical endstop for the rotator,
                        # then only rotate until you've
                        # exceeded the max cw position
                        if true_rot_end_found:
                            # Continue moving cw until you
                            # have exceeded the max cw
                            # position
                            if not done_moving_cw:
                                ml.rot_vel = ml.rot_vel_ends
                                if ml.rot_cw_pos_exceeded:
                                    print("exceeded rot cw pos")
                                    done_moving_cw = True
                                    ml.rot_vel = 0
                                    ml.flex_vel = 0
                        # Otherwise, drive the motor cw
                        # until the physical endstop is hit
                        else:
                            if not done_moving_cw:
                                if not cw_end_found:
                                    ml.rot_vel = ml.rot_vel_ends
                                else:
                                    done_moving_cw = True
                                    ml.rot_vel = 0
                                    ml.flex_vel = 0
                                    ml.max_cw_pos = ml.rot_pos
                                    print("max_cw_pos: " + str(ml.max_cw_pos))
                                    ml.max_ccw_pos = ml.max_cw_pos - 3900
                                    print("max_ccw_pos: " + str(ml.max_ccw_pos))
                        # Once you have finished moving cw,
                        # move ccw for a few frames just to
                        # ensure that you are not in a high
                        # current state when you move on to
                        # homing the flexor
                        if done_moving_cw:
                            if rot_counter < 5:
                                ml.rot_vel = -1*ml.rot_vel_ends
                                rot_counter = rot_counter + 1
                            # Once you have moved off the
                            # endstop, calculate this new
                            # position as your cw endstop
                            else:
                                true_rot_end_found = True
                                rotator_homed = True
                                ml.rot_endstop_found = True
                                ml.rot_vel = 0
                    # If the terminal device is right
                    # handed, follow the same proecudre,
                    # but move ccw first
                    elif ml.handedness == 1:
                        if true_rot_end_found:
                            if not done_moving_ccw:
                                ml.rot_vel = -1*ml.rot_vel_ends
                                if ml.rot_ccw_pos_exceeded:
                                    done_moving_ccw = True
                                    ml.rot_vel = 0
                                    ml.flex_vel = 0
                        else:
                            if not done_moving_ccw:
                                if not ccw_end_found:
                                    ml.rot_vel = -1*ml.rot_vel_ends
                                else:
                                    done_moving_ccw = True
                                    ml.rot_vel = 0
                                    ml.flex_vel = 0
                                    ml.max_ccw_pos = ml.rot_pos
                                    print("max_ccw_pos: " + str(ml.max_ccw_pos))
                                    ml.max_cw_pos = ml.max_ccw_pos + 3900
                                    print("max_cw_pos: " + str(ml.max_cw_pos))
                        if done_moving_ccw:
                            if rot_counter < 5:
                                ml.rot_vel = ml.rot_vel_ends
                                rot_counter = rot_counter + 1
                            else:
                                true_rot_end_found = True
                                rotator_homed = True
                                ml.rot_endstop_found = True
                                ml.rot_vel = 0
                elif train_rotator_condition == 2:
                    # Follow the following commands if the
                    # terminal device is left handed
                    if ml.handedness == 0:
                        # If you have already found the 
                        # physical endstop for the rotator,
                        # then only rotate until you've
                        # exceeded the max cw position
                        if true_rot_end_found:
                            # Continue moving ccw until you
                            # have exceeded the max ccw
                            # position
                            if not done_moving_ccw:
                                ml.rot_vel = -1*ml.rot_vel_ends
                                if ml.rot_ccw_pos_exceeded:
                                    print("exceeded rot ccw pos")
                                    done_moving_ccw = True
                                    ml.rot_vel = 0
                                    ml.flex_vel = 0
                        # Otherwise, drive the motor ccw
                        # until the physical endstop is hit
                        else:
                            if not done_moving_ccw:
                                if not ccw_end_found:
                                    ml.rot_vel = -1*ml.rot_vel_ends
                                else:
                                    done_moving_ccw = True
                                    ml.rot_vel = 0
                                    ml.flex_vel = 0
                                    ml.max_ccw_pos = ml.rot_pos
                                    print("max_ccw_pos: " + str(ml.max_ccw_pos))
                                    ml.max_cw_pos = ml.max_ccw_pos + 3900
                                    print("max_cw_pos: " + str(ml.max_cw_pos))
                        # Once you have finished moving ccw,
                        # move ccw for a few frames just to
                        # ensure that you are not in a high
                        # current state when you move on to
                        # homing the flexor
                        if done_moving_ccw:
                            if rot_counter < 5:
                                ml.rot_vel = ml.rot_vel_ends
                                rot_counter = rot_counter + 1
                            # Once you have moved off the
                            # endstop, calculate this new
                            # position as your ccw endstop
                            else:
                                true_rot_end_found = True
                                rotator_homed = True
                                ml.rot_endstop_found = True
                                ml.rot_vel = 0
                    # If the terminal device is right
                    # handed, follow the same proecudre,
                    # but move cw first
                    elif ml.handedness == 1:
                        if true_rot_end_found:
                            if not done_moving_cw:
                                ml.rot_vel = ml.rot_vel_ends
                                if ml.rot_cw_pos_exceeded:
                                    done_moving_cw = True
                                    ml.rot_vel = 0
                                    ml.flex_vel = 0
                        else:
                            if not done_moving_cw:
                                if not cw_end_found:
                                    ml.rot_vel = ml.rot_vel_ends
                                else:
                                    done_moving_cw = True
                                    ml.rot_vel = 0
                                    ml.flex_vel = 0
                                    ml.max_cw_pos = ml.rot_pos
                                    print("max_cw_pos: " + str(ml.max_cw_pos))
                                    ml.max_ccw_pos = ml.max_cw_pos - 3900
                                    print("max_ccw_pos: " + str(ml.max_ccw_pos))
                        if done_moving_cw:
                            if rot_counter < 5:
                                ml.rot_vel = -1*ml.rot_vel_ends
                                rot_counter = rot_counter + 1
                            else:
                                true_rot_end_found = True
                                rotator_homed = True
                                ml.rot_endstop_found = True
                                ml.rot_vel = 0
        # Otherwise, just mark that it as having been
        # homed
        else:
            rotator_homed = True
        # Once the rotator has been homed, move on to
        # the flexor
        if rotator_homed:
            # If you have selected the flexor as a DOF
            # to train, then home it
            if train_flexor and ml.motor_guided:
                # Keep running these commands until you
                # have homed the flexor
                if not flexor_homed:
                    # Check if you are training
                    # flexion, extension, or both
                    if train_flexor_condition in [1,3]:
                        # If you have already found the 
                        # physical endstop for the flexor,
                        # then only flex until you've
                        # exceeded the max ext position
                        if true_flex_end_found:
                            # Continue moving ext until you
                            # have exceeded the max ext
                            # position
                            if not done_moving_ext:
                                ml.flex_vel = -1*ml.flex_vel_ends
                                if ml.flex_ext_pos_exceeded:
                                    done_moving_ext = True
                                    ml.rot_vel = 0
                                    ml.flex_vel = 0
                        # Otherwise, drive the motor ext
                        # until the physical endstop is hit
                        else:
                            if not done_moving_ext:
                                if not ext_end_found:
                                    ml.flex_vel = -1*ml.flex_vel_ends
                                else:
                                    done_moving_ext = True
                                    ml.rot_vel = 0
                                    ml.flex_vel = 0
                                    ml.max_ext_pos = ml.flex_pos
                                    print("max_ext_pos: " + str(ml.max_ext_pos))
                                    ml.max_flex_pos = ml.max_ext_pos + 1215
                                    print("max_flex_pos: " + str(ml.max_flex_pos))
                        # Once you have finished moving ext,
                        # move flex for a few frames just to
                        # ensure that you are not in a high
                        # current state when you move on to
                        # training
                        if done_moving_ext:
                            if flex_counter < 5:
                                ml.flex_vel = ml.flex_vel_ends
                                flex_counter = flex_counter + 1
                            # Once you have moved off the
                            # endstop, calculate this new
                            # position as your flex endstop
                            else:
                                true_flex_end_found = True
                                flexor_homed = True
                                ml.flex_endstop_found = True
                    if train_flexor_condition == 2:
                        # If you have already found the 
                        # physical endstop for the flexor,
                        # then only flex until you've
                        # exceeded the max flex position
                        if true_flex_end_found:
                            # Continue moving flex until you
                            # have exceeded the max flex
                            # position
                            if not done_moving_flex:
                                ml.flex_vel = ml.flex_vel_ends
                                if ml.flex_flex_pos_exceeded:
                                    done_moving_flex = True
                                    ml.rot_vel = 0
                                    ml.flex_vel = 0
                        # Otherwise, drive the motor ext
                        # until the physical endstop is hit
                        else:
                            if not done_moving_flex:
                                if not flex_end_found:
                                    ml.flex_vel = ml.flex_vel_ends
                                else:
                                    done_moving_flex = True
                                    ml.rot_vel = 0
                                    ml.flex_vel = 0
                                    ml.max_flex_pos = ml.flex_pos
                                    print("max_flex_pos: " + str(ml.max_flex_pos))
                                    ml.max_ext_pos = ml.max_flex_pos - 1215
                                    print("max_ext_pos: " + str(ml.max_ext_pos))
                        # Once you have finished moving flex,
                        # move ext for a few frames just to
                        # ensure that you are not in a high
                        # current state when you move on to
                        # training
                        if done_moving_flex:
                            if flex_counter < 5:
                                ml.flex_vel = -1*ml.flex_vel_ends
                                flex_counter = flex_counter + 1
                            # Once you have moved off the
                            # endstop, calculate this new
                            # position as your flex endstop
                            else:
                                true_flex_end_found = True
                                flexor_homed = True
                                ml.flex_endstop_found = True
            # Otherwise, just mark that it as having been
            # homed
            else:
                flexor_homed = True
            # Once you have finished homing each DOF,
            # send a message back to the mobile app
            # confirming that you have finished homing
            if flexor_homed:
                home_wrist = False
                if not c._device_based_training:
                    mess_id = 0x0d
                    mess_string = b' Done Homing'
                    addMessageToList()
                c.sendStart()
                if c._device_based_training:
                    c.sendStartTrain()
                # Reset flags for next time you home
                hand_homed = False
                rotator_homed = False
                flexor_homed = False
                train_hand = False
                train_rotator = False
                train_flexor = False
                rot_counter = 0
                flex_counter = 0
                done_moving_cw = False
                done_moving_ccw = False
                done_moving_ext = False
                done_moving_flex = False
                is_training = True
                p_rec._is_training = True
                paused = False

    return [psyonic_homing_counter,cw_end_found,ccw_end_found,ext_end_found,flex_end_found]

###############################################################################

###############################################################################
###REQUIRED FORMATTING TO START ALL ASYNCHRONOUS FUNCTIONS###
###############################################################################

eloop = uasyncio.new_event_loop()
eloop.create_task(handleMessages())
eloop.create_task(parseBT())
eloop.create_task(motorCommands())
eloop.create_task(handleFrames())
eloop.run_forever()

###############################################################################