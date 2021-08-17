###############################################################################
# This class handles communication to the Psyonic Ability Hand over I2C.
#
#
# Author: Kevin Brenner
# Date: June 24, 2021
###############################################################################

import machine, math, pyb, time
from machine import I2C
from pyb import Pin

class MotorControlPsyonic:

    # Command Types
    hand_set_grasp = 0x1D
    hand_status = 0x48

    # Grip Commands
    general_open_cmd = 0x00
    power_grasp_cmd = 0x01
    key_grasp_cmd = 0x02
    pinch_grasp_cmd = 0x03
    chuck_ok_grasp_cmd = 0x04
    sign_of_the_horns_grasp_cmd = 0x05
    utility_cmd = 0x06 # NOT WORKING ON LEFT PSYONIC ABILITY HAND
    mode_switch = 0x08
    point_grasp_cmd = 0x09
    rude_point_cmd = 0x0A
    relax_cmd = 0x0C
    chuck_grasp_cmd = 0x0F
    handshake_grip_cmd = 0x11
    trigger_grip_cmd = 0x18

    def __init__(self):
    	self.scl_pin = Pin('Y9', Pin.OUT_PP)
    	self.sda_pin = Pin('Y10', Pin.OUT_PP)
    	self.i2c = machine.I2C(2,freq=100000)
        self.psyonic_add = 0x50
        self.i2c_mess = bytearray(3) # command type, grip command, speed
        self.grasp_recv_nbytes = 8 # Taken directly from Psyonic's API
        self.grip_command_type = self.general_open_cmd

    # Writes a message over I2C to the Psyonic Ability Hand
    #
    # Inputs:
    #	hand_activated:		flag for whether or not we want the hand to move
    #	command_type:		0x1D for setting a grasp and 0x48 for status
    #	grip_command:		one of the commands listed above (power, key, etc.)
    #	raw_hand_vel:		value from 0 to 255 with desired speed of motion
    #
    # Returns:
    #	N/A
    def writePsyonic(self,hand_activated,command_type,grip_command,raw_hand_vel):
    	if hand_activated:
            hand_vel = int(math.fabs(raw_hand_vel))
        else:
            hand_vel = 0

        self.i2c_mess[0] = command_type
        self.i2c_mess[1] = grip_command
        self.i2c_mess[2] = hand_vel
        self.i2c.writeto(self.psyonic_add,self.i2c_mess)

    # Reads a message over I2C from the Psyonic Ability Hand
    #
    # Inputs:
    #	N/A
    #
    # Returns:
    #	hand_status:		a status message indicating the current grip
    #						sequence you are or 0xFF if you have completed it
    def readPsyonic(self):
        hand_status = self.i2c.readfrom(self.psyonic_add,self.grasp_recv_nbytes)
        return hand_status

    # Tries calling the read command to detect if the Psyonic Ability Hand is
    # connected or not
    #
    # Inputs:
    #	N/A
    #
    # Returns:
    #	psyonic_attached:	a flag that detects whether or not an Ability Hand
    #						is attached to the wrist
    def detectPsyonicHand(self):
        psyonic_attached = False
        try:
            hand_status = self.readPsyonic()
            psyonic_attached = True
        except:
            psyonic_attached = False
        return psyonic_attached