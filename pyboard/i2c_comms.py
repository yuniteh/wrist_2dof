###############################################################################
# This class handles communication to the various devices using the I2C
# communication protocol.  
#
#
# Author: Kevin Brenner
# Date: June 24, 2021
###############################################################################
import machine, pyb, time
from machine import I2C
from pyb import Pin

class I2CComms:

    # Desired hand velocities
    hand_vel = 0
    prev_hand_vel = 0

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
    utility_cmd = 0x06
    mode_switch = 0x08
    point_grasp_cmd = 0x09
    rude_point_cmd = 0x0A
    relax_cmd = 0x0C
    chuck_grasp_cmd = 0x0F
    handshake_grip_cmd = 0x11
    trigger_grip_cmd = 0x18

    def __init__(self,i2c):
        self.i2c = i2c
        self.psyonic_add = 0x50
        self.i2c_mess = bytearray(3) # command type, grip command, speed
        self.grasp_recv_nbytes = 8 # Taken directly from Psyonic's API 

    # Writes a message over I2C to the Psyonic Ability Hand
    # command_type = 0x1D for setting a grasp and 0x48 for status
    # grip_command = one of the commands listed above (power, key, etc.)
    # speed = value from 0 to 255 with desired speed of motion
    def writePsyonic(self,command_type,grip_command,speed):
        self.i2c_mess[0] = command_type
        self.i2c_mess[1] = grip_command
        self.i2c_mess[2] = speed
        self.i2c.writeto(self.psyonic_add,self.i2c_mess)

    # Reads a message over I2C from the Psyonic Ability Hand 
    def readPsyonic(self):
        hand_status = self.i2c.readfrom(self.psyonic_add,self.grasp_recv_nbytes)
        return hand_status

    # Tries calling the read command to detect if the Psyonic Ability Hand is
    # connected or not
    def detectPsyonicHand(self):
        psyonic_attached = False
        try:
            hand_status = self.readPsyonic()
            psyonic_attached = True
        except:
            psyonic_attached = False
        return psyonic_attached