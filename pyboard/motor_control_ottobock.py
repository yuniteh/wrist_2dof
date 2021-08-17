###############################################################################
# This class handles communication to the Ottobock Transcarpal Hand over
# analog lines.
#
#
# Author: Kevin Brenner
# Date: June 24, 2021
###############################################################################

import machine, math, pyb, time
from pyb import DAC

class MotorControlOttobock:

    def __init__(self):
        self.hand_open_dac = DAC(2)
        self.hand_close_dac = DAC(1)

    # Writing a desired velocity to either hand open or hand close
    def writeOttobock(self,hand_activated,raw_hand_vel):
        if hand_activated:
            # Scaling the hand speed for analog output
            hand_vel = math.fabs(raw_hand_vel) * (255 / 100)
        else:
            hand_vel = 0

        if raw_hand_vel > 0:
            self.hand_close_dac.write(0)
            self.hand_open_dac.write(int(hand_vel))
        elif raw_hand_vel < 0:
            self.hand_open_dac.write(0)
            self.hand_close_dac.write(int(hand_vel))
        else:
            self.hand_open_dac.write(0)
            self.hand_close_dac.write(0)