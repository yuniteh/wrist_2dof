###############################################################################
# This class handles communication to the Dynamixel motors over UART.
#
#
# Author: Kevin Brenner
# Date: June 24, 2021
###############################################################################

import binascii, crc_calc, math, machine, pyb, time, uasyncio, ubinascii, ubluetooth, utime
from pyb import Pin, UART

class MotorControlDynamixel:

    # Dynamixel instruction packet headers
    header_string =             "fffffd00"
    rotator_motor =             "01"
    flexor_motor =              "02"
    reboot =                    "030008"
    enable_torque =             "060003400001"
    disable_torque =            "060003400000"
    init_pwm_limit =            "07000324000002"
    
    write_baud =                "0600030800"
    read_baud =                 "07000208000100"
    write_control =             "0600030b00"
    curr_control =              "00"
    vel_control =               "01"
    pos_control =               "03"
    ext_pos_control =           "04"
    curr_pos_control =          "05"
    
    write_bus_watchdog =        "0600036200"
    write_shutdown =            "0600033f003d"
    sync_write_vel_and_pos =    "fe1900837000080001"
    sync_read_ind =             "fe090082E00008000102"
    init_ind1 =                 "070003A8008400" #1st byte of position data
    init_ind2 =                 "070003AA008500" #2nd byte of position data
    init_ind3 =                 "070003AC008600" #3rd byte of position data
    init_ind4 =                 "070003AE008700" #4th byte of position data
    init_ind5 =                 "070003B0007E00" #1st byte of current data
    init_ind6 =                 "070003B2007F00" #2nd byte of current data
    init_ind7 =                 "070003B4006200" #1st byte of bus watchdog data
    init_ind8 =                 "070003B6004600" #1st byte of hardware error status data

    # Generating the command string for indirect sync read (this string will
    # not change throughout normal operation, which is why we initailize it
    # only one time)
    [crc_sync_read_pos,final_sync_read_ind] = crc_calc.createCommandString(header_string + sync_read_ind)

    def __init__(self):
        self.wrist_pin_out = Pin('X2', Pin.OUT_PP)
        self.uart = UART(2, 2000000)
        self.uart.init(baudrate = 2000000, bits=8,parity=None,stop=1,rxbuf = 200)

    # This function will handle the uart.read() command for all communication
    # to the Dynamixel motors
    #
    # Inputs:
    #   resp_len:       number of bytes to be read
    #
    # Returns:
    #   r_val:          flag of whether or not the read was successful
    #   resp_val:       bytes read from UART    
    def readSetting_BytePack(self,resp_len):
        r_val = True
        self.wrist_pin_out.low()
        pyb.udelay(500)
        dum = self.uart.any()
        try_read_counter = 0
        while dum < resp_len:
            pyb.udelay(10)
            dum = self.uart.any()
            try_read_counter = try_read_counter + 1
            if try_read_counter > 75:
                dum = 1000
        if dum != 1000:
            pyb.udelay(10)
            dum = self.uart.any()
            resp_val = bytearray(dum)
            self.uart.readinto(resp_val)
        else:
            resp_val = bytearray(2)
            r_val = False
        return [r_val,resp_val]

    # Enabling a specific motor's torque to high
    #
    # Inputs:
    #   motor_id:       id corresponding to specific motor
    #
    # Returns:
    #   N/A
    def enableMotor(self, motor_id):
        print("enabling " + str(motor_id))
        # Write a command message to a motor to enable it
        self.wrist_pin_out.high()
        pyb.udelay(100)
        command_v = self.header_string + motor_id + self.enable_torque 
        [crc,final_s] = crc_calc.createCommandString(command_v)
        self.uart.write(final_s)

        # Read the response from the motor
        pyb.udelay(100)
        tmp = bytearray(11)
        [resp,resp_val] = self.readSetting_BytePack(11)
        if resp:
            # Convert the returned byte array into a bytes object
            tmp_str = binascii.hexlify(resp_val)
            
            # Separate main instruction packet from crc
            tmp_no_crc = tmp_str[0:-4]
            tmp_crc = tmp_str[-4:]
            
            # Recalculate the command string as if the CRC wasn't there in order
            # to see what the CRC should have been
            [crc_final_tmp,final_tmp] = crc_calc.createCommandString(tmp_no_crc)
            
            # Compare crc's to ensure the correct CRC was used
            if crc_final_tmp not in tmp_crc:
                print("Enable motor CRC's are incorrect")
                print("crc_final_tmp: " + str(crc_final_tmp))
                print("tmp_crc: " + str(tmp_crc))
                a = 1/0

        else:
            a = 1/0
            
    # Disabling a specific motor's torque to low
    # Enabling a specific motor's torque to high
    #
    # Inputs:
    #   motor_id:       id corresponding to specific motor
    #
    # Returns:
    #   N/A
    def disableMotor(self, motor_id):
        print("disabling " + str(motor_id))
        # Write a command message to a motor to disable it
        self.wrist_pin_out.high()
        pyb.udelay(100)
        command_v = self.header_string + motor_id + self.disable_torque
        [crc,final_s] = crc_calc.createCommandString(command_v)
        self.uart.write(final_s)
        
        # Read the response from the motor
        pyb.udelay(100)
        tmp = bytearray(11)
        [resp,resp_val] = self.readSetting_BytePack(11)
        if resp:
            # Convert the returned byte array into a bytes object
            tmp_str = binascii.hexlify(resp_val)
            
            # Separate main instruction packet from crc
            tmp_no_crc = tmp_str[0:-4]
            tmp_crc = tmp_str[-4:]
            
            # Recalculate the command string as if the CRC wasn't there in order
            # to see what the CRC should have been
            [crc_final_tmp,final_tmp] = crc_calc.createCommandString(tmp_no_crc)
            
            # Compare crc's to ensure the correct CRC was used
            if crc_final_tmp not in tmp_crc:
                print("Disable motor CRC's are incorrect")
                print("crc_final_tmp: " + str(crc_final_tmp))
                print("tmp_crc: " + str(tmp_crc))
                a = 1/0
                
        else:
            a = 1/0

    # Write the baud rate for a specific motor
    #
    # Inputs:
    #   motor_id:       id corresponding to specific motor
    #   baud_rate:      baud rate to set specific motor to
    #
    # Returns:
    #   N/A
    def writeBaudRate(self, motor_id, baud_rate):
        if baud_rate == 9600:
            id = "00"
        elif baud_rate == 57600:
            id = "01"
        elif baud_rate == 115200:
            id = "02"
        elif baud_rate == 1000000:
            id = "03"
        elif baud_rate == 2000000:
            id = "04"
        elif baud_rate == 3000000:
            id = "05"
        elif baud_rate == 4000000:
            id = "06"
        elif baud_rate == 4500000:
            id = "07"
    
        # Write a command message to a motor to modify its baud rate
        self.wrist_pin_out.high()
        pyb.udelay(100)
        command_v = self.header_string + motor_id + self.write_baud + id
        print("command_v: " + str(command_v))
        [crc,final_s] = crc_calc.createCommandString(command_v)
        print("final_s: " + str(final_s))
        self.uart.write(final_s)
        
        # Read the response from the motor
        pyb.udelay(100)
        tmp = bytearray(11)
        [resp,resp_val] = self.readSetting_BytePack(11)
        print("resp_val: " + str(resp_val))
        if resp:
            # Convert the returned byte array into a bytes object
            tmp_str = binascii.hexlify(resp_val)
            
            # Separate main instruction packet from crc
            tmp_no_crc = tmp_str[0:-4]
            tmp_crc = tmp_str[-4:]
            
            # Recalculate the command string as if the CRC wasn't there in order
            # to see what the CRC should have been
            [crc_final_tmp,final_tmp] = crc_calc.createCommandString(tmp_no_crc)
            print("crc_final_tmp: " + str(crc_final_tmp))
            print("tmp_crc: " + str(tmp_crc))
            # Compare crc's to ensure the correct CRC was used
            if crc_final_tmp not in tmp_crc:
                print("Write control mode CRC's are incorrect")
                print("crc_final_tmp: " + str(crc_final_tmp))
                print("tmp_crc: " + str(tmp_crc))
                a = 1/0
                
        else:
            print("Unable to write control mode")
            self.disableMotor(self.rotator_motor)
            self.disableMotor(self.flexor_motor)
            a = 1/0
        
    # Read in the baud rate for a specific motor
    #
    # Inputs:
    #   motor_id:       id corresponding to specific motor
    #
    # Returns:
    #   baud_rate:      baud_rate of specific motor
    def readBaudRate(self, motor_id):
        # Write a command message to a motor to modify its baud rate
        self.wrist_pin_out.high()
        pyb.udelay(100)
        command_v = self.header_string + motor_id + self.read_baud
        [crc,final_s] = crc_calc.createCommandString(command_v)
        print("final_s: " + str(final_s))
        self.uart.write(final_s)
        
        # Read the response from the motor
        pyb.udelay(100)
        tmp = bytearray(11)
        [resp,resp_val] = self.readSetting_BytePack(11)
        
        print("resp_val: " + str(resp_val))
        # Convert the returned byte array into a bytes object
        baud_rate_str = binascii.hexlify(resp_val)
        print("baud_rate_str: " + str(baud_rate_str))
        baud_rate = crc_calc.signedPos(baud_rate_str)
        print("baud_rate: " + str(baud_rate))

        return baud_rate
    
    # Reboot a specific motor
    #
    # Inputs:
    #   motor_id:       id corresponding to specific motor
    #
    # Returns:
    #   N/A
    def rebootMotor(self, motor_id):
        print("rebooting " + str(motor_id))
        self.wrist_pin_out.high()
        pyb.udelay(100)
        command_v = self.header_string + motor_id + self.reboot
        [crc,final_s] = crc_calc.createCommandString(command_v)
        self.uart.write(final_s)
        
        pyb.udelay(100)
        tmp = bytearray(11)
        [resp,resp_val] = self.readSetting_BytePack(11)
        if resp:
            # Convert the returned byte array into a bytes object
            tmp_str = binascii.hexlify(resp_val)
            
            # Separate main instruction packet from crc
            tmp_no_crc = tmp_str[0:-4]
            tmp_crc = tmp_str[-4:]
            
            # Recalculate the command string as if the CRC wasn't there in order
            # to see what the CRC should have been
            [crc_final_tmp,final_tmp] = crc_calc.createCommandString(tmp_no_crc)
            
            # Compare crc's to ensure the correct CRC was used
            if crc_final_tmp not in tmp_crc:
                print("Reboot motor CRC's are incorrect")
                print("crc_final_tmp: " + str(crc_final_tmp))
                print("tmp_crc: " + str(tmp_crc))
                a = 1/0

    # Setting the operation mode for a specific motor (velocity control,
    # extended position control, etc.)
    #
    # Inputs:
    #   motor_id:       id corresponding to specific motor
    #   control_mode:   an identifier for the control mode you want to run
    #
    # Returns:
    #   N/A
    def writeControlMode(self, motor_id, control_mode):
        # Write a command message to a motor to write the control mode (velocity,
        # extended position, etc.)
        self.wrist_pin_out.high()
        pyb.udelay(100)
        command_v = self.header_string + motor_id + self.write_control + control_mode
        [crc,final_s] = crc_calc.createCommandString(command_v)
        self.uart.write(final_s)

        # Read the response from the motor
        pyb.udelay(100)
        tmp = bytearray(11)
        [resp,resp_val] = self.readSetting_BytePack(11)

        if resp:
            # Convert the returned byte array into a bytes object
            tmp_str = binascii.hexlify(resp_val)
            
            # Separate main instruction packet from crc
            tmp_no_crc = tmp_str[0:-4]
            tmp_crc = tmp_str[-4:]
            
            # Recalculate the command string as if the CRC wasn't there in order
            # to see what the CRC should have been
            [crc_final_tmp,final_tmp] = crc_calc.createCommandString(tmp_no_crc)
            
            # Compare crc's to ensure the correct CRC was used
            if crc_final_tmp not in tmp_crc:
                print("Write control mode CRC's are incorrect")
                print("crc_final_tmp: " + str(crc_final_tmp))
                print("tmp_crc: " + str(tmp_crc))
                a = 1/0
                
        else:
            print("Unable to write control mode")
            self.disableMotor(self.rotator_motor)
            self.disableMotor(self.flexor_motor)
            a = 1/0
    
    # Writing the shutdown conditions for a specific motor
    #
    # Inputs:
    #   motor_id:       id corresponding to specific motor
    #
    # Returns:
    #   N/A
    def writeShutdown(self,motor_id):
        self.wrist_pin_out.high()
        pyb.udelay(100)
        command_v = self.header_string + motor_id + self.write_shutdown
        [crc,final_s] = crc_calc.createCommandString(command_v)
        self.uart.write(final_s)
        
        # Read the response from the motor
        pyb.udelay(100)
        tmp = bytearray(11)
        [resp,resp_val] = self.readSetting_BytePack(11)

        if resp:
            # Convert the returned byte array into a bytes object
            tmp_str = binascii.hexlify(resp_val)
            
            # Separate main instruction packet from crc
            tmp_no_crc = tmp_str[0:-4]
            tmp_crc = tmp_str[-4:]
            
            # Recalculate the command string as if the CRC wasn't there in order
            # to see what the CRC should have been
            [crc_final_tmp,final_tmp] = crc_calc.createCommandString(tmp_no_crc)
            
            # Compare crc's to ensure the correct CRC was used
            if crc_final_tmp not in tmp_crc:
                print("Write shutdown CRC's are incorrect")
                print("crc_final_tmp: " + str(crc_final_tmp))
                print("tmp_crc: " + str(tmp_crc))
                a = 1/0
                
        else:
            print("Unable to write shutdown")
            self.disableMotor(self.rotator_motor)
            self.disableMotor(self.flexor_motor)
            a = 1/0

    # Write a PWM limit for a specific motor
    #
    # Inputs:
    #   motor_id:       id corresponding to specific motor
    #   pwm_limit:      a limit to the PWM capacity of the motor
    #
    # Returns:
    #   N/A
    def writePWMLimit(self,motor_id,pwm_limit):
        self.wrist_pin_out.high()
        pyb.udelay(100)
        command_v = self.header_string + motor_id + pwm_limit
        [crc,final_s] = crc_calc.createCommandString(command_v)
        self.uart.write(final_s)
        # Read the response from the motor
        pyb.udelay(100)
        tmp = bytearray(11)
        [resp,resp_val] = self.readSetting_BytePack(11)

        if resp:
            # Convert the returned byte array into a bytes object
            tmp_str = binascii.hexlify(resp_val)
            
            # Separate main instruction packet from crc
            tmp_no_crc = tmp_str[0:-4]
            tmp_crc = tmp_str[-4:]
            
            # Recalculate the command string as if the CRC wasn't there in order
            # to see what the CRC should have been
            [crc_final_tmp,final_tmp] = crc_calc.createCommandString(tmp_no_crc)
            
            # Compare crc's to ensure the correct CRC was used
            if crc_final_tmp not in tmp_crc:
                print("PWM limit CRC's are incorrect")
                print("crc_final_tmp: " + str(crc_final_tmp))
                print("tmp_crc: " + str(tmp_crc))
                a = 1/0
                
        else:
            print("Unable to set PWM")
            self.disableMotor(self.rotator_motor)
            self.disableMotor(self.flexor_motor)
            a = 1/0

    # Writing a bus watchdog value to a specific motor
    #
    # Inputs:
    #   motor_id:       id corresponding to specific motor
    #   value:          the raw watchdog value
    #
    # Returns:
    #   N/A
    def writeBusWatchdog(self,motor_id,value):
        watchdog_val = crc_calc.int2HexSize(value,1)
        self.wrist_pin_out.high()
        pyb.udelay(100)
        command_v = self.header_string + motor_id + self.write_bus_watchdog + watchdog_val
        [crc,final_s] = crc_calc.createCommandString(command_v)
        self.uart.write(final_s)
        
        # Read the response from the motor
        pyb.udelay(100)
        tmp = bytearray(11)
        [resp,resp_val] = self.readSetting_BytePack(11)

        if resp:
            # Convert the returned byte array into a bytes object
            tmp_str = binascii.hexlify(resp_val)
            
            # Separate main instruction packet from crc
            tmp_no_crc = tmp_str[0:-4]
            tmp_crc = tmp_str[-4:]
            
            # Recalculate the command string as if the CRC wasn't there in order
            # to see what the CRC should have been
            [crc_final_tmp,final_tmp] = crc_calc.createCommandString(tmp_no_crc)
            
            # Compare crc's to ensure the correct CRC was used
            if crc_final_tmp not in tmp_crc:
                print("Write bus watchdog CRC's are incorrect")
                print("crc_final_tmp: " + str(crc_final_tmp))
                print("tmp_crc: " + str(tmp_crc))
                a = 1/0
                
        else:
            print("Unable to write bus watchdog")
            self.disableMotor(self.rotator_motor)
            self.disableMotor(self.flexor_motor)
            a = 1/0
    
    # Writing to set the initial indirect addresses
    # These indirect addresses are used to map the addresses of the actual data
    # in the Control Table in such a way that data the entries are sequential
    # and can be unpacked all at once by reading data sequentially and then
    # unpacking it
    #
    # Inputs:
    #   motor_id:       id corresponding to specific motor
    #   command_id:     specific byte of data of interest
    #
    # Returns:
    #   N/A
    def initIndirect(self,motor_id,command_id):
        self.wrist_pin_out.high()
        pyb.udelay(100)
        command_v = self.header_string + motor_id + command_id
        [crc,final_s] = crc_calc.createCommandString(command_v)
        self.uart.write(final_s)
        
        # Read the response from the motor
        pyb.udelay(100)
        tmp = bytearray(11)
        [resp,resp_val] = self.readSetting_BytePack(11)

        if resp:
            # Convert the returned byte array into a bytes object
            tmp_str = binascii.hexlify(resp_val)
            
            # Separate main instruction packet from crc
            tmp_no_crc = tmp_str[0:-4]
            tmp_crc = tmp_str[-4:]
            
            # Recalculate the command string as if the CRC wasn't there in order
            # to see what the CRC should have been
            [crc_final_tmp,final_tmp] = crc_calc.createCommandString(tmp_no_crc)
            
            # Compare crc's to ensure the correct CRC was used
            if crc_final_tmp not in tmp_crc:
                print("Init indirect addresses CRC's are incorrect")
                print("crc_final_tmp: " + str(crc_final_tmp))
                print("tmp_crc: " + str(tmp_crc))
                a = 1/0
                
        else:
            print("Unable to init indirect addresses")
            self.disableMotor(self.rotator_motor)
            self.disableMotor(self.flexor_motor)
            a = 1/0
    
    # Writing desired position and profile velocity to the indirect addresses
    #
    # Inputs:
    #   N/A
    #
    # Returns:
    #   N/A
    def writeToMotorsInd(self,rot_pos_des,flex_pos_des):
        self.wrist_pin_out.high()

        pyb.udelay(100)
        command_v = self.header_string + self.sync_write_vel_and_pos + crc_calc.int2Hex(0) + crc_calc.int2Hex(int(rot_pos_des)) + "02" + crc_calc.int2Hex(0)  +  crc_calc.int2Hex(int(flex_pos_des))
        crc_conversion_time = utime.ticks_us()
        [crc,final_s] = crc_calc.createCommandString(command_v)
        self.uart.write(final_s)
    
    # Reading in information from each motor
    #
    # Inputs:
    #   N/A
    #
    # Returns:
    #   rot_pos:        position of the rotator
    #   flex_pos:       position of the flexor
    #   rot_curr:       current of the rotator
    #   flex_curr:      current of the flexor
    #   rot_watch:      watchdog value of the rotator
    #   flex_watch:     watchdog value of the flexor
    #   rot_error:      error status of the rotator
    #   flex_error:     error status of the flexor
    def syncReadInd(self):
        time_in_sync_read_pos = utime.ticks_us()
        self.wrist_pin_out.high()
        pyb.udelay(100)
        self.uart.write(self.final_sync_read_ind)
        pyb.udelay(100)
        tmp = bytearray(38)
        [resp,resp_val] = self.readSetting_BytePack(38)
        
        if resp:
            # Convert the returned byte array into a bytes object
            pos_str = binascii.hexlify(resp_val)
            
            # Figure out where in the bytes object each command is located and then
            # separate the object
            res = [i for i in range(len(str(pos_str))) if pos_str.startswith('fffffd00',i)]
            
            reading_1 = pos_str[res[0]:res[1]]
            reading_2 = pos_str[res[1]:]
            
            # Separate main instruction packet from crc
            reading_1_no_crc = reading_1[0:-4]
            reading_1_crc = reading_1[-4:]
            reading_2_no_crc = reading_2[0:-4]
            reading_2_crc = reading_2[-4:]
            
            # Recalculate the command string as if the CRC wasn't there in order
            # to see what the CRC should have been
            [crc_final_read_1,final_read_1] = crc_calc.createCommandString(reading_1_no_crc)
            [crc_final_read_2,final_read_2] = crc_calc.createCommandString(reading_2_no_crc)
            
            # Compare crc's to ensure the correct CRC was used
            if crc_final_read_1 not in reading_1_crc or crc_final_read_2 not in reading_2_crc:
                print("pos_str: " + str(pos_str))
                print("Reading position/current CRC's are incorrect")
                a = 1/0
            
            if 'fffffdfd' in reading_1:
                reading_1_final = str(reading_1).replace('fffffdfd','fffffd')
                reading_1 = reading_1_final
            if 'fffffdfd' in reading_2:
                reading_2_final = str(reading_2).replace('fffffdfd','fffffd')
                reading_2 = reading_2_final
            
            [pos_1,curr_1,watch_1,error_1] = crc_calc.signedIndPos(reading_1)
            [pos_2,curr_2,watch_2,error_2] = crc_calc.signedIndPos(reading_2)

            if error_1 != 0 or error_2 != 0:
                print("pos1: " + str(pos1))
                print("pos2: " + str(pos2))
                print("curr1: " + str(curr1))
                print("curr2: " + str(curr2))
                print("watch1: " + str(watch1))
                print("watch2: " + str(watch2))
                print("error1: " + str(error1))
                print("error2: " + str(error2))

            if 'fffffd0001' in str(reading_1):
                return [pos_1,pos_2,curr_1,curr_2,watch_1,watch_2,error_1,error_2]
            elif 'fffffd0002' in str(reading_1):
                return [pos_2,pos_1,curr_2,curr_1,watch_2,watch_1,error_2,error_1]
        else:
            print("Unable to uart.read() positions/currents")
            a = 1/0