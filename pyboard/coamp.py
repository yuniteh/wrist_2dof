###############################################################################
# This class handles communication between a pyboard D-series and a Coapt
# Coamp/CBM MyoMini (or MyoIMU)
# This requires that a functioning CAN tranceiver, and also assumes that ulab
# firmware has been included on the pyboard build
# (https://micropython-ulab.readthedocs.io/en/latest/ulab.html)
# This also assumes the CoAmp has a NID of 0x50. It only supports 16 bit EMG
# samples currently
#
#Authors: Levi J Hargrove and Kevin Brenner
#Date: June 24, 2021
###############################################################################

#import some packages
import pyb, utime, ubinascii, micropython, os
import ulab as np
from pyb import CAN



# code to be run in micropython
def timeit(f, *args, **kwargs):
    func_name = str(f).split(' ')[1]
    def new_func(*args, **kwargs):
        t = utime.ticks_us()
        result = f(*args, **kwargs)
        print('execution time: ', utime.ticks_diff(utime.ticks_us(), t), ' us')
        return result
    return new_func

class Coamp:
  # Initialization function. Set up variables, register callbacks, etc.
  def __init__(self, can,simulate_flag=False):
    self._new_frame = False
    self._circ_buffer_size = 60
    self._tv = []
    self._fbuf = []
    self._fmv = []
    self._can_streaming = 0x00
    for x in range(self._circ_buffer_size):
      self._fbuf.append(bytearray(8))
    for x in range(self._circ_buffer_size):
      self._fmv.append(memoryview(self._fbuf[x]))
    for x in range(self._circ_buffer_size):
      self._tv.append([0,0,0,self._fmv[x]])

    # Initializing the CAN class
    self._can = can
    self._simulate_flag = simulate_flag
    if self._simulate_flag:
      self._can.init(self._can.LOOPBACK)
    else:
      self._can.init(can.NORMAL, extframe=False, prescaler=3, sjw=1, bs1=11, bs2=2)
    self._sim_seq_count = 0
    # Initializing empty arrays for various messages such as updating the
    # sampling frequency as well as the emg gains
    self._simulated_message1 = bytearray(7)
    self._simulated_message2 = bytearray(7)
    self._simulated_message3 = bytearray(5)
    self._samp_freq_message = bytearray(5)
    self._gain_message = bytearray(5)
    # Initializing CAN messages for the socket switch.
    # Each message has a different command to represent a different buzz sound.
    self._start_train_message = bytearray([0x54,0x03,0xE8,0x00,0x64,0x00,0x19])
    self._train_pause_message = bytearray([0x54,0x00,0x00,0x00,0x00,0x00,0x19])
    self._stop_train_message = bytearray([0x54,0x05,0xDC,0x00,0xFA,0x00,0x19])
    self._train_success_message_0 = bytearray([0x54,0x04,0x17,0x00,0x64,0x00,0x19])
    self._train_success_message_1 = bytearray([0x54,0x05,0x28,0x00,0x64,0x00,0x19])
    self._train_success_message_2 = bytearray([0x54,0x06,0x21,0x00,0x64,0x00,0x19])
    self._train_success_message_3 = bytearray([0x54,0x08,0x31,0x01,0xF4,0x00,0x19])
    # Flags being initialized to assist with device based training
    self._device_based_training = False
    self._dbt_toggle = False
    self._stop_dbt_toggle = False
    # Initializing the EMG gains
    self._emg_gains = [1,1,1,1,1,1,1,1]

    # Ideally (_r_c and _p_c), and (_r_m and p_m) would stay in sync. However, it is possible that 
    # messages are recieved faster than they can be processed (when making a pattern
    # recognition decision, or saving a frame of data, for example). These variables 

    self._r_c = 0     #received counter (in circular buffer)
    self._p_c = 0     #processed counter (from the circular buffer)
    self._r_m = 0     #total messages received
    self._p_m = 0     #total messages counted

    # NID counters
    self._n1_c = 0    
    self._n2_c = 0
    self._n3_c = 0
    self._nchans = 8
    self._frame_len = 20

    self._tim = None
    # We need to use interleaved buffers. When one buffer is being processed
    # the other buffer should be filled with incoming data
    self._active_buffer = 1
    self._frame_full = [0,0,0]
    self._buffer_filling = [0,0,0]
    self._daq_data1 =  np.zeros((self._frame_len,self._nchans),dtype=np.uint16)
    self._daq_data2 =  np.zeros((self._frame_len,self._nchans),dtype=np.uint16)

    self._dd = [np.zeros((self._frame_len,self._nchans),dtype=np.uint16),np.zeros((self._frame_len,self._nchans),dtype=np.uint16)]

    # Setting CAN filters for both the EMG signals and the socket switch
    self._can.setfilter(0, CAN.LIST16, 0, (0x451,0x452,0x453,0x454))
    self._can.setfilter(1, CAN.LIST16, 0, (0xC0,0xC1,0x1C0,0x2C1))
    self._can.rxcallback(0, self.cb0)


  def checkFrameFlag(self):
    return self._new_frame

  def resetFrameFlag(self):
    self._new_frame = False

  def checkStreaming(self):
    return self._can_streaming

  def cb0(self,bus,reason):
    # If reason == 0, it means that a message is available in the CAN buffer
    # Read it into the circular buffer, increment the circular buffer counter 
    # and the total message counter
    if reason == 0:
      self._can.recv(0,self._tv[self._r_c])
      self._r_c = self._r_c + 1
      self._r_m = self._r_m + 1
      if self._r_c>=self._circ_buffer_size:
        self._r_c = 0 
            
    # If reason ==1, it means that 3 messages are available in the CAN buffer
    # Read them all in
    if reason == 1:
      #print("full FIFO")
      self._can.recv(0,self._tv[self._r_c])
      self._r_c = self._r_c + 1
      self._r_m = self._r_m + 1
      if self._r_c>=self._circ_buffer_size:
        self._r_c = 0 
      self._can.recv(0,self._tv[self._r_c])
      self._r_c = self._r_c + 1
      self._r_m = self._r_m + 1
      if self._r_c>=self._circ_buffer_size:
        self._r_c = 0 
      self._can.recv(0,self._tv[self._r_c])
      self._r_c = self._r_c + 1
      self._r_m = self._r_m + 1
      if self._r_c>=self._circ_buffer_size:
        self._r_c = 0 
      
    # If reason ==3, it means that we lost a message
    if reason == 2:
      print('overflow')
  
  #Return the opposite buffer that you are actively filling (return the full one)

  def getFullFrame2(self):
    if self._active_buffer == 1:
      return self._dd[1]
    else: 
      return self._dd[0] 

  def handleData2(self):
    d_lst = self._tv[self._p_c]
    try:
      # Listen for the socket switch button press to signify to either start a
      # device based training session or to stop one halfway through
      if d_lst[0] == 0x2C1:
        if d_lst[3][1] == 0:
          if not self._device_based_training:
            self._device_based_training = True
            self._dbt_toggle = True
          else:
            self._device_based_training = False
            self._stop_dbt_toggle = True
      # The rest of these conditional statements are for handling the EMG data
      if d_lst[0] == 0x451:
        self._dd[self._buffer_filling[0]][self._n1_c,0] = d_lst[3][1] << 8 | d_lst[3][2]
        self._dd[self._buffer_filling[0]][self._n1_c,1] = d_lst[3][3] << 8 | d_lst[3][4]
        self._dd[self._buffer_filling[0]][self._n1_c,2] = d_lst[3][5] << 8 | d_lst[3][6]
        self._n1_c = self._n1_c + 1
        if self._n1_c >= self._frame_len:
          self._n1_c = 0
          self._frame_full[0] = 1
          if self._buffer_filling[0] == 0:
            self._buffer_filling[0] =1
          else:
            self._buffer_filling[0] =0
      if d_lst[0] == 0x452:
        self._dd[self._buffer_filling[1]][self._n2_c,3] = d_lst[3][1] << 8 | d_lst[3][2]
        self._dd[self._buffer_filling[1]][self._n2_c,4] = d_lst[3][3] << 8 | d_lst[3][4]
        self._dd[self._buffer_filling[1]][self._n2_c,5] = d_lst[3][5] << 8 | d_lst[3][6]
        self._n2_c = self._n2_c + 1
        if self._n2_c >= self._frame_len:
          self._n2_c = 0
          self._frame_full[1] = 1
          if self._buffer_filling[1] == 0:
            self._buffer_filling[1] =1
          else:
            self._buffer_filling[1] =0
      if d_lst[0] == 0x453:
        self._dd[self._buffer_filling[2]][self._n3_c,6] = d_lst[3][1] << 8 | d_lst[3][2]
        self._dd[self._buffer_filling[2]][self._n3_c,7] = d_lst[3][3] << 8 | d_lst[3][4]
        self._n3_c = self._n3_c + 1
        if self._n3_c >= self._frame_len:
          self._n3_c = 0
          self._frame_full[2] = 1
          if self._buffer_filling[2] == 0:
            self._buffer_filling[2] =1
          else:
            self._buffer_filling[2] =0
      self._p_c = self._p_c + 1
      self._p_m = self._p_m + 1
      if self._p_c >= self._circ_buffer_size:
        self._p_c = 0
      #Check to see if we need to interleave the buffer
      if (self._frame_full[0] + self._frame_full[1] + self._frame_full[2]) == 3:
        self._frame_full = [0,0,0]
        self._new_frame = True
        #Toggle the active buffer
        if self._active_buffer == 1:
          self._active_buffer = 2
        else:
          self._active_buffer = 1
    except Exception as e:
      self._p_c = self._r_c
      self._p_m = self._r_m
      print(e)

  def getCounters(self):
    return ([self._r_m,self._p_m,self._r_c,self._p_c])

  def simulateCan(self,on_off_flag):
    if on_off_flag == True:
      self._tim = pyb.Timer(2, freq=50)      
      self._tim.callback(self.tick)
    else:
      self._tim.deinit()

  def tick(self,timer):
    self._simulated_message1[0] = self._sim_seq_count 
    self._simulated_message1[1] = 0x00 
    self._simulated_message1[2] = self._sim_seq_count 
    self._simulated_message1[3] = 0x00 
    self._simulated_message1[4] = self._sim_seq_count 
    self._simulated_message1[5] = 0x00 
    self._simulated_message1[6] = self._sim_seq_count 
    self._simulated_message2[0] = self._sim_seq_count 
    self._simulated_message2[1] = 0x00 
    self._simulated_message2[2] = self._sim_seq_count 
    self._simulated_message2[3] = 0x00 
    self._simulated_message2[4] = self._sim_seq_count 
    self._simulated_message2[5] = 0x00 
    self._simulated_message2[6] = self._sim_seq_count 
    self._simulated_message3[0] = self._sim_seq_count 
    self._simulated_message3[1] = 0x00 
    self._simulated_message3[2] = self._sim_seq_count 
    self._simulated_message3[3] = 0x00 
    self._simulated_message3[4] = self._sim_seq_count

    self._can.send(self._simulated_message1,0x451)
    self._can.send(self._simulated_message2,0x452)
    self._can.send(self._simulated_message3,0x453)
    self._sim_seq_count = self._sim_seq_count + 1
    if self._sim_seq_count > 255:
      self._sim_seq_count = 0

  # Sending the buzzing noise for starting the device based training sequence
  def sendStartTrain(self):
    print("SENDING START DBT BEEP")
    self._can.send(self._start_train_message,0x1C0)
    pyb.delay(50)
    self._can.send(self._train_pause_message,0x1C0)
    pyb.delay(50)
    self._can.send(self._start_train_message,0x1C0)
    pyb.delay(50)
    self._can.send(self._train_pause_message,0x1C0)
    pyb.delay(50)
    self._can.send(self._start_train_message,0x1C0)
    pyb.delay(50)

  # Sending the buzzing noise for canceling the device based training sequence
  def sendCancelTrain(self):
    print("SENDING CANCEL DBT BEEP")
    self._can.send(self._stop_train_message,0x1C0)
    pyb.delay(50)

  # Sending the buzzing noise for successfully completing the device based
  # training sequence
  def sendTrainSuccess(self):
    print("SENDING FINISH DBT BEEP")
    self._can.send(self._train_success_message_0,0x1C0)
    pyb.delay(50)
    self._can.send(self._train_success_message_1,0x1C0)
    pyb.delay(50)
    self._can.send(self._train_success_message_2,0x1C0)
    pyb.delay(50)
    self._can.send(self._train_success_message_3,0x1C0)
    pyb.delay(50)
    self._can.send(self._train_pause_message,0x1C0)
    pyb.delay(50)

  # Start listening to the incoming CAN messages again
  def sendStart(self):
    # Reset the CAN buffer to avoid buildup of messages
    self._r_c = 0
    self._r_m = 0
    self._p_c = 0
    self._p_m = 0
    #self._can.send(ubinascii.unhexlify(b'0402FE01F4'),0x150)
    self._can.send(ubinascii.unhexlify(b'5001'),0x150)
    self._can_streaming = 0x01

  # Stop listening to the incoming CAN messages
  def sendStop(self):
    self._can.send(ubinascii.unhexlify(b'5101'),0x150)
    self._can_streaming = 0x00
  
  # Update the sampling frequency to a user input frequency
  #
  # Inputs:
  #   freq:                       sampling frequency for the EMG
  #
  # Returns:
  #   N/A
  def updateSampleFreq(self,freq):
    # Convert the integer value for the sample frequency into a big endian byte
    freq_bytes = freq.to_bytes(2,'big')
    self._samp_freq_message[0] = 0x04
    self._samp_freq_message[1] = 0x02
    self._samp_freq_message[2] = 0xFE
    self._samp_freq_message[3] = freq_bytes[0]
    self._samp_freq_message[4] = freq_bytes[1]
    self._can.send(self._samp_freq_message,0x150)

  # Update the gain of a specific EMG channel
  #
  # Inputs:
  #   channel:                    EMG channel
  #   gain:                       EMG gain
  #
  # Returns:
  #   N/A
  def updateGain(self,channel,gain):
    channel_bytes = channel.to_bytes(1,'big')
    gain_bytes = gain.to_bytes(2,'big')
    self._gain_message[0] = 0x04
    self._gain_message[1] = 0x01
    self._gain_message[2] = channel_bytes[0]
    self._gain_message[3] = gain_bytes[0]
    self._gain_message[4] = gain_bytes[1]
    print(self._gain_message)
    self._can.send(self._gain_message,0x150)

if __name__ == '__main__':
  demo()