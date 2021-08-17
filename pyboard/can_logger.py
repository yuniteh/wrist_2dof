###############################################################################
#This class helps to save data from a CAN bus onto a Pyboard.   
#
#
#Authors: Levi J Hargrove and Kevin Brenner
#Date: June 24, 2021
###############################################################################

#import some packages
import pyb, utime, ubinascii, micropython, os
import ulab as np

# code to be run in micropython
def timeit(f, *args, **kwargs):
    func_name = str(f).split(' ')[1]
    def new_func(*args, **kwargs):
        t = utime.ticks_us()
        result = f(*args, **kwargs)
        print('execution time: ', utime.ticks_diff(utime.ticks_us(), t), ' us')
        return result
    return new_func

class CanLogger:
  # Initialization function. Set up variables, register callbacks, etc.
  def __init__(self):
    # Initializing file name variables and flags to determine if you are
    # currently writing data or if a file is open
    self._f_handle = None
    self._writing_data = 0x00
    self._file_open = False
    self._file_name = ""
    
    working_directory = os.getcwd()
    can_logger_folder_exists = False
    # Check to see if the CANLoggerData folder exists, and if it doesn't,
    # create it
    os.chdir('/sd')
    for x in os.listdir():
      if 'CANLoggerData' in x:
        can_logger_folder_exists = True
    if not can_logger_folder_exists:
      os.mkdir('CANLoggerData')
    os.chdir(working_directory)

  # This function will open a specific file within a specific file path
  #
  # Inputs:
  #   f_name:                     the name of the folder of interest
  #   sub_directory:              the sub directory where the file is
  #                               located
  #   folder_time:                the date-time of the folder of interest
  #   sequence_step:              the step of the troubleshooting sequence
  #
  # Returns:
  #   N/A
  def openFile(self,f_name,sub_directory,folder_time,sequence_step):
    # Checking if this a troubleshooting file or a calibration file
    if sub_directory == "Troubleshooting":
      # Depending on the sequence step number, the folder will get a different
      # secondary name
      if sequence_step == "0":
        sub_folder_name = "ARM_SUPPORTED"
      elif sequence_step == "1":
        sub_folder_name = "ARM_DOWN_SIDE"
      elif sequence_step == "2":
        sub_folder_name = "ARM_FRONT"
      else:
        sub_folder_name = "MVCS_SWEEPS_PUSH_PULL"
      # This series of try and excepts is in the event that these folders do
      # exist at all (which is unlikely if the initialization script
      # "initialize_stored_variables" worked properly)
      try:
        os.chdir('/sd/CANLoggerData/'+sub_directory+"/"+folder_time + "/" + sub_folder_name)
      except:
        try:
          os.chdir('/sd/CANLoggerData/'+sub_directory+"/"+folder_time)
          os.mkdir('/sd/CANLoggerData/'+sub_directory+"/"+folder_time + "/" + sub_folder_name)
          os.chdir('/sd/CANLoggerData/'+sub_directory+"/"+folder_time + "/" + sub_folder_name)
        except:
          os.chdir('/sd/CANLoggerData/'+sub_directory)
          os.mkdir('/sd/CANLoggerData/'+sub_directory+"/"+folder_time)
          os.chdir('/sd/CANLoggerData/'+sub_directory+"/"+folder_time)
          os.mkdir('/sd/CANLoggerData/'+sub_directory+"/"+folder_time + "/" + sub_folder_name)
          os.chdir('/sd/CANLoggerData/'+sub_directory+"/"+folder_time + "/" + sub_folder_name)
      print('/sd/CANLoggerData/'+sub_directory+"/"+folder_time + "/" + sub_folder_name)
      print(f_name)
    else:
      try:
        os.chdir('/sd/CANLoggerData/'+sub_directory)
      except:
        os.mkdir('/sd/CANLoggerData/'+sub_directory)
        os.chdir('/sd/CANLoggerData/'+sub_directory)
    # Open the file of interest and set the writing data variable to be true
    self._f_handle = open(f_name, 'wb')
    self._file_open = True
    self._writing_data = 0x01

  # Write the EMG data to the SD card
  #
  # Inputs:
  #   s_data:                     the EMG data to be written
  #
  # Returns:
  #   N/A
  def writeData(self,s_data):
    self._f_handle.write(s_data.flatten().tobytes())
    
  # Check to see if you should be writing data
  #
  # Inputs:
  #   N/A
  #
  # Returns:
  #   N/A
  def checkWriting(self):
    return self._writing_data

  # Close the file entirely and note that you should not be writing data
  #
  # Inputs:
  #   N/A
  #
  # Returns:
  #   N/A
  def closeFile(self):
    if self._f_handle is not None:
      self._f_handle.close()
      self._f_handle = None
      self._file_open = False
      self._writing_data = 0x00
