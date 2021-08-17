###############################################################################
# This class helps initialize the contents of the SD card on the Pyboard
# D-series   
#
#
# Author: Kevin Brenner
# Date: June 24, 2021
############################################################################
import os, config, pattern_rec, save_helper
import ulab as np

class InitVariables:
    def __init__(self):
        self.sh = save_helper.SaveHelper()
        self.config_inst = config.Config()
    def initialize(self):
        os.chdir('/sd')
        # Create CAN Logger and Controller data folders
        os.mkdir('CANLoggerData')
        os.mkdir('ControllerData')
        # Create default endstops json file
        endstops_dict = {"max_ccw_pos": [0,0], "max_cw_pos": [0,0], "mid_rot_pos": [0,0], "max_ext_pos": [0,0], "max_flex_pos": [0,0], "mid_flex_pos": [0,0]}
        endstops_file = "endstops.json"
        self.sh.writeDict(endstops_file,endstops_dict)
        # Create default settings json file
        settings_dict = {"EMG": [2, 2, 2, 2, 2, 2, 2, 2], "TD": [2, 85, 0, 5, 255, 255, 255, 50, 50, 255, 255, 50], "WD": [1, 1, 1, 50, 100, 100, 0, 0, 1, 1, 1, 50, 100, 100, 0, 0]}
        settings_file = "settings.json"
        self.sh.writeDict(settings_file,settings_dict)
        # Create calibration counter json file
        cal_count_dict = {"cal_count_ottobock":0,"cal_count_psyonic":0}
        cal_count_file = 'cal_count.json'
        self.sh.writeDict(cal_count_file,cal_count_dict)
        # Create error log text file
        error_mess = ""
        error_file = 'error_log.txt'
        error_log_file = open(error_file,'w')
        error_log_file.write(error_mess)
        error_log_file.close()
        # Create terminal device type text file
        td_type = "Ottobock"
        td_file = 'td_type.txt'
        td_type_file = open(td_file,'w')
        td_type_file.write(td_type)
        td_type_file.close()

        os.chdir('/sd/ControllerData')
        empty_cov = np.zeros((self.config_inst.total_num_feats,self.config_inst.total_num_feats))
        empty_mean = np.zeros((1,self.config_inst.total_num_feats))
        # Loop through the number of possible classes and create empty entries
        # for all of the classifier variables
        for x in range(self.config_inst.num_classes):
          self.sh.writeMN_COV(empty_cov,0,('COV'+str(x)+'.DAP'))
          self.sh.writeMN_COV(empty_mean,0,('MN'+str(x)+'.DAP'))
        self.sh.writeMN_COV(np.zeros((self.config_inst.total_num_feats,self.config_inst.total_num_feats)),0,('INV_COV.DAP'))
        self.sh.writeFloatMat(np.zeros((self.config_inst.total_num_feats,self.config_inst.total_num_feats), dtype = np.float),'WG.DAP')
        self.sh.writeFloatMat(np.zeros((1,self.config_inst.total_num_feats), dtype = np.float),'CG.DAP')
        self.sh.writeFloatMat(np.zeros((1,self.config_inst.total_num_feats), dtype = np.float),'Class_List.DAP')
        self.sh.writeFloatMat(np.array([[0.0,0.0]]),'NM_Thresh.txt')
        self.sh.writeFloatMat(np.array([[-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]]),'Grip_Data.DAP')

        # Make the necessary subfolders for both the controller data and the
        # CAN logger data
        os.mkdir('ControllerDataSubFolder')
        os.chdir('ControllerDataSubFolder')
        os.mkdir('Ottobock')
        os.mkdir('Psyonic')
        os.chdir('/sd/CANLoggerData')
        os.mkdir('Calibrations')
        os.mkdir('Troubleshooting')
        os.chdir('/sd')
