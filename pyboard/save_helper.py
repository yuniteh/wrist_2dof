###############################################################################
# This class helps to save data to the SD card on the Pyboard D-series.   
#
#
# Authors: Levi J Hargrove and Kevin Brenner
# Date: June 24, 2021
###############################################################################

#import some packages
import pyb, utime, ubinascii, micropython, os, ustruct
import ulab as np
import uasyncio
import ujson
import gc

# code to be run in micropython
def timeit(f, *args, **kwargs):
    func_name = str(f).split(' ')[1]
    def new_func(*args, **kwargs):
        t = utime.ticks_us()
        result = f(*args, **kwargs)
        print('execution time: ', utime.ticks_diff(utime.ticks_us(), t), ' us')
        return result
    return new_func

class SaveHelper:
    # Initialization function. Set up variables, register callbacks, etc.
    def __init__(self):
        self._f_handle = None
        self._writing_data = False

    # This function writes a mean or covariance matrix. The first element to be
    # written is N, the number of samples used to construct the mean of
    # covariance. The max value is 2^16.
    # Next, the shape is written. This is limited to a byte for the rows, and a
    # byte for the column. This means the biggest matrix that can be written is
    # 255*255.Finally data are written, and are assumed to be floats. 
    def writeMN_COV(self,s_data,N,fname):
        f_handle = open(fname, 'wb')
        shap_data = s_data.shape()
        shape_dat = bytearray(2)
        N_a = np.array(([N]),dtype=np.uint16)
        f_handle.write(N_a.flatten().tobytes())
        shape_dat[0] = shap_data[0]
        shape_dat[1] = shap_data[1]
        f_handle.write(shape_dat)
        f_handle.write(s_data.flatten().tobytes())
        f_handle.close()

    # This function reads a matrix that was written by writeMN_COV and returns
    # the value   
    def readMN_COV(self,fname):
        f_handle = open(fname, 'rb') 
        N = ustruct.unpack('H',f_handle.read(2))
        gc.collect()
        tmp_mat = np.zeros((ustruct.unpack('BB',f_handle.read(2))))
        dum = tmp_mat.shape()
        for x in range(dum[0]):
            for y in range(dum[1]):
                tmp = ustruct.unpack('d',f_handle.read(8))
                tmp_mat[x,y] = tmp[0]
        f_handle.close()
        return tmp_mat,N


    # This function is very similar to write_MN_COV, with the difference being
    # that the number of points does not need to be written
    def writeFloatMat(self,s_data,fname):
        f_handle = open(fname, 'wb')
        shap_data = s_data.shape()
        shape_dat = bytearray(2)
        shape_dat[0] = shap_data[0]
        shape_dat[1] = shap_data[1]
        f_handle.write(shape_dat)
        f_handle.write(s_data.flatten().tobytes())
        f_handle.close()

    # This function reads a matrix that was written by writeFloatMat and
    # returns the value
    def readFloatMat(self,fname):
        f_handle = open(fname, 'rb') 
        tmp_mat = np.zeros((ustruct.unpack('BB',f_handle.read(2))))
        dum = tmp_mat.shape()
        for x in range(dum[0]):
            for y in range(dum[1]):
                tmp = ustruct.unpack('d',f_handle.read(8))
                tmp_mat[x,y] = tmp[0]
        f_handle.close()
        return tmp_mat
    
    # This function writes any dicitonary to a specific json file
    def writeDict(self,fname,dict):
        ujson_dict = ujson.dumps(dict)
        f_handle = open(fname,'w')
        f_handle.write(ujson_dict)
        f_handle.close()

    # This function reads in the json file for wrist settings and unpacks them
    # into three separate lists: emg gains, terminal device settings, and wrist
    # settings        
    def readSettings(self,fname):
        with open(fname) as f:
            settings_data = ujson.load(f)
            f.close()
        for key in settings_data:
            if key == "EMG":
                emg_gains = settings_data[key]
            elif key == "TD":
                td_settings = settings_data[key]
            elif key == "WD":
                wrist_settings = settings_data[key]
        for i in range(len(td_settings)):
            if td_settings[i] == 255:
                td_settings[-1]
        final_dict = {"EMG": emg_gains, "TD": td_settings, "WD": wrist_settings}
        return emg_gains, td_settings, wrist_settings, final_dict

    # This function will read in the hardcoded endstops from the respective
    # json file
    def readEndstops(self,fname):
        with open(fname) as f:
            endstops_data = ujson.load(f)
            f.close()
        for key in endstops_data:
            if key == "max_ccw_pos":
                max_ccw_pos = endstops_data[key]
            elif key == "max_cw_pos":
                max_cw_pos = endstops_data[key]
            elif key == "mid_rot_pos":
                mid_rot_pos = endstops_data[key]
            elif key == "max_ext_pos":
                max_ext_pos = endstops_data[key]
            elif key == "max_flex_pos":
                max_flex_pos = endstops_data[key]
            elif key == "mid_flex_pos":
                mid_flex_pos = endstops_data[key]
        final_dict = {"max_ccw_pos": max_ccw_pos, "max_cw_pos": max_cw_pos, "mid_rot_pos": mid_rot_pos, "max_ext_pos": max_ext_pos, "max_flex_pos": max_flex_pos, "mid_flex_pos": mid_flex_pos}
        return max_ccw_pos,max_cw_pos,mid_rot_pos,max_ext_pos,max_flex_pos,mid_flex_pos,final_dict

    # This function will read in the number of calibrations performed with each
    # terminal device from the respective json file
    def readCalCount(self,fname):
        with open(fname) as f:
            cal_counter_data = ujson.load(f)
            f.close()
            print("cal_counter_data: ",cal_counter_data)
        for key in cal_counter_data:
            if key == "cal_count_ottobock":
                cal_count_ottobock = cal_counter_data[key]
            elif key == "cal_count_psyonic":
                cal_count_psyonic = cal_counter_data[key]
        final_dict = {"cal_count_ottobock":cal_count_ottobock,"cal_count_psyonic":cal_count_psyonic}
        return cal_count_ottobock,cal_count_psyonic,final_dict
     
# def demo():
#     os.chdir('/sd/ControllerData')
#     wg = np.zeros((32,10), dtype = np.float)
#     cg = np.zeros((1,10), dtype = np.float)
#     f_name = 'WG_DATA.DAP'
#     sh = SaveHelper()
#     sh.writeFloatMat(wg,f_name)
#     f_name = 'CG_DATA.DAP'
#     sh.writeFloatMat(cg,f_name)
#     os.chdir('/sd')

if __name__ == '__main__':
    demo()
