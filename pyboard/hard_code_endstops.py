import os, saveHelper

# Every time you need to mechanically disassemble the wrist, make sure to find
# the hard endstops of both the rotator and flexor again and then hardcode them
# here. Then run this script once and it will update the stored endstop values
# which get loaded on power up.
sh = saveHelper.SaveHelper()
endstops_file = "endstops.json"
const_max_ccw_pos = [-1681, 2415]
const_max_cw_pos = [2219, 6315]
const_mid_rot_pos = [269, 4365]
const_max_ext_pos = [-81, 4015]
const_max_flex_pos = [1134, 5230]
const_mid_flex_pos = [527, 5623]
endstops_dict = {"max_ccw_pos": const_max_ccw_pos, "max_cw_pos": const_max_cw_pos, "mid_rot_pos": const_mid_rot_pos, "max_ext_pos": const_max_ext_pos, "max_flex_pos": const_max_flex_pos, "mid_flex_pos": const_mid_flex_pos}

os.chdir('/sd')
sh.writeDict(endstops_file,endstops_dict)
