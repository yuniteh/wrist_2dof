__author__ = 'lhargrove and kbrenner'
import ubinascii
POLYNOMIAL = 0x8005
PRESET = 0
OFFSET = 1 << 32
MASK = OFFSET - 1
OFFSET = 1 << 32
MASK = OFFSET - 1

def _initial(c):
    crc = 0
    c = c << 8
    for j in range(8):
        if (crc ^ c) & 0x8000:
            crc = (crc << 1) ^ POLYNOMIAL
        else:
            crc = crc << 1
        c = c << 1
    return crc

_tab = [ _initial(i) for i in range(256) ]

def _update_crc(crc, c):
    cc = 0xff & c

    tmp = (crc >> 8) ^ cc
    crc = (crc << 8) ^ _tab[tmp & 0xff]
    crc = crc & 0xffff
    return crc

def crc(str):
    crc = PRESET
    for c in str:
        crc = _update_crc(crc, c)
    return crc

def crc_vel(str):
    crc = 34957
    for c in str:
        crc = _update_crc(crc, c)
    return crc

def crcb(*i):
    crc = PRESET
    for c in i:
        crc = _update_crc(crc, c)
    return crc

def createCommandString(crcStr):
    unhexS = ubinascii.unhexlify(crcStr)
    crcOut = hex(crc(unhexS))
    if len(crcOut) == 5:
        crcH = '0' + crcOut[2]
        crcL = crcOut[3:5]
    elif len(crcOut) == 4:
        crcH = '00'
        crcL = crcOut[2:4]
    elif len(crcOut) == 3:
        crcH = '00'
        crcL = '0' + crcOut[2]
    else:
        crcL = crcOut[-2:]
        crcH = crcOut[2:4]
    crc_string = crcL + crcH
    
    finalString = crcStr + crcL + crcH
    unhexS = ubinascii.unhexlify(finalString)
    return crc_string,unhexS
    
def createCommandVel(command_v,crcStr):
    unhexS = ubinascii.unhexlify(crcStr)
    tmp = crc_vel(unhexS)
    crcOut = hex(tmp)
    if len(crcOut) == 5:
        crcH = '0' + crcOut[2]
        crcL = crcOut[3:5]
    elif len(crcOut) == 4:
        crcH = '00'
        crcL = crcOut[2:4]
    elif len(crcOut) == 3:
        crcH = '00'
        crcL = '0' + crcOut[2]
    else:
        crcL = crcOut[-2:]
        crcH = crcOut[2:4]
    crc_string = crcL + crcH
    finalString = command_v + crcStr + crcL + crcH
    #print("final_string: " + str(finalString))
    unhexS = ubinascii.unhexlify(finalString)
    return crc_string, unhexS
    
# Converts integer values into hex values
def int2Hex(num):
    hex = '%08x' % (num + OFFSET & MASK)
    bytes = []
    for i in range(0, 4):
        bytes.append(hex[i * 2: i * 2 + 2])
    tmpV = bytes[::-1]
    outVal = tmpV[0] + tmpV[1] + tmpV[2] + tmpV[3]
    return outVal

def int2HexSize(num,size):
    hex = '%08x' % (num + OFFSET & MASK)
    bytes = []
    for i in range(0, 4):
        bytes.append(hex[i * 2: i * 2 + 2])
    tmpV = bytes[::-1]
    if size == 1:
        outVal = tmpV[0]
    elif size == 2:
        outVal = tmpv[0] + tmpV[1]
    elif size == 4:
        outVal = tmpv[0] + tmpV[1] + tmpV[2] + tmpV[3]
    return outVal

# Converts hex values into integer values
def signedPos(hexPos):
    # This is for 4 byte length parameters (ex: read position or velocity)
    if len(hexPos) == 30:
        pos2Com = hexPos[24:26] + hexPos[22:24] + hexPos[20:22]+ hexPos[18:20]
        intPos = int(pos2Com,16)
        if intPos > 0x7fffffff:
            intPos -= 0x100000000
    # This is for 2 byte length parameters (ex: read current)
    elif len(hexPos) == 26:
        pos2Com = hexPos[20:22]+hexPos[18:20]
        intPos = int(pos2Com,16)
        if intPos > 0x7fff:
            intPos -= 0xffff
    # This is for 1 byte length parameters (ex: read operation mode)
    elif len(hexPos) == 24:
        pos2Com = hexPos[18:20]
        intPos = int(pos2Com,16)
    # This is so we don't return a variable that hasn't be declared
    else:
        intPos = 0
    return intPos
    
def signedIndPos(hexPos):
    # This is for the parameters of (position, current, bus watchdog, and hardware error status)
    if len(hexPos) == 38:
        pos2Com = hexPos[24:26] + hexPos[22:24] + hexPos[20:22]+ hexPos[18:20]
        intPos = int(pos2Com,16)
        if intPos > 0x7fffffff:
            intPos -= 0x100000000
        vel2Com = hexPos[28:30] + hexPos[26:28]
        intCurr = int(vel2Com,16)
        if intCurr > 0x7fff:
            intCurr -= 0xffff
        watch2Com = hexPos[30:32]
        intWatch = int(watch2Com,16)
        error2Com = hexPos[32:34]
        intError = int(error2Com,16)
    else:
        intPos = 0
        intCurr = 0
        intWatch = 0
        intError = 0
    return [intPos,intCurr,intWatch,intError]