###############################################################################
# This script handles BLE communications between the Pyboard and a mobile phone
# application made in Unity 3D
#
#
#
# Author: Levi Hargrove
# Date: Jan 20, 2020
###############################################################################

import ubluetooth, utime,ubinascii, pyb, micropython, machine
from ble_advertising import advertising_payload

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)
_IRQ_GATTS_READ_REQUEST = const(4)
_IRQ_SCAN_RESULT = const(5)
_IRQ_SCAN_DONE = const(6)
_IRQ_PERIPHERAL_CONNECT = const(7)
_IRQ_PERIPHERAL_DISCONNECT = const(8)
_IRQ_GATTC_SERVICE_RESULT = const(9)
_IRQ_GATTC_SERVICE_DONE = const(10)
_IRQ_GATTC_CHARACTERISTIC_RESULT = const(11)
_IRQ_GATTC_CHARACTERISTIC_DONE = const(12)
_IRQ_GATTC_DESCRIPTOR_RESULT = const(13)
_IRQ_GATTC_DESCRIPTOR_DONE = const(14)
_IRQ_GATTC_READ_RESULT = const(15)
_IRQ_GATTC_READ_DONE = const(16)
_IRQ_GATTC_WRITE_DONE = const(17)
_IRQ_GATTC_NOTIFY = const(18)
_IRQ_GATTC_INDICATE = const(19)
_IRQ_GATTS_INDICATE_DONE = const(20)


# code to be run in micropython
def timeit(f, *args, **kwargs):
    func_name = str(f).split(' ')[1]
    def new_func(*args, **kwargs):
        t = utime.ticks_us()
        result = f(*args, **kwargs)
        print('execution time: ', utime.ticks_diff(utime.ticks_us(), t), ' us')
        return result
    return new_func

_serviceUUID = ubluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_uartReadCharacteristicUUID  = ubluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
_uartWriteCharacteristicUUID = ubluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")
_uartMessCharacteristicUUID = ubluetooth.UUID("6E400004-B5A3-F393-E0A9-E50E24DCCA9E")

_READ_CHAR = (_uartReadCharacteristicUUID, ubluetooth.FLAG_READ|ubluetooth.FLAG_NOTIFY,)
_WRITE_CHAR = (_uartWriteCharacteristicUUID, ubluetooth.FLAG_WRITE|ubluetooth.FLAG_NOTIFY,)
_MESS_CHAR = (_uartMessCharacteristicUUID, ubluetooth.FLAG_READ | ubluetooth.FLAG_NOTIFY, )
_CONTROLLER_SERVICE = (_serviceUUID, (_READ_CHAR,_WRITE_CHAR,_MESS_CHAR,),)


class BLEComms:
  # Initialization function. Set up variables, register callbacks, etc.
    def __init__(self, ble, name='PYB_2DOF_Wrist'):
        self._ble = ble
        self._conn_handle = None
        self._ble.active(True)
        #self._ble.config(rxbuf=2048)
        self._ble.config(gap_name=name)
        self._ble.irq(self._irq)
        ((self._txhandle,self._rxhandle,self._mxhandle),) = self._ble.gatts_register_services((_CONTROLLER_SERVICE,))
        self._connections = set()
        self._payload = advertising_payload(name=name, services=[_serviceUUID])
        self._name = name
        self._adv()
        self._send_saved_folders = True
        self._connected = False
        self._new_message = False
        self._message = bytearray(32)
        self._indicating = False
        self._send_controller_status = False
        self._exceptions = 0
        self._last_heartbeat = -1
        self._send_classifier_available_status = True
        self._first_time_connected = True

    def adv_encode(self,adv_type,value):
        tmp =  bytes((len(value) + 1, adv_type,)) + value
        return tmp

    def adv_encode_name(self,name):
        tmp = self.adv_encode(0x09, name.encode())
        return tmp

    def checkMessageFlag(self):
        return self._new_message

    def getMessage(self):
        self._new_message = False
        return self._message

    def sendMessage(self,mess):
        try:
            self._ble.gatts_notify(64, self._txhandle, mess)
            self._exceptions = 0
        except Exception as e:
            print("Got exception" + str(self._exceptions) + str(e))
            self._exceptions = self._exceptions + 1
            micropython.mem_info()
            if self._exceptions > 50:
                self.disconnectBLE()

    def writeMessage(self,mess):
        #print("writing BLE message")
        
        try:
            #self._ble.gatts_write(self._mxhandle, mess)
            self._ble.gatts_notify(64, self._mxhandle,mess)
            self._exceptions = 0
        except Exception as e:
            print("Got exception" + str(self._exceptions) + str(e))
            self._exceptions = self._exceptions + 1
            micropython.mem_info()
            if self._exceptions > 50:
                self.disconnectBLE()
                
    def setSendControllerStatus(self,status_val):
        self._send_controller_status = status_val


    def checkSendControllerStatus(self):
        return self._send_controller_status

    def checkIndicatingFlag(self):
        return self._indicating

    def disconnectBLE(self):
        for conn_handle in self._connections:
            try:
                self._ble.gap_disconnect(conn_handle)
            except Exception as e:
                print("Disconnect exception:  " + str(e))

        self._send_controller_status = False

        utime.sleep_ms(1)
        self._connections.clear()
        self._connected = False
        self._connected_counter = 0
        self._last_heartbeat = -1
        self._adv()
        

    def checkConnectionStatus(self):
        return self._connected

    def _irq(self, event, data):
    # Track connections so we can send notifications.
        if event == _IRQ_CENTRAL_CONNECT:
            self._conn_handle, _, _, = data
            self._connections.add(self._conn_handle)
            self._connected = True
            self._send_controller_status = True
            self._send_classifier_available_status = True
            print("Central Connect!")
        elif event == _IRQ_CENTRAL_DISCONNECT:
            self._conn_handle, _, _, = data

            #self._connections.remove(self._conn_handle)
            for conn_handle in self._connections:
                self._ble.gap_disconnect(conn_handle)
            self._connections.clear()
            self._connected = False
            self._connected_counter = 0
            self._last_heartbeat = -1
            self._send_controller_status = False
            self._send_saved_folders = True
            print("Central Disconnect!")
            self._first_time_connected = True
            
            # Start advertising again to allow a new connection.
            self._adv()
        elif event == _IRQ_PERIPHERAL_DISCONNECT:
            print("PERIPHERAL Disconnected")

        elif event == _IRQ_GATTS_WRITE:
            self.conn_handle, value_handle, = data
            if self.conn_handle in self._connections:
                if self._first_time_connected:
                    self._new_message = False
                    self._message = bytearray(32)
                    leftover_message = self._ble.gatts_read(self._rxhandle)
                    print("leftover_message: ",leftover_message)
                else:
                    self._new_message = True
                    self._message = self._ble.gatts_read(self._rxhandle)

        elif event == _IRQ_GATTS_INDICATE_DONE:
                # A central has acknowledged the indication.
                # Note: Status will be zero on successful acknowledgment, implementation-specific value otherwise.
                conn_handle, value_handle, status = data

    def _adv(self, interval_us=500000):
        try:
            self._ble.gap_advertise(100, self.adv_encode(0x01, b'\x06') + self.adv_encode(0x03, b'\x0d\x18') + self.adv_encode(0x19, b'\xc1\x03') + self.adv_encode_name(self._name))
        except Exception as e:
            print("Advertising excpetion:   " + str(e))

    def getHeartbeatTime(self):
        return self._last_heartbeat

    def updateHeartbeatTime(self):
        self._last_heartbeat = utime.time()

# def parseMessage(mess):
#     if mess[0] == 0x01:
#         self._last_heartbeat
#     elif mess[0] == 0x10:
#         print("Log Start")
#         save_name = mess[1:].decode() + '_' + str(utime.localtime()[0]) + '_' + str(utime.localtime()[1]) + '_' + str(utime.localtime()[2]) + '_' +  str(utime.localtime()[3]) + '_' + str(utime.localtime()[4]) + '_' + str(utime.localtime()[5]) + '.DAP' 
#         print(save_name)
#     elif mess[0] == 0x11:
#         print("Log Stop")
#     elif mess[0] == 0x12:
#         print("Can Start Command")
#     elif mess[0] == 0x13:
#         print("Can Stop Command") 

# def demo():
#     ble = ubluetooth.BLE()
#     b = BLEComms(ble)
#     while True:
#         mess_avail = b.checkMessageFlag()
#         if mess_avail:
#             parseMessage(b.getMessage())
#         utime.sleep_ms(500)

if __name__ == '__main__':
    demo()