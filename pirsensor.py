from bluepy.btle import UUID, Peripheral, DefaultDelegate, AssignedNumbers
import struct
import math
import time
import json

def _STD_UUID(val):
    return UUID("%08X-0000-1000-8000-00805f9b34fb" % (0xF0000000+val))
    
class SensorBase:
    # Derived classes should set: svcUUID, ctrlUUID, dataUUID
    sensorOn  = struct.pack("B", 0x01)
    sensorOff = struct.pack("B", 0x00)

    def __init__(self, periph):
        self.periph = periph
        self.service = None
        self.ctrl = None
        self.data = None

    def enable(self):
        if self.service is None:
            self.service = self.periph.getServiceByUUID(self.svcUUID)
        if self.ctrl is None:
            self.ctrl = self.service.getCharacteristics(self.ctrlUUID) [0]
        if self.data is None:
            self.data = self.service.getCharacteristics(self.dataUUID) [0]
        if self.sensorOn is not None:
            self.ctrl.write(self.sensorOn,withResponse=True)

    def read(self):
        return self.data.read()

    def disable(self):
        if self.ctrl is not None:
            self.ctrl.write(self.sensorOff)

    # Derived class should implement _formatData()


class PIRSensor(SensorBase):
    svcUUID  = _STD_UUID(0x180D)
    dataUUID = _TI_UUID(0x2A37)
    ctrlUUID = _TI_UUID(0x2A37)
    sensorOn = None

    def __init__(self, periph):
        SensorBase.__init__(self, periph)
        self.ctrlBits = 0

    def read(self):
        return self.data.read()

    # def enable(self, bits):
        # SensorBase.enable(self)
        # self.ctrlBits |= bits
        # self.ctrl.write( struct.pack("<H", self.ctrlBits) )

    # def disable(self, bits):
        # self.ctrlBits &= ~bits
        # self.ctrl.write( struct.pack("<H", self.ctrlBits) )

    def enable(self):
        SensorBase.enable(self)
        self.char_descr = self.service.getDescriptors(forUUID=0x2902)[0]
        self.char_descr.write(struct.pack('<bb', 0x01, 0x00), True)

    def disable(self):
        self.char_descr.write(struct.pack('<bb', 0x00, 0x00), True)


class PIRSensor(Peripheral):

    def __init__(self,addr,version=AUTODETECT):
        Peripheral.__init__(self,addr,iface=1) # Use plug-in BLE dongle
        print("Initialized peripheral. Scanning services.")
        if version==AUTODETECT:
            svcs = self.discoverServices()
            print("Service discovery completed.")

        self.pirsensor = PIRSensor(self)


def main():
    import sys
    import argparse

    parser = argparse.ArgumentParser()
    arg = parser.parse_args(sys.argv[1:])

    hosts = ['FF:A8:6B:DA:67:76'] 
    
    tag  = list(range(len(hosts)))

    for i, host in enumerate(hosts): 
        print('Connecting to ' + host)
        tag[i] = PIRSensor(host)
        tag[i].pirsensor.enable()

        # Some sensors (e.g., temperature, accelerometer) need some time for initialization.
        # Not waiting here after enabling a sensor, the first read value might be empty or incorrect.
        time.sleep(1.0)

    while True:
        for i in range(len(hosts)):
            pir = tag[i].pirsensor.read()
            print("PIR sensor value: {0}".format(pir))
            tag[i].waitForNotifications(1.0)

    for i in range(len(hosts)):
        tag[i].disconnect()
        del tag[i]

if __name__ == "__main__":
    main()
