import bluepy
import threading
import struct
import argparse
import time
import os
from bluepy.btle import UUID, Peripheral, DefaultDelegate, AssignedNumbers
import statistics
import scipy
from scipy.spatial import distance
import pickle

devicenames = {}
DEVICENAMESFILE = 'devices.txt'
MAXDEVICES = 10
AUTODETECT = "-"
BTNAME='Complete Local Name'
HCI=1

def is_float(text):
    try:
        float(text)
        # check for nan/infinity etc.
        if str(text).isalpha():
            return False
        return True
    except ValueError:
        return False
    except TypeError:
        return False

class _paireddevice():
    def __init__( self, dev, devdata ):
        if args.info:
            print ("devdata=",devdata)
        self.devdata = devdata
        self.name = self.devdata[ BTNAME ]
        if dev.addr not in devicenames:
            # the devices are named in order of discovery
            # remember automatically-assigned names so if the device goes away
            # and comes back in the same run it gets the same name
            devicenames[dev.addr] = args.name + chr(48+len(devicenames)+1)
        self.friendlyname = devicenames[dev.addr]
        self.addr = dev.addr
        self.addrType = dev.addrType
        self.rssi = dev.rssi
        self.report("status","found")
        if args.info:
            print ("created _paireddevice")
    # pair
    def pair(self):
        print ("pairing")
        pass
    def unpair(self):
        if args.info:
            print ("unpairing", self)
        self.running = False
        for thread in self.threads:
            thread.join()
        pass
    def _sensorlookup(self, sensorname):
        if not hasattr(self.tag, sensorname):
            if args.info:
                print ("not found",sensorname)
            return None
        return getattr(self.tag,sensorname)

    # do whatever it takes to kick off threads to read the sensors in f,m,s
    # at read rates FAST, MEDIUM, SLOW
    def start(self, f,m,s):
        """
        f,m,s are list of sensor names to run at FAST, MEDIUM and SLOW read rates
        """
        if args.info:
            print ("starting",f,m,s)
        self.running = True
        self.threads = []
        for sensors,interval in zip([f,m,s],[FAST,MEDIUM,SLOW]):
            if sensors:
                self.threads.append(threading.Thread(target=self.runner,args=(sensors,interval)))
                print ("setting daemon")
                self.threads[-1].daemon = True
                self.threads[-1].start()
    def runinit(self, sensors):
        if args.info:
            print ("initializing for run", sensors)
        return False
    def runread(self,sensors):
        if args.info:
            print('Doing something important in the background', self, sensors)
        return False
    def runner(self,sensors,interval):
        """ Method that runs forever """
        if not self.runinit(sensors):
            return
        while self.running:
            # Do something
            if not self.runread(sensors):
                break
#           print ("pausing for",interval)
            time.sleep(interval)
        if args.info:
            print ("Aborting")
    def report(self,tag,value=None):
        print ("report",self.addr, self.friendlyname, tag, value)
        if is_float(value):
            print ('{"deviceuid":"'+self.addr+'","devicename":"'+self.friendlyname+'","'+tag+'":'+str(value)+'}')
        else:
            if not isinstance(value, str):
                # a lit of numbers
                print ("["+",".join([str(x) for x in value])+"]")
                print ('{"deviceuid":"'+self.addr+'","devicename":"'+self.friendlyname+'","'+tag+'":'+"["+",".join([str(x) for x in value])+"]"+'}')
            else:
                # a simple string
                print ('{"deviceuid":"'+self.addr+'","devicename":"'+self.friendlyname+'","'+tag+'":"'+str(value)+'"}')
        sys.stdout.flush()

class SensorTag(Peripheral):     # Special type of BLE_Device

    # A toggle controlled by the button on the sensortag for calibration
    def __init__(self,addr,version=AUTODETECT):
        Peripheral.__init__(self,addr,iface=1) # Use plug-in BLE dongle
        print("Initialized peripheral. Scanning services.")
        if version==AUTODETECT:
            svcs = self.discoverServices()
            print("Service discovery completed.")
        self.mpu9250 = MovementSensorMPU9250(self)
        self.accelerometer = AccelerometerSensorMPU9250(self.mpu9250)
        self.humidity = HumiditySensorHDC1000(self)
        self.magnetometer = MagnetometerSensorMPU9250(self.mpu9250)
        self.barometer = BarometerSensorBMP280(self)
        self.gyroscope = GyroscopeSensorMPU9250(self.mpu9250)
        self.keypress = KeypressSensor(self)
        self.lightmeter = OpticalSensorOPT3001(self)
        self.battery = BatterySensor(self)


class KeypressDelegate(DefaultDelegate):
    BUTTON_L = 0x02
    BUTTON_R = 0x01
    ALL_BUTTONS = (BUTTON_L | BUTTON_R)

    _button_desc = {
        BUTTON_L : "Left button",
        BUTTON_R : "Right button",
        ALL_BUTTONS : "Both buttons"
    }

    def __init__(self):
        DefaultDelegate.__init__(self)
        self.lastVal = 0

    def handleNotification(self, hnd, data):
        # NB: only one source of notifications at present
        # so we can ignore 'hnd'.
        val = struct.unpack("B", data)[0]
        down = (val & ~self.lastVal) & self.ALL_BUTTONS
        if down != 0:
            self.onButtonDown(down)
        up = (~val & self.lastVal) & self.ALL_BUTTONS
        if up != 0:
            self.onButtonUp(up)
        self.lastVal = val

    def onButtonUp(self, but):
        print ( "** " + self._button_desc[but] + " UP")

    def onButtonDown(self, but):
        print ( "** " + self._button_desc[but] + " DOWN")

# this is a generic sensortag
class _SensorTag(_paireddevice):
    def __init__( self, dev, devdata):
        if args.info:
            print ("creating _SensorTag")
        self.tag = bluepy.sensortag.SensorTag(dev.addr)
        _paireddevice.__init__( self, dev, devdata )
        self.devicetype = "SensorTag generic"
        if args.info:
            print ("created _SensorTag")
        return True
    def runinit(self, sensors):
        if args.info:
            print ("_Sensortag runinit", sensors)
        self.report("status","enabled "+repr(sensors))
        for sensor in sensors:
            if args.info:
                print ("enabling",sensor)
            tagfn = self._sensorlookup(sensor)
            if tagfn:
                tagfn.enable()
#        time.sleep( 1.0 )
        return True
    def runread(self, sensors):
        try:
            for sensor in sensors:
                tagfn = self._sensorlookup(sensor)
                if tagfn:
                    self.report(sensor, tagfn.read())
        except bluepy.btle.BTLEException:
            self.report("status","lost")
            return False
        return True


class SensorBase:
    def __init__(self, _arg):
        self.periph = _arg
        self.service = None
        self.ctrl = None
        self.data = None
        self.sensorOn = None

    def enable(self,bits):
        if self.service is None:
            self.service = self.periph.getServiceByUUID(self.svcUUID)
        if self.ctrl is None:
            self.ctrl = self.service.getCharacteristics(self.ctrlUUID) [0]
        if self.data is None:
            self.data = self.service.getCharacteristics(self.dataUUID) [0]
        if self.sensorOn is None:
            self.ctrl.write(bits,withResponse=True)

    def read(self):
        return self.data.read()

    def disable(self,bits):
        if self.ctrl is not None:
            self.ctrl.write(bits)


# depending on the bluetooth device type, this
# creates an instance of the appropriate class
def paireddevicefactory( dev ):
    # get the device name to decide which type of device to create
    devdata = {}
    for (adtype, desc, value) in dev.getScanData():
        devdata[desc]=value
    if BTNAME not in devdata.keys():
        devdata[BTNAME] = 'Unknown!'
        return None
    else:
        #return BLE_Device( dev, devdata )
        return BLE_Device(dev)

# this scandelegate handles discovery of new devices
class ScanDelegate(bluepy.btle.DefaultDelegate):
    def __init__(self):
        bluepy.btle.DefaultDelegate.__init__(self)
        self.activedevlist = []
#        self.gw = gateway
    def handleDiscovery(self, dev, isNewDev, isNewData):
        global args
        if isNewDev:
            if args.info:
                print ("found", dev.addr)
            if args.only:
                if args.info:
                    print ("only", dev.addr, devicenames)
            if dev.addr not in devicenames:
                # ignore it!
                if args.info:
                    print ("ignoring only",dev.addr)
                return
            if len(self.activedevlist)<MAXDEVICES:
                thisdev = paireddevicefactory(dev)
                if args.info:
                    print ("thisdev=",thisdev)
                if thisdev:
                    self.activedevlist.append(thisdev)
                    thisdev.pair()
                    thisdev.start(args.fast,args.medium,args.slow)
                if args.info:
                    print ("activedevlist=",self.activedevlist)
            else:
                if args.info:
                    print ("TOO MANY DEVICES - IGNORED",dev.addr)
            # launch a thread which pairs with this device and reads temperatures
        elif isNewData:
            if args.info:
                print ("Received new data from", dev.addr)
            pass

    def shutdown( self ):
        if args.info:
            print ("My activedevlist=",self.activedevlist)
        # unpair the paired devices
        for dev in self.activedevlist:
            if args.info:
                print ("dev=",dev)
            dev.unpair()

class BLE_Device(Peripheral):
    def __init__(self,addr,version=AUTODETECT):
        Peripheral.__init__(self,addr,iface=1) # Use plug-in BLE dongle
        self.deviceType = None
        self.mac = None
        self.position = None
        self.sensors = []
        print("Initialized peripheral. Scanning services.")
        if version==AUTODETECT:
            svcs = self.discoverServices()
            print("Service discovery completed.")

    def setMAC(self,m):
        self.mac = m

    def setPosition(self,p):
        self.position = p

    def setDeviceType(self, t):
        self.deviceType = t

class Sensor(SensorBase):
    def __init__(self, _val):
        if isinstance(_val, Peripheral):
            super().__init__(_val)
            self.periph = _val
            self.sensorType = None
            self.uuid = None
            self.handle = None
            self.svcUUID = None
            self.dataUUID = None
            self.ctrlUUID = None
            self.ctrlBits = 0
        elif isinstance(_val, Sensor):
            super().__init__(_val.periph)
            self.data = _val.data
            self.service = _val.service
            self.ctrl = _val.ctrl
            self.sensorOn = _val.sensorOn
            self.sensorType = _val.sensorType
            self.uuid = _val.uuid
            self.handle = _val.handle
            self.svcUUID = _val.svcUUID
            self.dataUUID = _val.dataUUID
            self.ctrlUUID = _val.ctrlUUID
            self.ctrlBits = _val.ctrlBits

    def setMAC(self,m):
        self.mac = m

    def setServiceUUID(self,u):
        self.svcUUID = u

    def setControlUUID(self,u):
        self.ctrlUUID = u

    def setDataUUID(self,u):
        self.dataUUID = u

    def setHandle(self,h):
        self.handle = h

    def setPosition(self,p):
        self.position = p

    def setDeviceType(self, t):
        self.deviceType = t

    def setSensorType(self, t):
        self.sensorType = t

    def enable(self):
        if (self.sensorType=='IMU'):
            bits = struct.pack("<H", 0x02ff)
        else:
            bits = struct.pack("B", 0x01)
        super(Sensor,self).enable(bits)

    def disable(self):
        if (self.sensorTyp=='IMU'):
            bits = struct.pack("<H",0x0000)
        else:
            bits = ~struct.pack("B", 0x00)
        super(Sensor,self).disable(bits)

    def read(self):
        return None # Should be handled by inherited classes
        # if (self.sensorType=='Humidity'):
            # (rawT, rawH) = struct.unpack('<HH', self.data.read())
            # temp = -46.85 + 175.72 * (rawT / 65536.0)
            # RH = -6.0 + 125.0 * ((rawH & 0xFFFC)/65536.0)
            # return (temp, RH)
        # elif (self.sensorType=='Barometer'):
            # (tL,tM,tH,pL,pM,pH) = struct.unpack('<BBBBBB', self.data.read())
            # temp = (tH*65536 + tM*256 + tL) / 100.0
            # press = (pH*65536 + pM*256 + pL) / 100.0
            # return (temp, press)
        # elif (self.sensorType=='IMU'):
            # # GYRO
            # rawVals = self.rawRead()[0:3]
            # gyr = tuple([ v*500.0/65536.0 for v in rawVals ])
            # # MAGN
            # rawVals = self.rawRead()[6:9]
            # mag = tuple([ v*4912.0/32760.0 for v in rawVals ])
            # # ACCEL
            # rawVals = self.rawRead()[3:6]
            # acc = tuple([ v*8.0/32768.0 for v in rawVals ])
            # return ((gyr,mag,acc))
        # elif (self.sensorType=='PIR'):
            # rawVals = self.data.read()
            # if int.from_bytes(rawVals,"big") == 90:
                # return 'On'
            # else:
                # return 'Off'
        # else:
            # return("Cannot read this type of sensor.")

    def rawRead(self):
        dval = self.data.read()
        return struct.unpack("<hhhhhhhhh", dval)


class AccelerometerSensor(Sensor):

    def __init__(self, _sensor):
        super().__init__(_sensor)
        self.scale = 64.0

    def read(self):
        '''Returns (x_accel, y_accel, z_accel) in units of g'''
        x_y_z = struct.unpack('bbb', self.data.read())

    def read(self):
        '''Returns (x_accel, y_accel, z_accel) in units of g'''
        x_y_z = struct.unpack('bbb', self.data.read())
        return tuple([ (val/self.scale) for val in x_y_z ])


class MovementSensorMPU9250(Sensor):

    sensorOn = None
    GYRO_XYZ =  7
    ACCEL_XYZ = 7 << 3
    MAG_XYZ = 1 << 6
    ACCEL_RANGE_2G  = 0 << 8
    ACCEL_RANGE_4G  = 1 << 8
    ACCEL_RANGE_8G  = 2 << 8
    ACCEL_RANGE_16G = 3 << 8
    positions=['Closed','Tight','Half Open','Wide Open']
    cp = None # Center positions
    fileName = None

    def __init__(self, _sensor):
        super().__init__(_sensor)
        self.ctrlBits = 0

    def enable(self, bits):
        SensorBase.enable(self)
        self.ctrlBits |= bits
        self.ctrl.write( struct.pack("<H", self.ctrlBits) )

    def disable(self, bits):
        self.ctrlBits &= ~bits
        self.ctrl.write( struct.pack("<H", self.ctrlBits) )

    def rawRead(self):
        dval = self.data.read()
        return struct.unpack("<hhhhhhhhh", dval)

    def readGYR(self):
        rawVals = self.rawRead()[0:3]
        gyr = tuple([ v*500.0/65536.0 for v in rawVals ])
        return gyr

    def readMAG(self):
        rawVals = self.rawRead()[6:9]
        mag = tuple([ v*4912.0/32760.0 for v in rawVals ])
        return mag

    def readACC(self):
        rawVals = self.rawRead()[3:6]
        acc = tuple([ v*8.0/32768.0 for v in rawVals ])
        return acc

    def readALL(self):
        gyr = self.readGYR()
        mag = self.readMAG()
        acc = self.readACC()
        return (mag+acc+gyr)

    def calibrate(self):
        self.cp = list(range(len(self.positions)))
        self.fileName = "cal_imu_{0}.pkl".format(self.periph.addr.replace(':','').lower())
        if (os.path.isfile(self.fileName)):
            with open(self.fileName, 'rb') as f:
                self.cp = pickle.load(f)
        else:
            pc = list(range(9))
            for k, pos in enumerate(self.positions):
                input("Please put in position {0}".format(pos))
                p = list()
                for i in range(20):  #  Number of calibrations per position
                    # tag = self.periph
                    p.append(self.readALL())
                    # p.append(tag.magnetometer.read()+tag.accelerometer.read()+tag.gyroscope.read())
                for j in range(9):
                    x = list()
                    for i in range(len(p)):
                        x.append(p[i][j])
                    pc[j] = statistics.mean(x)
                self.cp[k] = [pc[0],pc[1],pc[2],pc[3],pc[4],pc[5],pc[6],pc[7],pc[8]]
            with open(self.fileName, 'wb') as f:
                pickle.dump(self.cp, f)

    def read(self):
        for k in range(10):
            tag = self.periph
            p = self.readALL()
            # p = [tag.magnetometer.read()[0], tag.magnetometer.read()[1], tag.magnetometer.read()[2], tag.accelerometer.read()[0], tag.accelerometer.read()[1], tag.accelerometer.read()[2], tag.gyroscope.read()[0], tag.gyroscope.read()[1], tag.gyroscope.read()[2]]
            imin = 0
            dmin = 9999999999999999
            for i, pos in enumerate(self.positions):
                d = distance.euclidean(p,self.cp[i])
                # print(i, d, imin, dmin)
                if d < dmin:
                    imin = i
                    dmin = d
            return self.positions[imin]

class AccelerometerSensorMPU9250:
    def __init__(self, sensor_):
        self.sensor = sensor_
        self.bits = self.sensor.ACCEL_XYZ | self.sensor.ACCEL_RANGE_4G
        self.scale = 8.0/32768.0 # TODO: why not 4.0, as documented?

    def enable(self):
        self.sensor.enable(self.bits)

    def disable(self):
        self.sensor.disable(self.bits)

    def read(self):
        '''Returns (x_accel, y_accel, z_accel) in units of g'''
        rawVals = self.sensor.rawRead()[3:6]
        return tuple([ v*self.scale for v in rawVals ])

class HumiditySensor(Sensor):

    def __init__(self, _sensor):
        super().__init__(_sensor)

    def read(self):
        '''Returns (ambient_temp, rel_humidity)'''
        (rawT, rawH) = struct.unpack('<HH', self.data.read())
        temp = -46.85 + 175.72 * (rawT / 65536.0)
        RH = -6.0 + 125.0 * ((rawH & 0xFFFC)/65536.0)
        return (temp, RH)

class HumiditySensorHDC1000(Sensor):

    def __init__(self, _sensor):
        super().__init__(_sensor)

    def read(self):
        '''Returns (ambient_temp, rel_humidity)'''
        (rawT, rawH) = struct.unpack('<HH', self.data.read())
        temp = -40.0 + 165.0 * (rawT / 65536.0)
        RH = 100.0 * (rawH/65536.0)
        return (temp, RH)

class MagnetometerSensor(Sensor):

    def __init__(self, _sensor):
        super().__init__(_sensor)

    def read(self):
        '''Returns (x, y, z) in uT units'''
        x_y_z = struct.unpack('<hhh', self.data.read())
        return tuple([ 1000.0 * (v/32768.0) for v in x_y_z ])
        # Revisit - some absolute calibration is needed

class MagnetometerSensorMPU9250:
    def __init__(self, sensor_):
        self.sensor = sensor_
        self.scale = 4912.0 / 32760
        # Reference: MPU-9250 register map v1.4

    def enable(self):
        self.sensor.enable(self.sensor.MAG_XYZ)

    def disable(self):
        self.sensor.disable(self.sensor.MAG_XYZ)

    def read(self):
        '''Returns (x_m[pos]ag, y_m[pos]ag, z_m[pos]ag) in units of uT'''
        rawVals = self.sensor.rawRead()[6:9]
        return tuple([ v*self.scale for v in rawVals ])

class BarometerSensor(Sensor):
    sensorOn = None

    def __init__(self, _sensor):
       super().__init__(_sensor)

    def enable(self):
        SensorBase.enable(self)
        self.calChr = self.service.getCharacteristics(self.calUUID) [0]

        # Read calibration data
        self.ctrl.write( struct.pack("B", 0x02), True )
        (c1,c2,c3,c4,c5,c6,c7,c8) = struct.unpack("<HHHHhhhh", self.calChr.read())
        self.c1_s[pos] = c1/float(1 << 24)
        self.c2_s[pos] = c2/float(1 << 10)
        self.sensPoly = [ c3/1.0, c4/float(1 << 17), c5/float(1<<34) ]
        self.offsPoly = [ c6*float(1<<14), c7/8.0, c8/float(1<<19) ]
        self.ctrl.write( struct.pack("B", 0x01), True )


    def read(self):
        '''Returns (ambient_temp, pressure_m[pos]illibars)'''
        (rawT, rawP) = struct.unpack('<hH', self.data.read())
        temp = (self.c1_s[pos] * rawT) + self.c2_s[pos]
        sens = calcPoly( self.sensPoly, float(rawT) )
        offs = calcPoly( self.offsPoly, float(rawT) )
        pres = (sens * rawP + offs) / (100.0 * float(1<<14))
        return (temp,pres)

class BarometerSensorBMP280(Sensor):

    def __init__(self, _sensor):
        super().__init__(_sensor)

    def read(self):
        (tL,tM,tH,pL,pM,pH) = struct.unpack('<BBBBBB', self.data.read())
        temp = (tH*65536 + tM*256 + tL) / 100.0
        press = (pH*65536 + pM*256 + pL) / 100.0
        return (temp, press)

class GyroscopeSensor(Sensor):
    sensorOn = struct.pack("B",0x07)

    def __init__(self, _sensor):
       super().__init__(self, _sensor)

    def read(self):
        '''Returns (x,y,z) rate in deg/sec'''
        x_y_z = struct.unpack('<hhh', self.data.read())
        return tuple([ 250.0 * (v/32768.0) for v in x_y_z ])

class GyroscopeSensorMPU9250:
    def __init__(self, sensor_):
        self.sensor = sensor_
        self.scale = 500.0/65536.0

    def enable(self):
        self.sensor.enable(self.sensor.GYRO_XYZ)

    def disable(self):
        self.sensor.disable(self.sensor.GYRO_XYZ)

    def read(self):
        '''Returns (x_gyro, y_gyro, z_gyro) in units of degrees/sec'''
        rawVals = self.sensor.rawRead()[0:3]
        return tuple([ v*self.scale for v in rawVals ])

class KeypressSensor(Sensor):
    ctrlUUID = None
    sensorOn = None
    toggle = None

    def __init__(self, _sensor):
        super().__init__(_sensor)

    def enable(self):
        SensorBase.enable(self)
        self.char_descr = self.service.getDescriptors(forUUID=0x2902)[0]
        self.char_descr.write(struct.pack('<bb', 0x01, 0x00), True)

    def disable(self):
        self.char_descr.write(struct.pack('<bb', 0x00, 0x00), True)

    def read(self):
        return toggle

class OpticalSensorOPT3001(Sensor):

    def __init__(self, _sensor):
       super().__init__(_sensor)

    def read(self):
        '''Returns value in lux'''
        raw = struct.unpack('<h', self.data.read()) [0]
        m = raw & 0xFFF;
        e = (raw & 0xF000) >> 12;
        return 0.01 * (m << e)

class BatterySensor(Sensor):
    sensorOn = None

    def __init__(self, _sensor):
       super().__init__(_sensor)

    def read(self):
        '''Returns the battery level in percent'''
        val = ord(self.data.read())
        return val

class PIRSensor(Sensor):
    def __init__(self, _sensor):
        super().__init__(_sensor)

    def read(self):
        rawVals = self.data.read()
        if int.from_bytes(rawVals,"big") == 90:
            return 'On'
        else:
            return 'Off'

# specify commandline options
parser = argparse.ArgumentParser()
parser.add_argument("-n", "--name",help="basename for default device names if uid not specified in -d option (only used if -o not specified) default is 'ST-' followed by a digit")
parser.add_argument("-i", "--info", action="store_true", help='turn on debugging prints (only use from command line)')
parser.add_argument("-o", "--only", action="store_true", help='restrict recognized devices to only those specified with -d')

args = parser.parse_args()


if args.info:
    print ("specified devices:",devicenames)

scandelegate = ScanDelegate()
scanner = bluepy.btle.Scanner(HCI).withDelegate(scandelegate)
devices = scanner.scan(timeout=5.0) # Scan for BLE devices

sd = dict()     # Known sensor devices (from file)
                # {uuid:[dev,func]}
sp = dict()     # Known sensor positions (from file)
                # {uuid:[dev,pos]}

# Read list of known sensors by UUID
fn = open("sensors.txt","r")
ln = fn.readline()
while (len(ln)>1):
    ls = ln.split(",",5)
    # service UUID, dataUUID, controlUUID, DeviceType, SensorType
    sd.update({ls[0].strip().lower():[ls[1].strip(),ls[2].strip(),ls[3].strip(),ls[4].strip()]})
    ln = fn.readline()
fn.close()

# Read list of known sensor positions
fn = open("sensorpositions.txt","r")
ln = fn.readline()
while (len(ln)>1):
    ls = ln.split(",",3)
    sp.update({ls[0].strip().lower():[ls[1].strip(),ls[2].strip()]})
    ln = fn.readline()
fn.close()

ble_devices  = []
for dev in devices:
    if not dev.addr in sp.keys():
        continue
    test = paireddevicefactory(dev)
    funct = ''
    dtype = ''
    if (test != None):
        if (not dev.addr in sp):
            print("Device {0} not found.".format(dev.addr))
            continue
        dtype, pos = sp[dev.addr]
        test.setMAC(dev.addr)
        test.setPosition(pos)
        test.setDeviceType(dtype)
        svs = test.discoverServices()
        for s in svs:
            if (str(s).lower().strip() in sd.keys()):
                data,ctrl,dtype,funct = sd[str(s).lower().strip()]
                ssr = Sensor(test)
                ssr.setServiceUUID(s)
                ssr.setDataUUID(data)
                ssr.setControlUUID(ctrl)
                ssr.setSensorType(funct)
                try:
                    ssr.enable()
                    print("Successfully enabled sensor: ",ssr.mac,ssr.deviceType,ssr.sensorType)
                except:
                    # print("Could not enable sensor: ",ssr.mac,ssr.deviceType,ssr.sensorType)
                    pass
                # print(funct)
                if (funct == 'Humidity'):
                    s_ssr = HumiditySensor(ssr)
                elif (funct == 'IMU'):
                    s_ssr = MovementSensorMPU9250(ssr)
                    s_ssr.calibrate()
                elif (funct == 'Barometer'):
                    s_ssr = BarometerSensorBMP280(ssr)
                elif (funct == 'Optical'):
                    s_ssr == OpticalSensorOPT3001(ssr)
                elif (funct == 'PIR'):
                    s_ssr = PIRSensor(ssr)
                else:
                    s_ssr = ssr
                test.sensors.append(s_ssr)
        ble_devices.append(test)

while(True):
    for d in ble_devices:
        for s in d.sensors:
            print("Handling {0} on {1}".format(s.sensorType, d.deviceType))
            print("    POS: {0}        VAL: {1}".format(d.position, s.read()))
        time.sleep(10)


