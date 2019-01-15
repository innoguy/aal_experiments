import bluepy
import threading
import struct
import argparse
import time
from bluepy.btle import UUID, Peripheral, DefaultDelegate, AssignedNumbers

devicenames = {}
DEVICENAMESFILE = 'devices.txt'
AUTODETECT = "-"
BTNAME='Complete Local Name'

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
        return BLE_Device( dev, devdata )
    
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
    #                thisdev.pair()
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
        print("Initialized peripheral. Scanning services.")
        if version==AUTODETECT:
            svcs = self.discoverServices()
            print("Service discovery completed.")

class SensorBase:
    def __init__(self, periph):
        self.periph = periph
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
            
class Sensor(SensorBase):
    def __init__(self, periph):
        SensorBase.__init__(self, periph)
    
        self.mac = None
        self.position = None
        self.sensorType = None
        self.deviceType = None
        self.uuid = None
        self.handle = None
        self.svcUUID = None
        self.dataUUID = None
        self.ctrlUUID = None
        self.ctrlBits = 0

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
        if (self.sensorType=='Humidity'):
            (rawT, rawH) = struct.unpack('<HH', self.data.read())
            temp = -46.85 + 175.72 * (rawT / 65536.0)
            RH = -6.0 + 125.0 * ((rawH & 0xFFFC)/65536.0)
            return (temp, RH)
        elif (self.sensorType=='Barometer'):    
            (tL,tM,tH,pL,pM,pH) = struct.unpack('<BBBBBB', self.data.read())
            temp = (tH*65536 + tM*256 + tL) / 100.0
            press = (pH*65536 + pM*256 + pL) / 100.0
            return (temp, press)
        elif (self.sensorType=='IMU'):
            # GYRO
            rawVals = self.rawRead()[0:3]
            gyr = tuple([ v*500.0/65536.0 for v in rawVals ])  
            # MAGN
            rawVals = self.rawRead()[6:9]
            mag = tuple([ v*4912.0/32760.0 for v in rawVals ]) 
            # ACCEL
            rawVals = self.rawRead()[3:6]
            acc = tuple([ v*8.0/32768.0 for v in rawVals ])
            return ((gyr,mag,acc))
        else:
            return("Cannot read this type of sensor.")        

    def rawRead(self):
        dval = self.data.read()
        return struct.unpack("<hhhhhhhhh", dval)

        
    

# specify commandline options       
parser = argparse.ArgumentParser()
parser.add_argument("-n", "--name",help="basename for default device names if uid not specified in -d option (only used if -o not specified) default is 'ST-' followed by a digit")
parser.add_argument("-i", "--info", action="store_true", help='turn on debugging prints (only use from command line)') 
parser.add_argument("-o", "--only", action="store_true", help='restrict recognized devices to only those specified with -d')

args = parser.parse_args()

        
if args.info:
    print ("specified devices:",devicenames)

scandelegate = DefaultDelegate()
scanner = bluepy.btle.Scanner().withDelegate(scandelegate)
devices = scanner.scan(timeout=5.0)

sd = dict()     # Known sensor devices (from file)
                # {uuid:[dev,func]}
sp = dict()     # Known sensor positions (from file)
                # {uuid:[dev,pos]}
sensors=[]      # list of Sensor devices

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

tag  = list(range(len(devices)))
for i, dev in enumerate(devices):
    test = paireddevicefactory(dev)
    funct = ''
    dtype = ''
    if (test != None):
        tag[i] = test
        addr = dev.addr
        if (not addr in sp):
            print("Device {0} not found.".format(addr))
            continue
        dtype, pos = sp[addr]
        svs = tag[i].discoverServices()
        for s in svs:                
            if (str(s).lower().strip() in sd.keys()):
                data,ctrl,dtype,funct = sd[str(s).lower().strip()]
                ssr = Sensor(test)
                ssr.setMAC(addr)
                ssr.setPosition(pos)
                ssr.setDeviceType(dtype)
                ssr.setServiceUUID(s)
                ssr.setDataUUID(data)
                ssr.setControlUUID(ctrl)
                ssr.setSensorType(funct)
                try:
                    ssr.enable()
                    print("Successfully enabled sensr: ",ssr.mac,ssr.deviceType,ssr.sensorType)
                except:
                    print("Could not enable sensor: ",ssr.mac,ssr.deviceType,ssr.sensorType)
                sensors.append(ssr)
       
while(True):       
    for s in sensors:    
        if (s.deviceType=='CC2650' and s.sensorType=='Humidity'):
            temp, hum = s.read()
            print("POS:",s.position,"TEMP:",temp,"HUM:",hum)
        elif (s.deviceType=='CC2650' and s.sensorType=='Barometer'):
            temp, pres = s.read()
            print("POS:",s.position,"TEMP:",temp,"PRES:",pres)
        elif (s.deviceType=='CC2650' and s.sensorType=='IMU'):
            print("POS:",s.position,"IMU:",s.read())
    time.sleep(10)            
        
        
