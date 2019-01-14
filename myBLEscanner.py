import bluepy
import threading
import struct
import argparse
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
                sensors.append(ssr)
x = None   # random sensor
                
for s in sensors:    
    print("Sensor:")
    print(s.position, s.mac, s.deviceType, s.sensorType)
    if ('CC2650' in s.deviceType):
        x = s
    print(s.svcUUID)
    print(s.dataUUID)
    print(s.ctrlUUID)

print(x.read())
            
        
        
