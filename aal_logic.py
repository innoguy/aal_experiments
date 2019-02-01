import datetime
import time

fmt = '%Y-%m-%d %H:%M:%S'

def now():
    # ts = time.strftime(fmt, time.gmtime())
    ts = '2019-01-17 23:50:00'
    return str_to_time(ts)
    
def str_to_time(ts):
    return datetime.datetime.strptime(ts, fmt)
    
def debug(_msg):
    pass
    # print('DEBUG : ',_msg)
    
def report(_msg):
    print("{0}: {1}".format(now(),_msg))
        
class Room:
    rnames=['LivingRoom','Kitchen','Bathroom','Hallway','Outdoor']

    def __init__(self, _name=None):
        self.name = None
        
        if _name in self.rnames:
            self.name = _name
        else:
            debug('Unknown room name {0}'.format(_name))
            
class Sensor:
    dtypes=['AirMentor','SensorTag','Niko','Feather','Elgato']
    stypes=['AQI','IMU','TH','PRS','PIR','POW']
    positions=['Window','Bed','Sofa','Door','TV','Kettle']
    directions=Room.rnames+[None]
    
    def __init__(self, _dtype=None, _stype=None, _room=None, _position=None, _direction=None):
        
        self.dtype = None
        self.stype = None
        self.room = None
        self.position = None
        self.direction = None
        self.value = {}
        self.lastUpdate = None
        self.state = None
        
        if (_dtype in self.dtypes):
            self.dtype = _dtype
        else:
            debug('Unknown sensor device type')
        if (_stype in self.stypes):
            self.stype = _stype
        else:
            debug('Unknown sensor type')
        self.value = None
        
        if _room in Room.rnames:
            self.room = _room
        else:
            debug("Tried to add sensor to unknown room name")
        if _position in self.positions:
            self.position = _position
        else:
            debug("Tried to assign sensor to unknown position")
        if _direction in self.directions:
            self.direction = _direction
        else:
            debug("Tried to add sensor in unknown direction")   
             
    def setRoom(self, _room):
        self.room = _room
        
    def setPosition(self, _room, _position):
        self.room = _room
        self.position = _position
        
    def setValue(self, _val):
        lastUpdate = now()
        for vkey in _val.keys():
            if (self.value == None):
                self.value = {}
            if (vkey not in self.value.keys()):
                self.value.update(_val)
            elif (vkey in self.value.keys()):
                self.value[vkey] = _val[vkey]
                
    def getState(self):
        if (self.stype == 'IMU'):
            if (self.value['POS'] == 'Closed'):
                self.state = 'GRE'
            elif (self.value['POS'] == 'Closed') and (now()-self.lastUpdate > 3600):
                self.state = 'YEL'
            elif (self.value['POS'] == 'Closed') and (now()-self.lastUpdate > 7200):
                self.state = 'RED'
        elif (self.stype == 'AQI'):
            if self.value['AQI'] < 30:
                self.state = 'GRE'
            elif self.value['AQI' < 100]:
                self.state = 'YEL'
            else:
                self.state = 'RED'
        elif (self.stype == 'POW'):
            if (self.value['POW'] == 'Off'):
                self.state = 'GRE'
            elif (self.value['POW'] == 'On') and (now()-self.lastUpdate > 3600):
                self.state = 'YEL'
            elif (self.value['POW'] == 'On') and (now()-self.lastUpdate > 7200):
                self.state = 'RED'
    
class House:
    def __init__(self):
        self.rooms = list()
        self.sensors = dict()
        self.state = None
        self.lastLocation = None
                
    def addRoom(self,_name):    
        self.rooms.append(Room(_name))
        
    def addSensor(self,_name,_dtype,_stype,_loc,_pos=None,_dir=None):
        ssr = Sensor(_dtype,_stype,_loc,_pos,_dir)
        self.sensors.update({_name:ssr})
        return ssr    
        
    def describe(self):
        print("{0} rooms defined:".format(len(self.rooms)))
        for r in self.rooms:
            print('- {0}'.format(r.name))
        print("{0} sensors defined:".format(len(self.sensors)))
        for skey in self.sensors:
            s = self.sensors[skey]
            if (s.position == None):
                print('- [{0}/{1}] in {2}'.format(s.dtype,s.stype,s.room))
            else:
                print('- [{0}/{1}] in {2} at {3} to {4}'.format(s.dtype,s.stype,s.room,s.position,s.direction))

    def event(self, _time, _sensorkey,_value):
        if (_time < now()):
            self.sensors[_sensorkey].setValue(_value)
            self.sensors[_sensorkey].lastUpdate = _time
            s = self.sensors[_sensorkey]
            # Event from a PIR sensor
            if (s.stype == 'PIR'):
                if (s.room == self.lastLocation):
                    pass
                else:
                    self.lastLocation = s.room
                    # report("[GRE] {0} Person detected in {1}".format(_time, s.room))
            # Event from an air quality monitor (AQI)
            elif (s.stype == 'AQI'):
                if (s.value['AQI'] > 100):
                    report("[RED] {0} Very bad air detected in {1}".format(_time, s.room))
                elif (s.value['AQI'] > 30):
                    report("[YEL] {0} Unpure air detected in {1}".format(_time, s.room))
        
    def check(self):  
        lastSeen = None
        # Has the person been seen anywhere in the house?
        for skey in self.sensors:
            s = self.sensors[skey]
            if (s.stype == 'PIR'):
                if (s.lastUpdate != None):
                    if (lastSeen == None):
                        lastSeen = s.lastUpdate
                    elif (lastSeen < s.lastUpdate):
                        lastSeen = s.lastUpdate
                        self.lastLocation = s.room
        td = now() - lastSeen
        if (td.total_seconds() > 10):
            report("[YEL] {0} Nobody seen for more than {1}".format(now(),td))
        # Are any doors or windows open?
        for skey in self.sensors:
            s = self.sensors[skey]
            if (s.stype == 'IMU'):
                if (s.value != None):
                    if (s.value['POS']=='Open'):
                        if (s.lastUpdate != None):
                            td = now() - s.lastUpdate
                            if (td.total_seconds() > 3600):
                                report("[YEL] {0} {1} {2} open for more than {3}".format(now(), s.position, skey, td))
        # Any high power devices on for long time?
        for skey in self.sensors:
            s = self.sensors[skey]
            if (s.stype == 'POW'):
                if (s.value != None):
                    if (s.value['POW']=='On'):
                        if (s.lastUpdate != None):
                            td = now() - s.lastUpdate
                            if (td.total_seconds() > 3600):
                                report("[YEL] {0} {1} {2} on for more than {3}".format(now(), s.position, skey, td))
        

house = House()  

house.addRoom('LivingRoom')
house.addRoom('Kitchen')
house.addRoom('Bathroom')
house.addRoom('Hallway')
house.addRoom('Outdoor')
house.addRoom('Terrace')

house.addSensor('kitchen_PIR','Niko','PIR','Kitchen')
house.addSensor('Bathroom_PIR','Niko','PIR','Bathroom')
house.addSensor('living_PIR','Niko','PIR','LivingRoom')
house.addSensor('living_AQI','AirMentor','AQI','LivingRoom')
house.addSensor('kitchen_window','SensorTag','IMU','Kitchen','Window','Outdoor')
house.addSensor('living_bathroom','SensorTag','IMU','LivingRoom','Door','Bathroom')
house.addSensor('frontdoor','SensorTag','IMU','Hallway','Door','Outdoor')
house.addSensor('bed','Feather','PRS','Living','Bed', None)
house.addSensor('sofa','Feather','PRS','Living','Sofa', None)
house.addSensor('TV','Elgato','POW','Living','TV', None)
house.addSensor('kettle','Elgato','POW','Kitchen','Kettle', None)

# house.describe()

event_list = [
    ('2019-01-17 10:00:00','kitchen_window',{'POS':'Open'}),
    ('2019-01-17 11:00:00','kitchen_PIR',{'PIR':'On'}),
    ('2019-01-17 11:10:00','living_PIR',{'PIR':'On'}),
    ('2019-01-17 13:00:00','living_AQI',{'AQI':50}),
    ('2019-01-17 13:00:00','frontdoor',{'POS':'Open'}),
    ('2019-01-17 14:00:00','living_PIR',{'PIR':'On'}),
    ('2019-01-17 19:00:00','Bathroom_PIR',{'PIR':'On'}),    
    ('2019-01-17 19:30:00','kitchen_window',{'POS':'Closed'}),
    ('2019-01-17 20:00:00','kettle',{'POW':'On'}),
    ]


for e in event_list:    
    if (str_to_time(e[0]) > now()):
        break
    house.event(str_to_time(e[0]),e[1],e[2])

house.check()
print("Person probably in {0}".format(house.lastLocation))

"""
Possible elements for definition of system state: 
- time spent in each PIR-equipped room
- time each POW plugged device is on
- time each IMU tagged window/door is open
- time spent in PRS equipped bed/sofa/chair

"""

