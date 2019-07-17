#import struct
#struct.pack('f', 3.141592654)
#b'\xdb\x0fI@'
#struct.unpack('f', b'\xdb\x0fI@')
#(3.1415927410125732,)
#struct.pack('4f', 1.0, 2.0, 3.0, 4.0)
#'\x00\x00\x80?\x00\x00\x00@\x00\x00@@\x00\x00\x80@'
#from setuptools.command import easy_install

import struct
import sys
import glob
from multiprocessing import Pool, Value
import multiprocessing
import datetime
import math
import logging
import os.path

filename=r'errorLog\BLtranslatelogErrors.log'
dirname = os.path.join(os.getcwd(),os.path.dirname(filename))
if not os.path.exists(dirname):
    os.makedirs(dirname)

logging.basicConfig(level=logging.DEBUG, filename=os.path.join(os.getcwd(),filename) )

#dictionary of CAN data
GPStimePGN = 0xfee6; PGStimeCB = 0;
GPSlocPGN = 0xfef3; GPSlocCB = 0;
GPSspdPGN = 0xfee8; PGSspdCB = 0;
scalePGN = 0xffff; scaleCB = 0xce0e; cumVolscCB = 0xce09; cumvolCB = 0xce0a;
elevMesaPGN = 0xfffe; elevMesaCB = 0x58; fanspdCB = 0x59;
elevAriesPGN = 0xfff8; elevAriesCB = 0x29
WheelSpdPGN = 0xfe6e; wheelSpdCB = 0;

IDdict = [GPStimePGN, GPSlocPGN, GPSspdPGN, elevMesaPGN, elevAriesPGN, WheelSpdPGN]

class canobject:
    def __init__(self, timestamp=0,ID=0, data = [], SA = 0):
        self.ts = timestamp
        self.ID = ID
        self.data = data
        self.SA = SA

def init(args):
    ''' store the counter for later use '''
    global counter
    counter = args

# example:
def chunks(l, n):
    n = max(1, n)
    return ('%.3f' % round(struct.unpack('>f', l[i:i + n])[0],3) for i in range(0, len(l), n))

def chunks_fanPrs(l):
    n = max(1, 2)
    return ('{0}' .format(int(7500*(struct.unpack('>f', l[i+4:i + 8])[0])/(struct.unpack('>f', l[i:i + 4])[0]) - 750)) for i in range(0, len(l), 8))

def chunks_CANbus(l):
    for i in range(0, len(l), 24):
        x = canobject()
        x.ID = struct.unpack('>H', l[i+9:i + 11])[0]
        if x.ID in IDdict:
            x.SA = l[i+11]
            x.ts = (struct.unpack('>Q', l[i:i + 8])[0])/10000000
            x.data = []
            for n in range(i+16,i+24):
                x.data.append(l[n])
            x.data = bytearray(x.data)
            yield x

def fanprsProc(fin):
    dirname = os.path.join(os.getcwd(),os.path.dirname(fin),'extracted')
    newfile = os.path.join(dirname, os.path.basename(fin) + '.csv')
    
    if not os.path.exists(dirname):
        os.makedirs(dirname)

    with open(fin, "rb") as f:
        data = chunks_fanPrs(f.read())

    with open(newfile,'w') as f:
        for x in data:
            f.write(x + "\n")
    counter.value -= 1
    print(counter.value)
    sys.stdout.flush()

def accelProc(fin):
    dirname = os.path.join(os.getcwd(),os.path.dirname(fin),'extracted')
    newfile = os.path.join(dirname, os.path.basename(fin) + '.csv')
    
    if not os.path.exists(dirname):
        os.makedirs(dirname)

    with open(fin, "rb") as f:
        data = chunks(f.read(),4)

    with open(newfile,'w') as f:
        f.write('S1,S2,S3,S4\n')
        for x in data:
            f.write(",".join([x,next(data),next(data),next(data)]) + "\n")
    counter.value -= 1
    print(counter.value)
    sys.stdout.flush()

def canProc(fin):
    dirname = os.path.join(os.getcwd(),os.path.dirname(fin),'extracted')
    newfile = os.path.join(dirname, os.path.basename(fin) + '.csv')
    
    if not os.path.exists(dirname):
        os.makedirs(dirname)

    with open(fin, "rb") as f:
        data = chunks_CANbus(f.read())
    with open(newfile,'w') as f:
        f.write('filetime,GPStime,lat,lon,GPSspd,fanspd,elevstat,Lwheelspd,Rwheelspd\n')
             
        for d in data:
            try: 
                if d.ID == GPStimePGN and d.SA == 0x1c:
                    GPStime = datetime.datetime(1985+d.data[5],d.data[3], math.ceil(d.data[4]*.25),d.data[2],d.data[1],math.floor(d.data[0]*0.25))
                elif d.ID == GPSlocPGN and d.SA == 0x1c:
                    lat = struct.unpack('<I', d.data[0:4])[0] * 0.0000001 - 210
                    lon = struct.unpack('<I', d.data[4:8])[0] * 0.0000001 - 210
                elif d.ID == GPSspdPGN and d.SA == 0x1c:
                    GPSspd = struct.unpack('<H', d.data[2:4])[0]/256
                #elif d.ID == scalePGN:
                #    if struct.unpack('>H', d.data[0:2])[0] == scaleCB:
                #        pass
                #    elif struct.unpack('>H', d.data[0:2])[0] == cumVolscCB:
                #        pass
                elif d.ID == elevMesaPGN:
                    if d.data[0] == elevMesaCB:
                        elevstat = d.data[2] & 0x03
                    elif d.data[0] == fanspdCB:
                        fanspd = struct.unpack('<H', d.data[1:3])[0]*0.125
                        f.write('{0},{1},{2},{3},{4},{5},{6},{7}, {8}\n' .format(d.ts,str(GPStime),lat,lon,GPSspd,fanspd,elevstat,LwhlSpd,RwhlSpd))
                elif d.ID == elevAriesPGN and d.data[0] == elevAriesCB:
                    elevstat = d.data[3] & 0x03
                elif d.ID == WheelSpdPGN:
                    LwhlSpd = struct.unpack('<H', d.data[4:6])[0] * 0.001
                    RwhlSpd = struct.unpack('<H', d.data[6:8])[0] * 0.001

            except Exception as e: 
                logging.exception("oops")

        counter.value -= 1
        print(counter.value)
        sys.stdout.flush()
    #newfile = fin + '_extracted.csv'
    #with open(newfile,'w') as f:
    #    f.write('CAN_time,fanspd,GPSspd,Date,elevStat\n')
    #    for x in data:
    #        f.write(",".join([x,next(data),next(data),next(data)]) + "\n")
    #counter.value -= 1
    #print(counter.value)
    #sys.stdout.flush()

if __name__ == "__main__":

    ###########################  accelerometer data ####################################
    print("extracting accelerometer data...")

    accelfiles = glob.glob('*_accel_*')

    files = [x for x in accelfiles if '.csv' not in x]

    counter = Value('i',len(files))

    p = Pool(processes = multiprocessing.cpu_count(), initializer = init, initargs = (counter,))

    p.map(accelProc, files)

    ################################## CANbus data ##########################################

    print("extracting CANbus data...")

    canfiles = glob.glob('*_CANbus*')

    #canfiles = glob.glob(r'D:\Sugarcane\billet loss\FL2016\01-26-16\*_CANbus*')

    files = [x for x in canfiles if '.csv' not in x]

    counter = Value('i',len(files))

    p = Pool(processes = multiprocessing.cpu_count(), initializer = init, initargs = (counter,))

    p.map(canProc, files)


    ################################## Fan Pressure data ##########################################

    print("extracting FANpressure data...")

    fanprsfiles = glob.glob('*_fanPressure*')

    #fanprsfiles = glob.glob(r'D:\Sugarcane\billet loss\FL2016\01-26-16\*_fanPressure*')

    files = [x for x in fanprsfiles if '.csv' not in x]

    counter = Value('i',len(files))

    p = Pool(processes = multiprocessing.cpu_count(), initializer = init, initargs = (counter,))

    p.map(fanprsProc, files)

    #######for f in files:
    #######    print(f)
    #######    input()

    ######with open(r'D:\Sugarcane\billet loss\FL2016\01-26-16\02_29PM_accel_700rpm_noGT', "rb") as f:
    ######    data = chunks(f.read(),4)
    ######    #while data:
    ######    #    for i in range(0, len(data), 4):
    ######    #        print(",".join(data[i:i+4]))
    ######    #    data = chunks(f.read(16),4)
    ######    #    input()

    ######    #data = chunks(f.read(),4)
    ######with open(r'C:\Users\John\Desktop\accellBL.csv','w') as f:
    ######    for x in data:
    ######        f.write(",".join([x,next(data),next(data),next(data)]) + "\n")
        
    ######    #for i in range(0, len(data), 4):
    ######    #    f.write(",".join(data[i:i+4]) + "\n")
