import ctypes
import platform
import sys
import time


#def errorHandler(temp):
#    if temp != 0:
#        print(f"Error: {temp}")
#        exit()

def errorHandler(temp):
    if temp != 0:
        print(f"Error: {temp}")

        tt = ctypes.create_string_buffer(1024)
        pRecords = ctypes.cast(tt, ctypes.c_char_p)

        while temp != 0:
            atc3dg.GetErrorText(ctypes.c_int32(temp), pRecords, ctypes.c_int32(1024), ctypes.c_int32(1))
            print(pRecords.value.decode('utf-8'))
            #error_message = pRecords.contents.value.decode('utf-8')
            #print(error_message)
            print()
            raise Exception("Driver will terminate.")
        exit()


# Determine if platform is 32 or 64 bit
if platform.architecture()[0] == '64bit':
    libstring = 'ATC3DG64'
else:
    libstring = 'ATC3DG'

# Loading the ATC3DG library
atc3dg = ctypes.WinDLL(libstring)

# Check if loading library is successful
if not bool(atc3dg._handle):
    raise OSError('Problem loading library: WinDLL')

# Initialize the system
temp = atc3dg.InitializeBIRDSystem()
errorHandler(temp)

# Get system configuration
from enum import Enum
class AGC_MODE_TYPE(Enum):
    TRANSMITTER_AND_SENSOR_AGC = 0
    SENSOR_AGC_ONLY = 1

class SYSTEM_CONFIGURATION(ctypes.Structure):
    _fields_ = [("measurementRate", ctypes.c_double),
                ("powerLineFrequency", ctypes.c_double),
                ("maximumRange", ctypes.c_double),
                ("agcMode", ctypes.c_int),
                ("numberBoards", ctypes.c_int),
                ("numberSensors", ctypes.c_int),
                ("numberTransmitters", ctypes.c_int),
                ("transmitterIDRunning", ctypes.c_int),
                ("metric", ctypes.c_bool)]

Record3 = SYSTEM_CONFIGURATION()
Record3.agcMode = 0
pRecord3 = ctypes.pointer(Record3)
temp3 = atc3dg.GetBIRDSystemConfiguration(pRecord3)
errorHandler(temp3)
measurementRate = Record3.measurementRate
numBoards = Record3.numberBoards

# Get sensor configuration
class DEVICE_TYPES(Enum):
    STANDARD_SENSOR = 0
    TYPE_800_SENSOR = 1
    STANDARD_TRANSMITTER = 2
    MINIBIRD_TRANSMITTER = 3
    SMALL_TRANSMITTER = 4
    TYPE_500_SENSOR = 5
    TYPE_180_SENSOR = 6
    TYPE_130_SENSOR = 7
    TYPE_TEM_SENSOR = 8
    UNKNOWN_SENSOR = 9
    UNKNOWN_TRANSMITTER = 10
    TYPE_800_BB_SENSOR = 11
    TYPE_800_BB_STD_TRANSMITTER = 12
    TYPE_800_BB_SMALL_TRANSMITTER = 13
    TYPE_090_BB_SENSOR = 14

class SENSOR_CONFIGURATION(ctypes.Structure):
    _fields_ = [("serialNumber", ctypes.c_ulong),
                ("boardNumber", ctypes.c_ushort),
                ("channelNumber", ctypes.c_ushort),
                ("type", ctypes.c_int),
                ("attached", ctypes.c_bool)]

if numBoards == 1:
    Records0 = SENSOR_CONFIGURATION()
    Records0.type = 11
    pRecords0 = ctypes.byref(Records0)
    temps0 = atc3dg.GetSensorConfiguration(0, pRecords0)
    errorHandler(temps0)

    Records1 = SENSOR_CONFIGURATION()
    Records1.type = 11
    pRecords1 = ctypes.byref(Records1)
    temps1 = atc3dg.GetSensorConfiguration(1, pRecords1)
    errorHandler(temps1)

    Records2 = SENSOR_CONFIGURATION()
    Records2.type = 11
    pRecords2 = ctypes.byref(Records2)
    temps2 = atc3dg.GetSensorConfiguration(2, pRecords2)
    errorHandler(temps2)

    Records3 = SENSOR_CONFIGURATION()
    Records3.type = 11
    pRecords3 = ctypes.byref(Records3)
    temps3 = atc3dg.GetSensorConfiguration(3, pRecords3)
    errorHandler(temps3)

    SensorNumAttached = [
        Records0.attached,
        Records1.attached,
        Records2.attached,
        Records3.attached,
    ]

elif numBoards == 2:
    Records0 = SENSOR_CONFIGURATION()
    Records0.type = 11
    pRecords0 = ctypes.byref(Records0)
    temps0 = atc3dg.GetSensorConfiguration(0, pRecords0)
    errorHandler(temps0)

    Records1 = SENSOR_CONFIGURATION()
    Records1.type = 11
    pRecords1 = ctypes.byref(Records1)
    temps1 = atc3dg.GetSensorConfiguration(1, pRecords1)
    errorHandler(temps1)

    Records2 = SENSOR_CONFIGURATION()
    Records2.type = 11
    pRecords2 = ctypes.byref(Records2)
    temps2 = atc3dg.GetSensorConfiguration(2, pRecords2)
    errorHandler(temps2)

    Records3 = SENSOR_CONFIGURATION()
    Records3.type = 11
    pRecords3 = ctypes.byref(Records3)
    temps3 = atc3dg.GetSensorConfiguration(3, pRecords3)
    errorHandler(temps3)

    Records4 = SENSOR_CONFIGURATION()
    Records4.type = 11
    pRecords4 = ctypes.byref(Records4)
    temps4 = atc3dg.GetSensorConfiguration(4, pRecords4)
    errorHandler(temps4)

    Records5 = SENSOR_CONFIGURATION()
    Records5.type = 11
    pRecords5 = ctypes.byref(Records0)
    temps5 = atc3dg.GetSensorConfiguration(5, pRecords5)
    errorHandler(temps5)

    Records6 = SENSOR_CONFIGURATION()
    Records6.type = 11
    pRecords6 = ctypes.byref(Records6)
    temps6 = atc3dg.GetSensorConfiguration(6, pRecords6)
    errorHandler(temps6)

    Records7 = SENSOR_CONFIGURATION()
    Records7.type = 11
    pRecords7 = ctypes.byref(Records7)
    temps7 = atc3dg.GetSensorConfiguration(7, pRecords7)
    errorHandler(temps7)

    SensorNumAttached = [
        Records0.attached,
        Records1.attached,
        Records2.attached,
        Records3.attached,
        Records4.attached,
        Records5.attached,
        Records6.attached,
        Records7.attached,
    ]

elif numBoards == 3:
    Records0 = SENSOR_CONFIGURATION()
    Records0.type = 11
    pRecords0 = ctypes.byref(Records0)
    temps0 = atc3dg.GetSensorConfiguration(0, pRecords0)
    errorHandler(temps0)

    Records1 = SENSOR_CONFIGURATION()
    Records1.type = 11
    pRecords1 = ctypes.byref(Records1)
    temps1 = atc3dg.GetSensorConfiguration(1, pRecords1)
    errorHandler(temps1)

    Records2 = SENSOR_CONFIGURATION()
    Records2.type = 11
    pRecords2 = ctypes.byref(Records2)
    temps2 = atc3dg.GetSensorConfiguration(2, pRecords2)
    errorHandler(temps2)

    Records3 = SENSOR_CONFIGURATION()
    Records3.type = 11
    pRecords3 = ctypes.byref(Records3)
    temps3 = atc3dg.GetSensorConfiguration(3, pRecords3)
    errorHandler(temps3)

    Records4 = SENSOR_CONFIGURATION()
    Records4.type = 11
    pRecords4 = ctypes.byref(Records4)
    temps4 = atc3dg.GetSensorConfiguration(4, pRecords4)
    errorHandler(temps4)

    Records5 = SENSOR_CONFIGURATION()
    Records5.type = 11
    pRecords5 = ctypes.byref(Records0)
    temps5 = atc3dg.GetSensorConfiguration(5, pRecords5)
    errorHandler(temps5)

    Records6 = SENSOR_CONFIGURATION()
    Records6.type = 11
    pRecords6 = ctypes.byref(Records6)
    temps6 = atc3dg.GetSensorConfiguration(6, pRecords6)
    errorHandler(temps6)

    Records7 = SENSOR_CONFIGURATION()
    Records7.type = 11
    pRecords7 = ctypes.byref(Records7)
    temps7 = atc3dg.GetSensorConfiguration(7, pRecords7)
    errorHandler(temps7)

    Records8 = SENSOR_CONFIGURATION()
    Records8.type = 11
    pRecords8 = ctypes.byref(Records8)
    temps8 = atc3dg.GetSensorConfiguration(8, pRecords8)
    errorHandler(temps8)

    Records9 = SENSOR_CONFIGURATION()
    Records9.type = 11
    pRecords9 = ctypes.byref(Records9)
    temps9 = atc3dg.GetSensorConfiguration(9, pRecords9)
    errorHandler(temps9)
    
    Records10 = SENSOR_CONFIGURATION()
    Records10.type = 11
    pRecords10 = ctypes.byref(Records10)
    temps10 = atc3dg.GetSensorConfiguration(10, pRecords10)
    errorHandler(temps10)

    Records11 = SENSOR_CONFIGURATION()
    Records11.type = 11
    pRecords11 = ctypes.byref(Records11)
    temps11 = atc3dg.GetSensorConfiguration(11, pRecords11)
    errorHandler(temps11)

    SensorNumAttached = [
        Records0.attached,
        Records1.attached,
        Records2.attached,
        Records3.attached,
        Records4.attached,
        Records5.attached,
        Records6.attached,
        Records7.attached,
        Records8.attached,
        Records9.attached,
        Records10.attached,
        Records11.attached,
    ]

else:
    Records0 = SENSOR_CONFIGURATION()
    Records0.type = 11
    pRecords0 = ctypes.byref(Records0)
    temps0 = atc3dg.GetSensorConfiguration(0, pRecords0)
    errorHandler(temps0)

    Records1 = SENSOR_CONFIGURATION()
    Records1.type = 11
    pRecords1 = ctypes.byref(Records1)
    temps1 = atc3dg.GetSensorConfiguration(1, pRecords1)
    errorHandler(temps1)

    Records2 = SENSOR_CONFIGURATION()
    Records2.type = 11
    pRecords2 = ctypes.byref(Records2)
    temps2 = atc3dg.GetSensorConfiguration(2, pRecords2)
    errorHandler(temps2)

    Records3 = SENSOR_CONFIGURATION()
    Records3.type = 11
    pRecords3 = ctypes.byref(Records3)
    temps3 = atc3dg.GetSensorConfiguration(3, pRecords3)
    errorHandler(temps3)

    Records4 = SENSOR_CONFIGURATION()
    Records4.type = 11
    pRecords4 = ctypes.byref(Records4)
    temps4 = atc3dg.GetSensorConfiguration(4, pRecords4)
    errorHandler(temps4)

    Records5 = SENSOR_CONFIGURATION()
    Records5.type = 11
    pRecords5 = ctypes.byref(Records0)
    temps5 = atc3dg.GetSensorConfiguration(5, pRecords5)
    errorHandler(temps5)

    Records6 = SENSOR_CONFIGURATION()
    Records6.type = 11
    pRecords6 = ctypes.byref(Records6)
    temps6 = atc3dg.GetSensorConfiguration(6, pRecords6)
    errorHandler(temps6)

    Records7 = SENSOR_CONFIGURATION()
    Records7.type = 11
    pRecords7 = ctypes.byref(Records7)
    temps7 = atc3dg.GetSensorConfiguration(7, pRecords7)
    errorHandler(temps7)

    Records8 = SENSOR_CONFIGURATION()
    Records8.type = 11
    pRecords8 = ctypes.byref(Records8)
    temps8 = atc3dg.GetSensorConfiguration(8, pRecords8)
    errorHandler(temps8)

    Records9 = SENSOR_CONFIGURATION()
    Records9.type = 11
    pRecords9 = ctypes.byref(Records9)
    temps9 = atc3dg.GetSensorConfiguration(9, pRecords9)
    errorHandler(temps9)
    
    Records10 = SENSOR_CONFIGURATION()
    Records10.type = 11
    pRecords10 = ctypes.byref(Records10)
    temps10 = atc3dg.GetSensorConfiguration(10, pRecords10)
    errorHandler(temps10)

    Records11 = SENSOR_CONFIGURATION()
    Records11.type = 11
    pRecords11 = ctypes.byref(Records11)
    temps11 = atc3dg.GetSensorConfiguration(11, pRecords11)
    errorHandler(temps11)

    Records12 = SENSOR_CONFIGURATION()
    Records12.type = 11
    pRecords12 = ctypes.byref(Records12)
    temps12 = atc3dg.GetSensorConfiguration(12, pRecords12)
    errorHandler(temps12)
    
    Records13 = SENSOR_CONFIGURATION()
    Records13.type = 11
    pRecords13 = ctypes.byref(Records13)
    temps13 = atc3dg.GetSensorConfiguration(13, pRecords13)
    errorHandler(temps13)
    
    Records14 = SENSOR_CONFIGURATION()
    Records14.type = 11
    pRecords14 = ctypes.byref(Records14)
    temps14 = atc3dg.GetSensorConfiguration(14, pRecords14)
    errorHandler(temps14)
    
    Records15 = SENSOR_CONFIGURATION()
    Records15.type = 11
    pRecords15 = ctypes.byref(Records15)
    temps15 = atc3dg.GetSensorConfiguration(15, pRecords15)
    errorHandler(temps15)

    SensorNumAttached = [
        Records0.attached,
        Records1.attached,
        Records2.attached,
        Records3.attached,
        Records4.attached,
        Records5.attached,
        Records6.attached,
        Records7.attached,
        Records8.attached,
        Records9.attached,
        Records10.attached,
        Records11.attached,
        Records12.attached,
        Records13.attached,
        Records14.attached,
        Records15.attached,
    ]

# Turn ON Transmitter
print('Turning on transmitter')
int_obj = ctypes.c_int(0)
int_ptr = ctypes.addressof(int_obj)
buffer = ctypes.create_string_buffer(2)
ctypes.memmove(buffer, int_ptr, ctypes.sizeof(ctypes.c_int))
tempSetP = atc3dg.SetSystemParameter(0, ctypes.byref(buffer), 2)
errorHandler(tempSetP)

import numpy as np
# Set sensor parameters

value = 26
int_obj = ctypes.c_int(value)
int_ptr = ctypes.addressof(int_obj)
buffer = ctypes.create_string_buffer(4)
ctypes.memmove(buffer, int_ptr, ctypes.sizeof(ctypes.c_int))

print('Setting sensor parameters')
print(1)
ref = np.int32(26)
buffers1 = ctypes.create_string_buffer(4)
Error1 = atc3dg.SetSensorParameter(0, 0, ctypes.byref(buffer), 4)
errorHandler(Error1)
print(2)
buffers2 = ctypes.create_string_buffer(4)
Error1 = atc3dg.SetSensorParameter(1, 0, ctypes.byref(buffer), 4)
errorHandler(Error1)
print(3)
buffers3 = ctypes.create_string_buffer(4)
Error1 = atc3dg.SetSensorParameter(2, 0, ctypes.byref(buffer), 4)
errorHandler(Error1)
print(4)
buffers4 = ctypes.create_string_buffer(4)
Error1 = atc3dg.SetSensorParameter(3, 0, ctypes.byref(buffer), 4)
errorHandler(Error1)

#for count in range(4*numBoards):
#    print(count)
#    Error1 = atc3dg.SetSensorParameter(count+1, 0, ctypes.byref(buffers[count]), 4)
#    errorHandler(Error1)

if (temp == 0):
    print('Tracking system initialized: Done')
else:
    raise ValueError('Problem initialising the system: InitializeBIRDSystem')

# Free memory
#ctypes.c_free(Record3)
#ctypes.c_free(pRecord3)
#ctypes.c_free(Records0)
#ctypes.c_free(pRecords0)
#ctypes.c_free(Records1)
#ctypes.c_free(pRecords1)
#ctypes.c_free(Records2)
#ctypes.c_free(pRecords2)
#ctypes.c_free(Records3)
#ctypes.c_free(pRecords3)



# Now get data

class DOUBLE_POSITION_ANGLES_TIME_Q_RECORD(ctypes.Structure):
    _fields_ = [('x', ctypes.c_double),
                ('y', ctypes.c_double),
                ('z', ctypes.c_double),
                ('a', ctypes.c_double),
                ('e', ctypes.c_double),
                ('r', ctypes.c_double),
                ('time', ctypes.c_double),
                ('quality', ctypes.c_ushort)]
    
# Get synchronous Record
# Initialize structure
def sensorRead():
    sm = {}
    for kk in range(4 * numBoards):
        sm[f'x{kk}'] = 0
        sm[f'y{kk}'] = 0
        sm[f'z{kk}'] = 0
        sm[f'a{kk}'] = 0
        sm[f'e{kk}'] = 0
        sm[f'r{kk}'] = 0
        sm[f'time{kk}'] = 0
        sm[f'quality{kk}'] = 0

    # Create a pointer to a DOUBLE_POSITION_ANGLES_TIME_Q_RECORD_AllSensors_Four struct
    DOUBLE_POSITION_ANGLES_TIME_Q_RECORD_AllSensors_Four = DOUBLE_POSITION_ANGLES_TIME_Q_RECORD * (4 * numBoards)
    pRecord = ctypes.POINTER(DOUBLE_POSITION_ANGLES_TIME_Q_RECORD_AllSensors_Four)()
    # Initialize the pointer with the values in the sm dictionary
    pRecord.contents = DOUBLE_POSITION_ANGLES_TIME_Q_RECORD_AllSensors_Four(**sm)

    # Call the GetAsynchronousRecord function
    Error = atc3dg.GetAsynchronousRecord(0xffff, pRecord, 4*numBoards*64)
    errorHandler(Error)

    # Extract the data from the record
    Record = pRecord.contents

    tempPos = np.zeros((3, 4*numBoards))
    tempAng = np.zeros((3, 4*numBoards))
    temptime = np.zeros((4*numBoards,))
    tempQuality = np.zeros((4*numBoards,))

    for count in range(4*numBoards):
        tempPos[0, count] = Record[count].x
        tempPos[1, count] = Record[count].y
        tempPos[2, count] = Record[count].z
        tempAng[0, count] = Record[count].a
        tempAng[1, count] = Record[count].e
        tempAng[2, count] = Record[count].r
        temptime[count] = Record[count].time
        tempQuality[count] = Record[count].quality

    # Plot the data for each attached sensor
    for count in range(4*numBoards):
        if SensorNumAttached[count]:
            positions = [temptime[count], 
                        tempPos[0, count]*25.4, 
                        tempPos[1, count]*25.4, 
                        tempPos[2, count]*25.4, 
                        tempAng[0, count], 
                        tempAng[1, count], 
                        tempAng[2, count], 
                        tempQuality[count]]
            
    return positions
    #print(["{:10.3f}".format(num) for num in positions])