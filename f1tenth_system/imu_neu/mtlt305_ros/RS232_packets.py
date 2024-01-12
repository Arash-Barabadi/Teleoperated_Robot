import struct

# Link Test
PING_CMD = 0x504B
ECHO_CMD = 0x4348

# interactive Commands
GET_PKG_RQ = 0x4750
ALGO_RST   = 0x4152
ERROR_RSP  = 0x1515

# Output Messages: Status & other, (Polled Only)
ID_DATA   = 0x4944
VR_DATA   = 0x5652
TEST_DATA = 0x5430

# Output Messages: Measurement Data (Continuous or Polled)
ANGLE_2_MSG = 0x4132

# Advanced Commands
WRITE_FIELDS_RQ = 0x5746
WRITE_FIELDS_RSP = 0x5746
SET_FIELDS_RQ = 0x5346
SET_FIELDS_RSP = 0x5346
READ_FIELDS_RQ = 0x5246
READ_FIELDS_RSP = 0x5246
GET_FIELDS_RQ = 0x4746
GET_FIELDS_RSP = 0x4746


class Angle2_Message():
    ang2_msg = struct.Struct(">hhhhhhhhhhhhIH")
    size = ang2_msg.size

    def __init__(self, byteStream: bytes):
        a = self.ang2_msg.unpack(byteStream[:self.size])
        #print(a)
        # shorts = np.frombuffer(byteStream, dtype=np.int16)
        self.angle_conversion = 360/(2**16)  # 2*np.pi/(2**16)
        self.rate_conversion = 1260/(2**16)  # 7*np.pi/(2**16)
        self.accel_conversion = 20/(2**16)
        self.temp_conversion = 200/(2**16)

        self.rollAngle = a[0] * self.angle_conversion
        self.pitchAngle = a[1] * self.angle_conversion
        self.yawAngleTrue = a[2] * self.angle_conversion
        self.rollRate = a[3] * self.rate_conversion
        self.pitchRate = a[4] * self.rate_conversion
        self.yawRate = a[5] * self.rate_conversion
        self.x_accel = a[6] * self.accel_conversion
        self.y_accel = a[7] * self.accel_conversion
        self.z_accel = a[8] * self.accel_conversion
        self.rollrate_temp = a[9] * self.temp_conversion
        self.pitchrate_temp = a[10] * self.temp_conversion
        self.yawrate_temp = a[11] * self.temp_conversion
        self.timeITOW = a[12]
        self.status = a[13]

    def __str__(self) -> str:
        strin = f"********  ANGLE2 MESSAGE of MTLT305 @t={self.timeITOW/1000:.3f}s ******************** \n"
        strin += f"pitchAngle  [°]: {self.pitchAngle:0.5f} \t rollAngle  [°]: {self.rollAngle:0.5f} \t yawAngle  [°]: {self.yawAngleTrue:0.5f}\n"
        strin += f"pitchRate [°/s]: {self.pitchRate:0.5f} \t rollRate [°/s]: {self.rollRate:0.5f} \t yawRate [°/s]: {self.yawRate:0.5f}\n"
        strin += f"x-Accel     [g]: {self.x_accel:0.5f} \t y-Accel     [g]: {self.y_accel:0.5f} \t z-Accel     [g]: {self.z_accel:0.5f}\n"
        strin += f"xRateTemp  [°C]: {self.rollrate_temp:.5f} \t yRateTemp  [°C]: {self.pitchrate_temp:.5f} \t zRateTemp  [°C]: {self.yawrate_temp:.5f}\n"
        return strin


class Ping_Command():
    ping_cmd = struct.pack(">HHBH", 0x5555, 0x504B, 0x00, 0x9EF4)

class Ping_Response():
    ping_rsp = struct.pack(">HHBH", 0x5555, 0x504B, 0x00, 0x9EF4)

class T0_Message():
    t0_msg = struct.Struct(">HHHHHHHHHHHHHH")
    size = t0_msg.size

    def __init__(self, byteStream: bytes) -> None:
        a = self.t0_msg.unpack(byteStream[:self.size])

        self.BITstatus = a[0]
        self.hardwareBIT = a[1]
        self.hardwarePowerBIT = a[2]
        self.hardwareEnvironmentBIT = a[3]
        self.comBIT = a[4]
        self.comSerialABIT = a[5]
        self.comSerialBBIT = a[6]
        self.softwareBIT = a[7]
        self.softwareAlgorithmBIT = a[8]
        self.softwareDataBIT = a[9]
        self.hardwareStatus = a[10]
        self.comStatus = a[11]
        self.softwareStatus = a[12]
        self.sensorStatus = a[13]

    def __str__(self) -> str:
        strin = f"********  T0 MESSAGE of MTLT305  ******************** \n"
        strin += f"BITstatus  : {self.BITstatus} \t hardwareBIT  : {self.hardwareBIT} \t hardwarePowerBIT  : {self.hardwarePowerBIT}\n"
        strin += f"hardwareEnvironmentBIT : {self.hardwareEnvironmentBIT} \t comBIT : {self.comBIT} \t comSerialABIT : {self.comSerialABIT}\n"
        strin += f"comSerialBBIT     : {self.comSerialBBIT} \t softwareBIT     : {self.softwareBIT} \t softwareAlgorithmBIT     : {self.softwareAlgorithmBIT}\n"
        strin += f"softwareDataBIT  : {self.softwareDataBIT} \t hardwareStatus  : {self.hardwareStatus} \t comStatus  : {self.comStatus}\n"
        strin += f"softwareStatus  :{self.softwareStatus} \t  sensorStatus : {self.sensorStatus} \n"
        return strin


class VR_Message():
    vr_msg = struct.Struct(">BBBBB")
    size = vr_msg.size
    dev_stage = {0: "release",
                 1: "development",
                 2: "alpha",
                 3: "beta",
                 4: "unknown"}

    def __init__(self, byteStream: bytes) -> None:
        a = self.vr_msg.unpack(byteStream[:self.size])

        self.majorVersion = a[0]
        self.minorVersion = a[1]
        self.patch        = a[2]
        if a[3] < 4:
            self.stage = a[3]
        else:
            self.stage = 4
        self.buildNumber  = a[4]
    
    def __str__(self) -> str:
        strin = f"Version: {self.majorVersion}.{self.minorVersion}.{self.patch} - {self.dev_stage[self.stage]}.{self.buildNumber} \n"
        return strin
    

Messages = {PING_CMD: {"length": 0, "Data": None},
            ECHO_CMD: {"length": None, "Data": None},
            GET_PKG_RQ: {"length": 2, "Data": None},
            ALGO_RST: {"length": 0, "Data": None},
            ERROR_RSP: {"length": 2, "Data": None},
            ID_DATA: {"length": 5, "Data": None},
            VR_DATA: {"length": 5, "Data": VR_Message},
            TEST_DATA: {"length": 28, "Data": T0_Message},
            ANGLE_2_MSG: {"length": 30, "Data": Angle2_Message},
            WRITE_FIELDS_RQ: {"length": 1, "Data": None},
            WRITE_FIELDS_RSP: {"length": 1, "Data": None},
            SET_FIELDS_RQ: {"length": 1, "Data": None},
            SET_FIELDS_RSP: {"length": 1, "Data": None},
            READ_FIELDS_RQ: {"length": 1, "Data": None},
            READ_FIELDS_RSP: {"length": 1, "Data": None},
            GET_FIELDS_RQ: {"length": 1, "Data": None},
            GET_FIELDS_RSP: {"length": 1, "Data": None}
            }
