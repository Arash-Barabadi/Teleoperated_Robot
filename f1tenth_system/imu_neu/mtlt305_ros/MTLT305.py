import time
import struct
import dataclasses

from serial import Serial
import serial.tools.list_ports
import numpy as np

from mtlt305_ros.RS232_packets import Angle2_Message, T0_Message, VR_Message


@dataclasses.dataclass
class QUEUE_TYPE:
    count=0
    front=0
    rear=-1
    entry=np.zeros((256,1), dtype=np.uint8)


@dataclasses.dataclass
class ACEINNA_PACKET:
    packettype=0
    length=0
    crc=0
    data=np.zeros((256,1), dtype=np.uint8)


class MTLT305():
    MAXQUEUE = 50
    
    def __init__(self, port=None, baudrate=57600) -> None:
        self.baudrate = baudrate

        self.circ_buf = QUEUE_TYPE()
        self.result = ACEINNA_PACKET()
        self.received_ping = False
        self.received_echo = False
        self.received_id   = False
        self.received_vr   = False
        self.received_t0   = False
        self.received_ang2 = False
        self.received_err  = False
        self.received_ar   = False

        self.echo_resp = None
        self.id = ""
        self.VR = VR_Message(np.zeros((256,1), dtype=np.uint8))
        self.t0 = T0_Message(np.zeros((256,1), dtype=np.uint8))
        self.a2 = Angle2_Message(np.zeros((256, 1), dtype=np.uint8))
        
        self.ping_cmd = struct.pack(">HHBH", 0x5555, 0x504B, 0x00, 0x9EF4)
        #self.echo_cmd = self.construct_echo_cmd()
        if port is None:
            self.port = self.__detect_port()
        else:
            try:
                self.port = serial.Serial(port=port, baudrate=self.baudrate, timeout=4)
                self.port.flush()
                self.port.write(self.ping_cmd)
                print(self.ping_cmd)
                time.sleep(0.03)
                self.read_packet_response()
                print(f"self.received_ping = {self.received_ping} \n" +
                      f"self.received_echo = {self.received_echo} \n" +
                      f"self.received_id   = {self.received_id} \n" +
                      f"self.received_vr   = {self.received_vr} \n" +
                      f"self.received_t0   = {self.received_t0} \n" +
                      f"self.received_ang2 = {self.received_ang2} \n" +
                      f"self.received_err  = {self.received_err} \n" +
                      f"self.received_ar   = {self.received_ar} \n")
                if not self.received_ping:
                    self.port.close()
                    self.port = None
            except serial.SerialException:
                self.port = None
        pass

    def __detect_port(self) -> Serial:
        ports = serial.tools.list_ports.comports()
        found = False
        for port in ports:
            try:
                mtlt305 = serial.Serial(port, 
                                        baudrate=self.baudrate, 
                                        timeout=4)
                mtlt305.flush()
                mtlt305.write(self.ping_cmd)
                found = (mtlt305.read(7) == self.ping_cmd)
                if found:
                    return mtlt305
                else:
                    mtlt305.close()
            except Exception as e:
                print(e)
        
        return None
    
    def calcCRC_bytes(self, byteStream: bytes) -> np.uint16:
        #print(num, startIndex, self.circ_buf)
        crc = np.uint16(0x1D0F)
        #print(f"{crc:04x}")

        for byt in byteStream:
            crc = np.bitwise_xor(crc, np.left_shift(np.uint16(byt),8)).astype(np.uint16)
            crc = np.bitwise_and(crc, 0xFFFF)
            for j in range(8):
                if np.bitwise_and(crc, 0x8000):
                    crc = np.bitwise_xor(np.left_shift(crc,1), np.uint16(0x1021)).astype(np.uint16)
                else:
                    crc = np.left_shift(crc,1).astype(np.uint16)
        return crc
    

    def calcCRC(self, startIndex: int, num: int) -> np.uint16:
        #print(num, startIndex, queue_ptr)
        crc = np.uint16(0x1D0F)
        #print(f"{crc:04x}")

        for i in range(num):
            #print(f"peek {peekByte(queue_ptr, startIndex+i):02x}")
            crc = np.bitwise_xor(crc, np.left_shift(np.uint16(self.peekByte(startIndex+i)),8)).astype(np.uint16)
            crc = np.bitwise_and(crc, 0xFFFF)
            #print(f"{crc:04x}")
            for j in range(8):
                if np.bitwise_and(crc, 0x8000):
                    #print("hello")
                    crc = np.bitwise_xor(np.left_shift(crc,1), np.uint16(0x1021)).astype(np.uint16)
                else:
                    crc = np.left_shift(crc,1).astype(np.uint16)    
        return crc # np.bitwise_and(crc, 0xFFFF)
    
    
    def AddQueue(self, item: np.uint8):
        retval = 0
        if self.circ_buf.count >= self.MAXQUEUE:
            retval = 0  # queue is full
        else:
            self.circ_buf.count += 1
            self.circ_buf.rear = (self.circ_buf.rear + 1) % self.MAXQUEUE
            self.circ_buf.entry[self.circ_buf.rear] = item
            retval = 1
        return retval
    

    def DeleteQueue(self):
        retval = 0
        if self.circ_buf.count <= 0:
            retval = 0  # queue is empty
        else:
            self.circ_buf.count -= 1
            item = self.circ_buf.entry[self.circ_buf.front]
            self.circ_buf.front = (self.circ_buf.front + 1) % self.MAXQUEUE
        return retval, item
    
    
    def peekByte(self, index: int):
        firstIndex = (self.circ_buf.front + index) % self.MAXQUEUE
        byte = self.circ_buf.entry[firstIndex]
        return byte[0]


    # *******************************************************************************
    # * FUNCTION:   peekWord returns 2-byte word from buffer without popping
    # * ARGUMENTS:  queue_ptr is pointer to the queue to return word from
    # *             index is offset into buffer to which word to return
    # * RETURNS:    2-byte word
    # * REMARKS:    does not do boundary checking. please do this first
    # *******************************************************************************

    def peekWord(self, index: int):
        firstIndex = (self.circ_buf.front + index) % self.MAXQUEUE
        secondIndex = (self.circ_buf.front + index + 1) % self.MAXQUEUE
        word = np.left_shift(self.circ_buf.entry[firstIndex].astype(np.uint16), 8)
        word = np.bitwise_or(word,self.circ_buf.entry[secondIndex].astype(np.uint16))
        return word


    # *******************************************************************************
    # * FUNCTION:   Pop - discard item(s) from queue
    # * ARGUMENTS:  queue_ptr is pointer to the queue
    # *             numToPop is number of items to discard
    # * RETURNS:    return the number of items discarded
    # *******************************************************************************

    def Pop(self, numToPop: int):
        i=0
        for i in range(numToPop):
            if not self.DeleteQueue():
                break
        return i


    # *******************************************************************************
    # * FUNCTION:   Size
    # * ARGUMENTS:  queue_ptr is pointer to the queue
    # * RETURNS:    return the number of items in the queue
    # *******************************************************************************

    def Size(self):
        return self.circ_buf.count


    # *******************************************************************************
    # * FUNCTION:   Empty
    # * ARGUMENTS:  queue_ptr is pointer to the queue
    # * RETURNS:    return 1 if empty, 0 if not
    # *******************************************************************************

    def Empty(self):
        return self.circ_buf.count <= 0


    # *******************************************************************************
    # * FUNCTION:   Full
    # * ARGUMENTS:  queue_ptr is pointer to the queue
    # * RETURNS:    return 1 if full, 0 if not full
    # *******************************************************************************

    def Full(self):
        return self.circ_buf.count >= self.MAXQUEUE


    def get_packet_request(self, packet="ID") -> None:
        cmd = None
        gp = struct.Struct(">HBH")
        gp_crc = struct.Struct(">HHBHH")
        if packet == "ID":
            cmd = gp.pack(0x4750, 0x2, 0x4944)
            crc = self.calcCRC_bytes(cmd)
            cmd = gp_crc.pack(0x5555, 0x4750, 0x2, 0x4944, crc)
        elif packet == "VR":
            cmd = gp.pack(0x4750, 0x2, 0x5652)
            crc = self.calcCRC_bytes(cmd)
            cmd = gp_crc.pack(0x5555, 0x4750, 0x2, 0x5652, crc)
        elif packet == "T0":
            cmd = gp.pack(0x4750, 0x2, 0x5430)
            crc = self.calcCRC_bytes(cmd)
            cmd = gp_crc.pack(0x5555, 0x4750, 0x2, 0x5430, crc)
        elif packet == "A2":
            cmd = gp.pack(0x4750, 0x2, 0x4132)
            crc = self.calcCRC_bytes(cmd)
            cmd = gp_crc.pack(0x5555, 0x4750, 0x2, 0x4132, crc)
        
        if cmd is not None:
            self.port.write(cmd)
    

    def send_AlgoReset(self) -> None:
        ar = struct.Struct(">HB")
        ar_crc = struct.Struct(">HHBH")
        crc = self.calcCRC_bytes(ar.pack(0x4152, 0x00))
        cmd = ar_crc.pack(0x5555, 0x4152, 0x00, crc)

        self.port.write(cmd)

    
    def send_EchoCmd(self, echo_data:bytes = None) -> None:
        if echo_data is None:
            echo_data = np.random.random(30)*255
            echo_data = echo_data.astype(np.uint8).tobytes()

        ch = struct.Struct(f">H{len(echo_data)}s")
        ch_crc = struct.Struct(f">HH{len(echo_data)}sH")
        crc = self.calcCRC_bytes(ch.pack(0x4348, echo_data))
        cmd = ch_crc.pack(0x5555, 0x4348, echo_data, crc)

        self.port.write(cmd)


    def read_packet_response(self) -> None:
        self.received_ping = False
        self.received_echo = False
        self.received_ar   = False
        self.received_err  = False
        self.received_id   = False
        self.received_vr   = False
        self.received_t0   = False
        self.received_ang2 = False
        
        byts = self.port.read_all()
        for byt in byts:
            n_err = self.AddQueue(byt)
            if n_err:
                n_err, packet = self.process_aceinna_packet()
                if n_err:
                    if packet.packettype == 0x504B:
                        self.received_ping = True
                        print("ping response")
                        pass
                    elif packet.packettype == 0x4348:
                        self.received_echo = True
                        self.echo_resp = packet.data[:packet.length]
                        print("got echo response")
                        pass
                    elif packet.packettype == 0x4152:
                        self.received_ar   = True
                        print("Algo resetted")
                        pass
                    elif (packet.packettype == 0x1515) or (packet.packettype == 0x0000):
                        self.received_err  = True
                        print("Error Message Received")
                        pass
                    elif packet.packettype == 0x4944:
                        self.received_id   = True
                        id = ""
                        print(packet.data)
                        for c in packet.data:
                            id = id + chr(int(c))
                        print("received ID: ", id)
                    elif packet.packettype == 0x5652:
                        self.received_vr   = True
                        print(f"Version Number: {packet.data[0]}.{packet.data[1]}.{packet.data[2]}-{packet.data[3]}.{packet.data[4]}")
                    elif packet.packettype == 0x5430:
                        self.received_t0   = True
                        self.t0 = T0_Message(packet.data[:packet.length])
                        print(self.t0)
                    elif packet.packettype == 0x4132:
                        self.received_ang2 = True
                        self.a2 = Angle2_Message(packet.data[:packet.length])
                        print(self.a2)
                    else:
                        print("not (yet) supported; maybe next version")


    def process_aceinna_packet(self) -> (int, ACEINNA_PACKET):
        myCRC=0
        packetCRC=0
        numToPop=0
        counter=0
        dataLength=0
        result = ACEINNA_PACKET()

        if self.Empty():
            return 0, None  # empty buffer
        
        ## find header
        for numToPop in range(self.Size()):
            if 0x5555 == self.peekWord(numToPop):
                break

        self.Pop(numToPop)

        if self.Size() <= 0:
            #print("empty Queue")
            return 0, None  # header was not found
        
        ## make sure we can read through minimum length packet

        if self.Size() < 7:
            #print("too less bytes read")
            return 0, None
        
        ## get data length (5th byte of packet)
        dataLength = self.peekByte(4)

        if self.Size() < 7+dataLength:
            #print("too less data bytes received")
            return 0, None  # not received whole packet
        
        ## check CRC
        myCRC = self.calcCRC(2, int(dataLength+3))
        packetCRC = self.peekWord(dataLength+5)[0]
        #print(myCRC, packetCRC)

        if myCRC != packetCRC:
            # bad CRC on Packet - remove the bad packet from the queue and return
            print(f"bad CRC detected: got 0x{packetCRC:4x}, but calculated 0x{myCRC:4x}")
            self.Pop(int(dataLength+7))
            return 0, None
        
        ## fill out result of parsing in structure
        result.packettype = self.peekWord(2)[0]
        result.length = self.peekByte(4)
        #print(result.length)
        result.crc = packetCRC
        for counter in range(int(result.length)):
            result.data[counter] = self.peekByte(5+counter)

        self.Pop(int(dataLength+7))

        return 1, result
    