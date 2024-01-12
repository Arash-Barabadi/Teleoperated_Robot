import serial
import serial.tools.list_ports

from decoder import *
from mtlt305_ros.RS232_packets import *
from mtlt305_ros.MTLT305 import MTLT305

def main_com_port():
    ## Not Tested yet if unsure about autodetection use code below to set specific port
    # ports = serial.tools.list_ports.comports()
    # found == False
    # for port in ports:
    #     try:
    #         mtlt305 = serial.Serial(port, 
    #                                 baudrate=230400, 
    #                                 timeout=4)
    #         mtlt305.flush()
    #         mtlt305.write(Ping_Command.ping_cmd)
    #         found = (mtlt305.read(7) == Ping_Response.ping_resp)
    #     except Exception as e:
    #         print(e)
    #
    # if not found:
    #     return

    mtlt305 = serial.Serial("/dev/ttyUSB0", 
                            baudrate=57600, 
                            timeout=4)
    mtlt305.flush()
    circ_buf = QUEUE_TYPE()
    result = ACEINNA_PACKET()
    while True:
        try:
            byt = mtlt305.read(1)[0]
            retval, circ_buf = AddQueue(byt, circ_buf)
            if not retval:
                raise MemoryError("circular buffer only holds 50 bytes")
            retval, packet = process_aceinna_packet(circ_buf, result)
            if retval:
                for key in Messages.keys():
                    if packet.packettype == key:
                        if Messages[key]["Data"](packet.data) is not None:
                            res = Messages[key]["Data"](packet.data)
                            if key == ANGLE_2_MSG:
                                print(res)
        except KeyboardInterrupt:
            break
    pass


def main_bytestream():
    hex_str  = '555541321e001c000afee6000300000000fffffffcf341206a206a206a000b529800008a25'
    hex_str += '555541321e001c000afee6000000010000fffefffaf340206a206a206a000b52a200007140'
    hex_str += '555541321e001c000afee6000000020000fffefff9f33f206a206a206a000b52ac00004701'
    bits = bytes.fromhex(hex_str)
    #print(f"{bits[0]:2x}")

    circ_buf = QUEUE_TYPE()
    result = ACEINNA_PACKET()
    
    for byt in bits:
        retval, circ_buf = AddQueue(byt, circ_buf)
        if not retval:
            raise MemoryError("circular buffer only holds 50 bytes")
        retval, packet = process_aceinna_packet(circ_buf, result)
        if retval:
            print(f"{packet.packettype:4x}")
            for key in Messages.keys():
                if packet.packettype == key:
                    if Messages[key]["Data"](packet.data) is not None:
                        res = Messages[key]["Data"](packet.data)
                        if key == ANGLE_2_MSG:
                            print(res)
    print("finsihed!!!")

def main_playbackfile():
    circ_buf = QUEUE_TYPE()
    result = ACEINNA_PACKET()

    with open("test_RAW.logimu.txt", "r") as fil:
        while fil.readline() != "\n":
            pass
        fil.readline()
        hex_str = ""
        for lin in fil.readlines():
            li = lin.split("\n")[0].split("\t")[-1].split(" ")
            for l in li:
                hex_str = hex_str + l
    bits = bytes.fromhex(hex_str)
    for i, byt in enumerate(bits):
        #print(i, f"{byt:2x}")
        retval, circ_buf = AddQueue(byt, circ_buf)
        if not retval:
            raise MemoryError("circular buffer only holds 50 bytes")
        retval, packet = process_aceinna_packet(circ_buf, result)
        if retval:
            #print(f"{packet.packettype:4x}")
            for key in Messages.keys():
                if packet.packettype == key:
                    if Messages[key]["Data"](packet.data) is not None:
                        res = Messages[key]["Data"](packet.data)
                        if key == ANGLE_2_MSG:
                            print(res)
                            pass
        # if i == 37:
        #     break
    print("finsihed!!!")


def main_class():
    a = MTLT305(port="/dev/ttyUSB0")
    if a.port is not None:
        while True:
            try:
                #a.get_packet_request("A2")
                a.read_packet_response()
            except KeyboardInterrupt:
                break
    else:
        print("MTLT305 not found")
    print("finished!!!")


if __name__ == "__main__":
    # main_com_port()
    # main_bytestream()
    # main_playbackfile()
    main_class()