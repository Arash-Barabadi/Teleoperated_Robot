import dataclasses

import numpy as np

MAXQUEUE = 50

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


def process_aceinna_packet(queue_ptr: QUEUE_TYPE, result: ACEINNA_PACKET):
    myCRC=0
    packetCRC=0
    numToPop=0
    counter=0
    dataLength=0

    if Empty(queue_ptr):
        return 0, None  # empty buffer
    
    ## find header
    for numToPop in range(Size(queue_ptr)):
        if 0x5555 == peekWord(queue_ptr, numToPop):
            break

    Pop(queue_ptr, numToPop)

    if Size(queue_ptr) <= 0:
        #print("empty Queue")
        return 0, None  # header was not found
    
    ## make sure we can read through minimum length packet

    if Size(queue_ptr) < 7:
        #print("too less bytes read")
        return 0, None
    
    ## get data length (5th byte of packet)
    dataLength = peekByte(queue_ptr, 4)

    if Size(queue_ptr) < 7+dataLength:
        #print("too less data bytes received")
        return 0, None  # not received whole packet
    
    ## check CRC
    myCRC = calcCRC(queue_ptr, 2, int(dataLength+3))
    packetCRC = peekWord(queue_ptr, dataLength+5)[0]
    #print(myCRC, packetCRC)

    if myCRC != packetCRC:
        # bad CRC on Packet - remove the bad packet from the queue and return
        print(f"bad CRC detected: got 0x{packetCRC:4x}, but calculated 0x{myCRC:4x}")
        Pop(queue_ptr, int(dataLength+7))
        return 0, None
    
    ## fill out result of parsing in structure
    result.packettype = peekWord(queue_ptr, 2)[0]
    result.length = peekByte(queue_ptr, 4)
    #print(result.length)
    result.crc = packetCRC
    for counter in range(int(result.length)):
        result.data[counter] = peekByte(queue_ptr, 5+counter)

    Pop(queue_ptr, int(dataLength+7))

    return 1, result

# *******************************************************************************
# * FUNCTION:   calcCRC calculates a 2-byte CRC on serial data using
# *             CRC-CCITT 16-bit standard maintained by the ITU
# *             (International Telecommunications Union).
# * ARGUMENTS:  queue_ptr is pointer to queue holding area to be CRCed
# *             startIndex is offset into buffer where to begin CRC calculation
# *             num is offset into buffer where to stop CRC calculation
# * RETURNS:    2-byte CRC
# *******************************************************************************

def calcCRC(queue_ptr: QUEUE_TYPE, startIndex: int, num: int):
    #print(num, startIndex, queue_ptr)
    crc = np.uint16(0x1D0F)
    #print(f"{crc:04x}")

    for i in range(num):
        #print(f"peek {peekByte(queue_ptr, startIndex+i):02x}")
        crc = np.bitwise_xor(crc, np.left_shift(np.uint16(peekByte(queue_ptr, startIndex+i)),8)).astype(np.uint16)
        crc = np.bitwise_and(crc, 0xFFFF)
        #print(f"{crc:04x}")
        for j in range(8):
            if np.bitwise_and(crc, 0x8000):
                #print("hello")
                crc = np.bitwise_xor(np.left_shift(crc,1), np.uint16(0x1021)).astype(np.uint16)
            else:
                crc = np.left_shift(crc,1).astype(np.uint16)
    
    return crc # np.bitwise_and(crc, 0xFFFF)
    

# *******************************************************************************
# * FUNCTION:   AddQueue - add item in front of queue
# * ARGUMENTS:  item holds item to be added to queue
# *             queue_ptr is pointer to the queue
# * RETURNS:    returns 0 if queue is full. 1 if successful
# *******************************************************************************

def AddQueue(item: np.uint8, queue_ptr: QUEUE_TYPE):
    retval = 0
    if queue_ptr.count >= MAXQUEUE:
        retval = 0  # queue is full
    else:
        queue_ptr.count += 1
        queue_ptr.rear = (queue_ptr.rear + 1) % MAXQUEUE
        queue_ptr.entry[queue_ptr.rear] = item
        retval = 1

    return retval, queue_ptr


# *******************************************************************************
# * FUNCTION:   DeleteQeue - return an item from the queue
# * ARGUMENTS:  item will hold item popped from queue
# *             queue_ptr is pointer to the queue
# * RETURNS:    returns 0 if queue is empty. 1 if successful
# *******************************************************************************

def DeleteQueue(queue_ptr: QUEUE_TYPE):
    retval = 0
    if queue_ptr.count <= 0:
        retval = 0  # queue is empty
    else:
        queue_ptr.count -= 1
        item = queue_ptr.entry[queue_ptr.front]
        queue_ptr.front = (queue_ptr.front + 1) % MAXQUEUE
    
    return retval, item


# *******************************************************************************
# * FUNCTION:   peekByte returns 1 byte from buffer without popping
# * ARGUMENTS:  queue_ptr is pointer to the queue to return byte from
# *             index is offset into buffer to which byte to return
# * RETURNS:    1 byte
# * REMARKS:    does not do boundary checking. please do this first
# *******************************************************************************

def peekByte(queue_ptr: QUEUE_TYPE, index: int):
    firstIndex = (queue_ptr.front + index) % MAXQUEUE
    byte = queue_ptr.entry[firstIndex]
    return byte[0]


# *******************************************************************************
# * FUNCTION:   peekWord returns 2-byte word from buffer without popping
# * ARGUMENTS:  queue_ptr is pointer to the queue to return word from
# *             index is offset into buffer to which word to return
# * RETURNS:    2-byte word
# * REMARKS:    does not do boundary checking. please do this first
# *******************************************************************************

def peekWord(queue_ptr: QUEUE_TYPE, index: int):
    firstIndex = (queue_ptr.front + index) % MAXQUEUE
    secondIndex = (queue_ptr.front + index + 1) % MAXQUEUE
    # print(f"1st index: {firstIndex}; 2nd index: {secondIndex}")
    # print(f"1st value: {queue_ptr.entry[firstIndex]}; 2nd value: {queue_ptr.entry[secondIndex]}")
    word = np.left_shift(queue_ptr.entry[firstIndex].astype(np.uint16), 8)
    word = np.bitwise_or(word,queue_ptr.entry[secondIndex].astype(np.uint16))
    return word


# *******************************************************************************
# * FUNCTION:   Pop - discard item(s) from queue
# * ARGUMENTS:  queue_ptr is pointer to the queue
# *             numToPop is number of items to discard
# * RETURNS:    return the number of items discarded
# *******************************************************************************

def Pop(queue_ptr: QUEUE_TYPE, numToPop: int):
    i=0
    for i in range(numToPop):
        if not DeleteQueue(queue_ptr):
            break
    return i


# *******************************************************************************
# * FUNCTION:   Size
# * ARGUMENTS:  queue_ptr is pointer to the queue
# * RETURNS:    return the number of items in the queue
# *******************************************************************************

def Size(queue_ptr: QUEUE_TYPE):
    return queue_ptr.count


# *******************************************************************************
# * FUNCTION:   Empty
# * ARGUMENTS:  queue_ptr is pointer to the queue
# * RETURNS:    return 1 if empty, 0 if not
# *******************************************************************************

def Empty(queue_ptr: QUEUE_TYPE):
    return queue_ptr.count <= 0


# *******************************************************************************
# * FUNCTION:   Full
# * ARGUMENTS:  queue_ptr is pointer to the queue
# * RETURNS:    return 1 if full, 0 if not full
# *******************************************************************************

def Full(queue_ptr: QUEUE_TYPE):
    return queue_ptr.count >= MAXQUEUE
