#===============================================================================
# File      : host.py
# Brief     : Host application to communicate with the STM32 bootloader
# Author    : Kyungjae Lee
# Date      : Jul 23, 2023
#
# Note      : A binary file must be present in the same folder for the binary
#             flashing (BL_WRITE_MEM) feature to work. Otherwise, modify the
#             binary file path defined in the "File ops" section in the code.
#===============================================================================

import serial	# pip (or pip3) install pyserial
import struct
import os
import sys
import glob

# Flash memory status
Flash_HAL_OK			= 0x00
Flash_HAL_ERROR			= 0x01
Flash_HAL_BUSY			= 0x02
Flash_HAL_TIMEOUT		= 0x03
Flash_HAL_INV_ADDR		= 0x04
Flash_INVALID_NUM_OF_SECTORS = 0x04

# Bootloader commands
BL_GET_VER				= 0x51
BL_GET_HELP				= 0x52
BL_GET_CID				= 0x53
BL_GO_TO_ADDR			= 0x54
BL_ERASE_FLASH			= 0x55
BL_READ_MEM				= 0x56	# TODO
BL_WRITE_MEM			= 0x57
BL_GET_RDP_LEVEL		= 0x58
BL_SET_RDP_LEVEL		= 0x59
BL_ENABLE_WRP			= 0x5A
BL_DISABLE_WRP			= 0x5B
BL_GET_WRP_STATUS		= 0x5C
BL_READ_OTP				= 0x5D

# Bootloader command lengths
BL_GET_VER_LEN			= 6
BL_GET_HELP_LEN			= 6
BL_GET_CID_LEN			= 6
BL_GO_TO_ADDR_LEN		= 10
BL_ERASE_FLASH_LEN		= 8
BL_READ_MEM_LEN			= 11
BL_WRITE_MEM_LEN 		= 11
BL_GET_RDP_LEVEL_LEN	= 6
BL_SET_RDP_LEVEL_LEN	= 7
BL_ENABLE_WRP_LEN		= 8
BL_DISABLE_WRP_LEN		= 6
BL_GET_WRP_STATUS_LEN	= 6
#BL_READ_OTP_LEN			= 6

verbose_mode = 1
mem_write_active =0

#----------------------------- File ops-----------------------------------------

def calc_file_len():
    size = os.path.getsize("stm32f407xx_user_application.bin")
    return size

def open_the_file():
    global bin_file
    bin_file = open('stm32f407xx_user_application.bin','rb')
    #read = bin_file.read()
    #global file_contents = bytearray(read)

def read_the_file():
    pass

def close_the_file():
    bin_file.close()

#----------------------------- Utilities----------------------------------------

def word_to_byte(addr, index , lowerfirst):
    value = (addr >> (8 * (index -1)) & 0x000000FF)
    return value

def get_crc(buff, length):
    crc = 0xFFFFFFFF
    #print(length)
    for data in buff[0:length]:
        crc = crc ^ data
        for i in range(32):
            if(crc & 0x80000000):
                crc = (crc << 1) ^ 0x04C11DB7
            else:
                crc = (crc << 1)
    return crc

#----------------------------- Serial port -------------------------------------

def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

def Serial_Port_Configuration(port):
    global ser
    try:
        ser = serial.Serial(port, 115200, timeout = 2)
    except:
        print("\n   Oops! That was not a valid port")
        
        port = serial_ports()
        if(not port):
            print("\n   No ports Detected")
        else:
            print("\n   Here are some available ports on your PC. Try Again!")
            print("\n   ", port)
        return -1
    if ser.is_open:
        print("\n   Port Open Success")
    else:
        print("\n   Port Open Failed")
    return 0


def read_serial_port(length):
    """ Reads a specified number of bytes from a serial port and
        returns the read data
    """
    read_value = ser.read(length)
    return read_value

def Close_serial_port():
    pass

def purge_serial_port():
    """ Purges or clears the input buffer of a serial port
    """
    ser.reset_input_buffer()
    
def Write_to_serial_port(value, *length):
        data = struct.pack('>B', value)
        if (verbose_mode):
            value = bytearray(data)
            #print("   "+hex(value[0]), end='')
            print("   "+"0x{:02x}".format(value[0]), end = ' ')
        if(mem_write_active and (not verbose_mode)):
                print("#",end=' ')
        ser.write(data)
        
#---------------------------- Command processing--------------------------------

# 0x51
def process_BL_GET_VER(length):
    ver = read_serial_port(1)
    value = bytearray(ver)
    print("\n   Bootloader Ver. : ", hex(value[0]))

# 0x52
def process_BL_GET_HELP(length):
    #print("reading:", length)
    value = read_serial_port(length) 
    reply = bytearray(value)
    print("\n   Supported Commands :", end = ' ')
    for x in reply:
        print(hex(x), end = ' ')
    print()

# 0x53
def process_BL_GET_CID(length):
    value = read_serial_port(length)
    ci = (value[1] << 8) + value[0]
    print("\n   Chip ID : ", hex(ci))

# 0x54
def process_BL_GO_TO_ADDR(length):
    addr_status = 0
    value = read_serial_port(length)
    addr_status = bytearray(value)
    print("\n   Address Status : ", hex(addr_status[0]))

# 0x55
def process_BL_ERASE_FLASH(length):
    erase_status = 0
    value = read_serial_port(length)
    if len(value):
        erase_status = bytearray(value)
        if(erase_status[0] == Flash_HAL_OK):
            print("\n   Erase Status: Success  Code: FLASH_HAL_OK")
        elif(erase_status[0] == Flash_HAL_ERROR):
            print("\n   Erase Status: Fail  Code: FLASH_HAL_ERROR")
        elif(erase_status[0] == Flash_HAL_BUSY):
            print("\n   Erase Status: Fail  Code: FLASH_HAL_BUSY")
        elif(erase_status[0] == Flash_HAL_TIMEOUT):
            print("\n   Erase Status: Fail  Code: FLASH_HAL_TIMEOUT")
        elif(erase_status[0] == Flash_INVALID_NUM_OF_SECTORS):
            print("\n   Erase Status: Fail  Code: FLASH_INVALID_NUM_OF_SECTORS")
        else:
            print("\n   Erase Status: Fail  Code: UNKNOWN_ERROR_CODE")
    else:
        print("Timeout: Bootloader is not responding")

# TODO: 0x56
def process_BL_READ_MEM(length):
    pass

# 0x57
def process_BL_WRITE_MEM(length):
    write_status = 0
    value = read_serial_port(length)
    write_status = bytearray(value)
    if(write_status[0] == Flash_HAL_OK):
        print("\n   Write Status: FLASH_HAL_OK")
    elif(write_status[0] == Flash_HAL_ERROR):
        print("\n   Write Status: FLASH_HAL_ERROR")
    elif(write_status[0] == Flash_HAL_BUSY):
        print("\n   Write Status: FLASH_HAL_BUSY")
    elif(write_status[0] == Flash_HAL_TIMEOUT):
        print("\n   Write Status: FLASH_HAL_TIMEOUT")
    elif(write_status[0] == Flash_HAL_INV_ADDR):
        print("\n   Write Status: FLASH_HAL_INV_ADDR")
    else:
        print("\n   Write Status: UNKNOWN_ERROR_CODE")
    print("\n")
    
# TODO: Delete this function since it is not necessary for STM32F407xx MCU
protection_mode= [ "Write Protection", "Read/Write Protection","No protection" ]
def protection_type(status,n):
    if(status & (1 << n)):
        return protection_mode[2]
    else:
        return protection_mode[0]

# 0x58
def process_BL_GET_RDP_LEVEL(length):
    value = read_serial_port(length)
    rdp = bytearray(value)
    print("\n   RDP Level: ", hex(rdp[0]))

# TODO: 0x59 Rewrite the following function
def process_BL_SET_RDP_LEVEL(length):
    status = 0
    value = read_serial_port(length)
    status = bytearray(value)
    if(status[0]):
        print("\n   FAIL (Check if the RDP level you entered is valid)")
    else:
        print("\n   SUCCESS")

# 0x5A
def process_BL_ENABLE_WRP(length):
    status=0
    value = read_serial_port(length)
    status = bytearray(value)
    if(status[0]):
        print("\n   FAIL")
    else:
        print("\n   SUCCESS")
        
# 0x5B
def process_BL_DISABLE_WRP(length):
    status=0
    value = read_serial_port(length)
    status = bytearray(value)
    if(status[0]):
        print("\n   FAIL")
    else:
        print("\n   SUCCESS")

# 0x5C
# TODO: This function needs update. Check if 2-byte long 'nwrp' is transformed
# into correct output
def process_BL_GET_WRP_STATUS(length):
    nwrp = 0

    value = read_serial_port(length)
    nwrp = bytearray(value)
    #s_status.flash_sector_status = (uint16_t)(status[1] << 8 | status[0] )
    #print("\n   Sector Status : ", s_status[0])
    print("\n  ====================================")
    print("\n  Sector                               \tProtection") 
    print("\n  ====================================")

    for x in range(8):
        print("\n   Sector{0}                               {1}".format(x,protection_type(nwrp[0],x) ) )
    for x in range(4):
        print("\n   Sector{0}                               {1}".format(x,protection_type(nwrp[1],x) ) )
        
def decode_menu_command_code(command):
    ret_value = 0
    data_buf = []
    for i in range(255):
        data_buf.append(0)
    
    if(command  == 0 ):
        print("\n   Exiting...!")
        raise SystemExit

    elif(command == 1):
        print("\n   Command == > BL_GET_VER")
        BL_GET_VER_LEN              = 6
        data_buf[0] = BL_GET_VER_LEN-1 
        data_buf[1] = BL_GET_VER 
        crc32       = get_crc(data_buf,BL_GET_VER_LEN-4)
        crc32 = crc32 & 0xffffffff
        data_buf[2] = word_to_byte(crc32,1,1) 
        data_buf[3] = word_to_byte(crc32,2,1) 
        data_buf[4] = word_to_byte(crc32,3,1) 
        data_buf[5] = word_to_byte(crc32,4,1) 
        
        Write_to_serial_port(data_buf[0],1)
        for i in data_buf[1:BL_GET_VER_LEN]:
            Write_to_serial_port(i,BL_GET_VER_LEN-1)

        ret_value = read_bootloader_reply(data_buf[1])
        
    elif(command == 2):
        print("\n   Command == > BL_GET_HELP")
        BL_GET_HELP_LEN             =6
        data_buf[0] = BL_GET_HELP_LEN-1 
        data_buf[1] = BL_GET_HELP 
        crc32       = get_crc(data_buf,BL_GET_HELP_LEN-4)
        crc32 = crc32 & 0xffffffff
        data_buf[2] = word_to_byte(crc32,1,1) 
        data_buf[3] = word_to_byte(crc32,2,1) 
        data_buf[4] = word_to_byte(crc32,3,1) 
        data_buf[5] = word_to_byte(crc32,4,1) 

        Write_to_serial_port(data_buf[0],1)
        for i in data_buf[1:BL_GET_HELP_LEN]:
            Write_to_serial_port(i,BL_GET_HELP_LEN-1)

        ret_value = read_bootloader_reply(data_buf[1])

    elif(command == 3):
        print("\n   Command == > BL_GET_CID")
        BL_GET_CID_LEN             =6
        data_buf[0] = BL_GET_CID_LEN-1 
        data_buf[1] = BL_GET_CID 
        crc32       = get_crc(data_buf,BL_GET_CID_LEN-4)
        crc32 = crc32 & 0xffffffff
        data_buf[2] = word_to_byte(crc32,1,1) 
        data_buf[3] = word_to_byte(crc32,2,1) 
        data_buf[4] = word_to_byte(crc32,3,1) 
        data_buf[5] = word_to_byte(crc32,4,1) 

        Write_to_serial_port(data_buf[0],1)
        for i in data_buf[1:BL_GET_CID_LEN]:
            Write_to_serial_port(i,BL_GET_CID_LEN-1)

        ret_value = read_bootloader_reply(data_buf[1])

    elif(command == 4):
        print("\n   Command == > BL_GO_TO_ADDR")
        go_address  = input("\n   Please enter 4 bytes go address in hex:")
        go_address = int(go_address, 16)
        data_buf[0] = BL_GO_TO_ADDR_LEN-1 
        data_buf[1] = BL_GO_TO_ADDR 
        data_buf[2] = word_to_byte(go_address,1,1) 
        data_buf[3] = word_to_byte(go_address,2,1) 
        data_buf[4] = word_to_byte(go_address,3,1) 
        data_buf[5] = word_to_byte(go_address,4,1) 
        crc32       = get_crc(data_buf,BL_GO_TO_ADDR_LEN-4) 
        data_buf[6] = word_to_byte(crc32,1,1) 
        data_buf[7] = word_to_byte(crc32,2,1) 
        data_buf[8] = word_to_byte(crc32,3,1) 
        data_buf[9] = word_to_byte(crc32,4,1) 

        Write_to_serial_port(data_buf[0],1)
        
        for i in data_buf[1:BL_GO_TO_ADDR_LEN]:
            Write_to_serial_port(i,BL_GO_TO_ADDR_LEN-1)
        
        ret_value = read_bootloader_reply(data_buf[1])

    elif(command == 5):
        print("\n   Command == > BL_ERASE_FLASH")
        data_buf[0] = BL_ERASE_FLASH_LEN-1 
        data_buf[1] = BL_ERASE_FLASH 
        sector_num = input("\n   Enter sector number(0-11 or 0xFF) here :")
        sector_num = int(sector_num, 16)
        if(sector_num != 0xff):
            nsec=int(input("\n   Enter number of sectors to erase(max 12) here :"))
        else:
            nsec=0  # Added by Kyungjae Lee to resolve an issue when sector_num=0xff
        
        data_buf[2]= sector_num 
        data_buf[3]= nsec 

        crc32       = get_crc(data_buf,BL_ERASE_FLASH_LEN-4) 
        data_buf[4] = word_to_byte(crc32,1,1) 
        data_buf[5] = word_to_byte(crc32,2,1) 
        data_buf[6] = word_to_byte(crc32,3,1) 
        data_buf[7] = word_to_byte(crc32,4,1) 

        Write_to_serial_port(data_buf[0],1)
        
        for i in data_buf[1:BL_ERASE_FLASH_LEN]:
            Write_to_serial_port(i,BL_ERASE_FLASH_LEN-1)
        
        ret_value = read_bootloader_reply(data_buf[1])
        
    elif(command == 6):
        print("\n   Command == > BL_READ_MEM")
        print("\n   This command is not supported")
        
    elif(command == 7):
        print("\n   Command == > BL_WRITE_MEM")
        bytes_remaining=0
        t_len_of_file=0
        bytes_so_far_sent = 0
        len_to_read=0
        base_mem_address=0

        data_buf[1] = BL_WRITE_MEM

        #First get the total number of bytes in the .bin file.
        t_len_of_file =calc_file_len()

        #keep opening the file
        open_the_file()

        bytes_remaining = t_len_of_file - bytes_so_far_sent

        base_mem_address = input("\n   Enter the memory write address here :")
        base_mem_address = int(base_mem_address, 16)
        global mem_write_active
        while(bytes_remaining):
            mem_write_active=1
            if(bytes_remaining >= 128):
                len_to_read = 128
            else:
                len_to_read = bytes_remaining
            #get the bytes in to buffer by reading file
            for x in range(len_to_read):
                file_read_value = bin_file.read(1)
                file_read_value = bytearray(file_read_value)
                data_buf[7+x]= int(file_read_value[0])
            #read_the_file(&data_buf[7],len_to_read) 
            #print("\n   base mem address = \n",base_mem_address, hex(base_mem_address)) 

            #populate base mem address
            data_buf[2] = word_to_byte(base_mem_address,1,1)
            data_buf[3] = word_to_byte(base_mem_address,2,1)
            data_buf[4] = word_to_byte(base_mem_address,3,1)
            data_buf[5] = word_to_byte(base_mem_address,4,1)

            data_buf[6] = len_to_read

            #/* 1 byte len + 1 byte command code + 4 byte mem base address
            #* 1 byte payload len + len_to_read is amount of bytes read from file + 4 byte CRC
            #*/
            mem_write_cmd_total_len = BL_WRITE_MEM_LEN+len_to_read

            #first field is "len_to_follow"
            data_buf[0] =mem_write_cmd_total_len-1

            crc32       = get_crc(data_buf,mem_write_cmd_total_len-4)
            data_buf[7+len_to_read] = word_to_byte(crc32,1,1)
            data_buf[8+len_to_read] = word_to_byte(crc32,2,1)
            data_buf[9+len_to_read] = word_to_byte(crc32,3,1)
            data_buf[10+len_to_read] = word_to_byte(crc32,4,1)

            #update base mem address for the next loop
            base_mem_address+=len_to_read

            Write_to_serial_port(data_buf[0],1)
        
            for i in data_buf[1:mem_write_cmd_total_len]:
                Write_to_serial_port(i,mem_write_cmd_total_len-1)

            bytes_so_far_sent+=len_to_read
            bytes_remaining = t_len_of_file - bytes_so_far_sent
            print("\n   bytes_so_far_sent:{0} -- bytes_remaining:{1}\n".format(bytes_so_far_sent,bytes_remaining)) 
        
            ret_value = read_bootloader_reply(data_buf[1])
        mem_write_active=0

    elif(command == 8):
        print("\n   Command == > BL_GET_RDP_LEVEL")
        data_buf[0] = BL_GET_RDP_LEVEL_LEN-1
        data_buf[1] = BL_GET_RDP_LEVEL
        crc32       = get_crc(data_buf,BL_GET_RDP_LEVEL_LEN-4)
        crc32 = crc32 & 0xffffffff
        data_buf[2] = word_to_byte(crc32,1,1)
        data_buf[3] = word_to_byte(crc32,2,1)
        data_buf[4] = word_to_byte(crc32,3,1)
        data_buf[5] = word_to_byte(crc32,4,1)
        
        Write_to_serial_port(data_buf[0],1)
        
        for i in data_buf[1:BL_GET_RDP_LEVEL_LEN]:
            Write_to_serial_port(i,BL_GET_RDP_LEVEL_LEN-1)
        
        ret_value = read_bootloader_reply(data_buf[1])

    elif(command == 9):
        print("\n   Command == > BL_SET_RDP_LEVEL")
        print("\n   Read Protection Level 0: 0 - No read protection")
        print("\n   Read Protection Level 1: 1 - Memory read protection")
        print("\n   Read Protection Level 2: 2 - Chip read protection (Be careful!!! It's irreversible!)")
        rdp_level = int(input("\n   Enter the RDP level to set:"))
        if(rdp_level != 0 and rdp_level != 1 and rdp_level != 2):
            printf("\n   Invalid RDP level. Command Dropped.")
            return
        if(rdp_level == 2):
            print("""
                \n   In this program, you are not allowed to transition to 
                RDP Level 2 since it is an irreversible transition. Please try
                RDP Level 0 or 1.
                """)
            return
        data_buf[0] = BL_SET_RDP_LEVEL_LEN-1
        data_buf[1] = BL_SET_RDP_LEVEL
        data_buf[2] = rdp_level;
        crc32       = get_crc(data_buf,BL_SET_RDP_LEVEL_LEN-4)
        crc32 = crc32 & 0xffffffff
        data_buf[3] = word_to_byte(crc32,1,1)
        data_buf[4] = word_to_byte(crc32,2,1)
        data_buf[5] = word_to_byte(crc32,3,1)
        data_buf[6] = word_to_byte(crc32,4,1)
        
        Write_to_serial_port(data_buf[0],1)
        
        for i in data_buf[1:BL_SET_RDP_LEVEL_LEN]:
            Write_to_serial_port(i,BL_SET_RDP_LEVEL_LEN-1)
        
        ret_value = read_bootloader_reply(data_buf[1])

    elif(command == 10):
        print("\n   Command == > BL_ENABLE_WRP")
        total_sector = int(input("\n   How many sectors do you want to write protect ?: "))
        sector_numbers = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]   # 12 sectors (MCU specific)
        sector_not_write_protect = 0 
        for x in range(total_sector):
#TODO: Optimize the output string
            sector_numbers[x] = int(input("\n   Enter sector number[{0}]: ".format(x + 1)))
            sector_details = sector_not_write_protect | (1 << sector_numbers[x])

        data_buf[0] = BL_ENABLE_WRP_LEN-1 
        data_buf[1] = BL_ENABLE_WRP 
        data_buf[2] = (sector_not_write_protect >> 8) & 0x0F
        data_buf[3] = sector_not_write_protect & 0xFF
        crc32       = get_crc(data_buf,BL_ENABLE_WRP_LEN-4) 
        data_buf[4] = word_to_byte(crc32,1,1) 
        data_buf[5] = word_to_byte(crc32,2,1) 
        data_buf[6] = word_to_byte(crc32,3,1) 
        data_buf[7] = word_to_byte(crc32,4,1) 

        Write_to_serial_port(data_buf[0],1)
        
        for i in data_buf[1:BL_ENABLE_WRP_LEN]:
            Write_to_serial_port(i,BL_ENABLE_WRP_LEN-1)
        
        ret_value = read_bootloader_reply(data_buf[1])

    elif(command == 11):
        print("\n   Command == > BL_DISABLE_WRP")
        data_buf[0] = BL_DISABLE_WRP_LEN-1 
        data_buf[1] = BL_DISABLE_WRP 
        crc32       = get_crc(data_buf,BL_DISABLE_WRP_LEN-4) 
        data_buf[2] = word_to_byte(crc32,1,1) 
        data_buf[3] = word_to_byte(crc32,2,1) 
        data_buf[4] = word_to_byte(crc32,3,1) 
        data_buf[5] = word_to_byte(crc32,4,1) 

        Write_to_serial_port(data_buf[0],1)
        
        for i in data_buf[1:BL_DISABLE_WRP_LEN]:
            Write_to_serial_port(i,BL_DISABLE_WRP_LEN-1)
        
        ret_value = read_bootloader_reply(data_buf[1])

    elif(command == 12):
        print("\n   Command == > BL_GET_WRP_STATUS")
        data_buf[0] = BL_GET_WRP_STATUS_LEN-1 
        data_buf[1] = BL_GET_WRP_STATUS 

        crc32       = get_crc(data_buf,BL_GET_WRP_STATUS_LEN-4) 
        data_buf[2] = word_to_byte(crc32,1,1) 
        data_buf[3] = word_to_byte(crc32,2,1) 
        data_buf[4] = word_to_byte(crc32,3,1) 
        data_buf[5] = word_to_byte(crc32,4,1) 

        Write_to_serial_port(data_buf[0],1)
        
        for i in data_buf[1:BL_GET_WRP_STATUS_LEN]:
            Write_to_serial_port(i,BL_GET_WRP_STATUS_LEN-1)
        
        ret_value = read_bootloader_reply(data_buf[1])

    elif(command == 13):
        print("\n   Command == > COMMAND_OTP_READ")
        print("\n   This command is not supported")

        
    else:
        print("\n   Please enter a valid menu\n")
        return

    if ret_value == -2 :
        print("\n   TimeOut : No response from the bootloader")
        print("\n   Reset the board and Try Again !")
        return

def read_bootloader_reply(command_code):
    #ack=[0,0]
    len_to_follow=0 
    ret = -2 

    #read_serial_port(ack,2)
    #ack = ser.read(2)
    ack=read_serial_port(2)
    if(len(ack) ):
        a_array=bytearray(ack)
        #print("read uart:",ack) 
        if (a_array[0]== 0xA5):
            #CRC of last command was good .. received ACK and "len to follow"
            len_to_follow=a_array[1]
            print("\n   CRC : SUCCESS Len :",len_to_follow)
            #print("command_code:",hex(command_code))
            if (command_code) == BL_GET_VER :
                process_BL_GET_VER(len_to_follow)
                
            elif(command_code) == BL_GET_HELP:
                process_BL_GET_HELP(len_to_follow)
                
            elif(command_code) == BL_GET_CID:
                process_BL_GET_CID(len_to_follow)
                
            elif(command_code) == BL_GO_TO_ADDR:
                process_BL_GO_TO_ADDR(len_to_follow)
                
            elif(command_code) == BL_ERASE_FLASH:
                process_BL_ERASE_FLASH(len_to_follow)
                
            elif(command_code) == BL_READ_MEM:
                process_BL_READ_MEM(len_to_follow)
                
            elif(command_code) == BL_WRITE_MEM:
                process_BL_WRITE_MEM(len_to_follow)
                
            elif(command_code) == BL_GET_RDP_LEVEL:
                process_BL_GET_RDP_LEVEL(len_to_follow)
                
            elif(command_code) == BL_SET_RDP_LEVEL:
                process_BL_SET_RDP_LEVEL(len_to_follow)
                
            elif(command_code) == BL_ENABLE_WRP:
                process_BL_ENABLE_WRP(len_to_follow)
                
            elif(command_code) == BL_DISABLE_WRP:
                process_BL_DISABLE_WRP(len_to_follow)
                
            elif(command_code) == BL_GET_WRP_STATUS:
                process_BL_GET_WRP_STATUS(len_to_follow)
                
            elif(command_code) == BL_GET_OTP:
                process_BL_GET_OTP(len_to_follow)
                
            else:
                print("\n   Invalid command code\n")
                
            ret = 0
         
        elif a_array[0] == 0x7F:
            #CRC of last command was bad .. received NACK
            print("\n   CRC: FAIL \n")
            ret= -1
    else:
        print("\n   Timeout : Bootloader not responding")
        
    return ret

#--------------------------------- Menu ----------------------------------------

name = input("Enter the port name of your device "
			 "(Hit enter to see available ports): ")
ret = 0
ret=Serial_Port_Configuration(name)
if(ret < 0):
    decode_menu_command_code(0)
  
while True:
    print("\n +==========================================+")
    print(" |       STM32F407xx MCU Bootloader         |")
    print(" |          Available Commands              |")
    print(" +==========================================+")
    print("")
    print(" 1.  BL_GET_VER")
    print(" 2.  BL_GET_HLP")
    print(" 3.  BL_GET_CID")
    print(" 4.  BL_GO_TO_ADDR")
    print(" 5.  BL_ERASE_FLASH")
    print(" 6.  BL_READ_MEM")
    print(" 7.  BL_WRITE_MEM")
    print(" 8.  BL_GET_RDP_LEVEL")
    print(" 9.  BL_SET_RDP_LEVEL")
    print(" 10. BL_ENABLE_WRP")
    print(" 11. BL_DISABLE_WRP")
    print(" 12. BL_GET_WRP_STATUS")
    print(" 13. BL_GET_OTP")
    print(" 0.  EXIT")

    #command_code = int(input("\n Select a bootloader command: "))
    command_code = input("\n Enter a bootloader command number: ")

    if(not command_code.isdigit()):
        print("\n Please Input valid code shown above")
    else:
        decode_menu_command_code(int(command_code))

    input("\n Press any key to continue  :")
    purge_serial_port()

def check_flash_status():
    pass

def protection_type():
    pass
