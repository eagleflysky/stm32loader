#!/usr/bin/env python

# -*- coding: utf-8 -*-
# vim: sw=4:ts=4:si:et:enc=utf-8

# Author: Ivan A-R <ivan@tuxotronic.org>
# Project page: http://tuxotronic.org/wiki/projects/stm32loader
# Modified by Eagle Ding <dingfp@hotmail.com>
#   mainly change code to python3.x and correct some CRC checksum
#
# This file is part of stm32loader.
#
# stm32loader is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free
# Software Foundation; either version 3, or (at your option) any later
# version.
#
# stm32loader is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License
# along with stm32loader; see the file COPYING3.  If not see
# <http://www.gnu.org/licenses/>.

import sys, getopt
import serial
import time

try:
    from progressbar import *
    usepbar = 1
except:
    usepbar = 0

# Verbose level
QUIET = 20

# these come from AN2606
chip_ids = {
    0x412: "STM32 Low-density",
    0x410: "STM32 Medium-density",
    0x414: "STM32 High-density",
    0x420: "STM32 Medium-density value line",
    0x428: "STM32 High-density value line",
    0x430: "STM32 XL-density",
    0x416: "STM32 Medium-density ultralow power line",
    0x411: "STM32F2xx",
    0x450: "STM32H750",
}

def mdebug(level, message):
    if(QUIET >= level):
        print(message) # >> sys.stderr , message

FLASH_BASE_ADDR   = 0x90000000
FLASH_SECTOR_SIZE = 0x10000       #64KBytes
def prepare_sectors(address,data):
    assert(address >= FLASH_BASE_ADDR)
    data_len = len(data)
    address  = address - FLASH_BASE_ADDR
    sectors  = []
    sector_addrs = range(address,data_len,FLASH_SECTOR_SIZE)
    for sector_addr in sector_addrs:
        sector_num = math.floor(sector_addr/FLASH_SECTOR_SIZE)
        sectors.append( (sector_num<<8) & 0xFF )
        sectors.append( sector_num & 0xFF )
    return sectors

class CmdException(Exception):
    pass

class CommandInterface:
    extended_erase = 0

    def open(self, aport='COM7', abaudrate=115200, aparity=serial.PARITY_NONE) :
        global FLASH_BASE_ADDR
        if(aparity == serial.PARITY_EVEN):
            FLASH_BASE_ADDR = 0x08000000 #internal flash 
        self.port = aport
        self.baud = abaudrate
        self.parity = aparity
        self.sp = serial.Serial(
            port=aport,
            baudrate=abaudrate,     # baudrate
            bytesize=8,             # number of databits
            parity=aparity,         #serial.PARITY_EVEN,
            stopbits=1,
            xonxoff=0,              # don't enable software flow control
            rtscts=0,               # don't enable RTS/CTS flow control
            timeout=2               # set a timeout value, None for waiting forever
        )
        
    def reopen(self):
        self.sp.close()
        self.sp = serial.Serial(
            port=self.port,
            baudrate=self.baud,     # baudrate
            bytesize=8,             # number of databits
            parity=self.parity,#serial.PARITY_EVEN,
            stopbits=1,
            xonxoff=0,              # don't enable software flow control
            rtscts=0,               # don't enable RTS/CTS flow control
            timeout=2               # set a timeout value, None for waiting forever
        )     
        self.reset()

    def _wait_for_ask(self, info = ""):
        # wait for ask
        try:
            #ask = ord(self.sp.read())
            ask = self.sp.read()
            #for a in ask:
            #    print(hex(a))
        except:
            raise CmdException("Can't read port or timeout")
        else:
            if ask[0] == 0x79:
                # ACK
                return 1
            else:
                if ask[0] == 0x1F:
                    # NACK
                    raise CmdException("NACK "+info)
                else:
                    # Unknown responce
                    raise CmdException("Unknown response. "+info+": "+ask.hex())


    def reset(self):
        self.sp.flush()
        self.sp.setDTR(0)
        #time.sleep(0.01)
        self.sp.setDTR(1)
        #time.sleep(0.01)

    def initChip(self):
        # Set boot
        self.sp.setRTS(0)
        if(self.parity!=serial.PARITY_EVEN):
            self.sp.write(b"\x55\x55\x55\x55\x55") # autobaud for qspi boot loader
        self.sp.write(b"\x7F")       # Syncro
        return self._wait_for_ask("Syncro")

    def releaseChip(self):
        self.sp.setRTS(1)
        self.reset()

    def cmdGeneric(self, cmd):
        #time.sleep(0.001)
        self.sp.write(bytes([cmd]))
        #time.sleep(0.001)
        self.sp.write(bytes([cmd ^ 0xFF])) # Control byte
        #print(hex(cmd),hex(cmd ^ 0xFF))
        return self._wait_for_ask(hex(cmd))

    def cmdGet(self):
        if self.cmdGeneric(0x00):
            mdebug(10, "*** Get command");
            len = ord(self.sp.read())
            version = ord(self.sp.read())
            mdebug(10, "    Bootloader version: "+hex(version))
            rx = self.sp.read(len)
            #dat = map(lambda c: hex(ord(c)), self.sp.read(len))
            dat = rx.hex()
            if '44' in dat:
                self.extended_erase = 1
            mdebug(10, "    Available commands: "+dat)#", ".join(dat))
            self._wait_for_ask("0x00 end")
            return version
        else:
            raise CmdException("Get (0x00) failed")

    def cmdGetVersion(self):
        if self.cmdGeneric(0x01):
            mdebug(10, "*** GetVersion command")
            version = ord(self.sp.read())
            self.sp.read(2)
            self._wait_for_ask("0x01 end")
            mdebug(10, "    Bootloader version: "+hex(version))
            return version
        else:
            raise CmdException("GetVersion (0x01) failed")

    def cmdGetID(self):
        if self.cmdGeneric(0x02):
            mdebug(10, "*** GetID command")
            len = ord(self.sp.read())
            id = self.sp.read(len+1)
            self._wait_for_ask("0x02 end")
            return id[0]<<8 | id[1]
            #return reduce(lambda x, y: x*0x100+y, map(ord, id))
        else:
            raise CmdException("GetID (0x02) failed")


    def _encode_addr(self, addr):
        byte3 = (addr >> 0) & 0xFF
        byte2 = (addr >> 8) & 0xFF
        byte1 = (addr >> 16) & 0xFF
        byte0 = (addr >> 24) & 0xFF
        crc = byte0 ^ byte1 ^ byte2 ^ byte3
        #return (chr(byte0) + chr(byte1) + chr(byte2) + chr(byte3) + chr(crc))
        return bytes([byte0,byte1,byte2,byte3,crc])


    def cmdReadMemory(self, addr, lng):
        assert(lng <= 256)
        if self.cmdGeneric(0x11):
            mdebug(10, "*** ReadMemory command")
            #self.sp.write(self._encode_addr(addr))
            addr_bytes = self._encode_addr(addr)
            #print(len(addr_bytes),addr_bytes.hex())
            for addr_byte in addr_bytes:
                self.sp.write(bytes([addr_byte]))
                #print(hex(addr_byte))
                #time.sleep(0.001)
            self._wait_for_ask("0x11 address failed")
            
            #self.reopen()
            N = bytes( [(lng - 1) & 0xFF] )
            crc = bytes( [(lng - 1) & 0xFF ^ 0xFF] )
            #self.sp.write(chr(N) + chr(crc))
            #self.cmdGeneric(N)
            self.sp.write(N) #,N.hex())
            #time.sleep(0.001)
            self.sp.write(crc) #,crc.hex())
            #time.sleep(0.001)
            self._wait_for_ask("0x11 length failed")
            
            rx = self.sp.read(lng)
            #print(len(rx),rx.hex())
            return rx.hex()
            #return map(lambda c: ord(c), self.sp.read(lng))
        else:
            raise CmdException("ReadMemory (0x11) failed")


    def cmdGo(self, addr):
        if self.cmdGeneric(0x21):
            mdebug(10, "*** Go command")
            self.sp.write(self._encode_addr(addr))
            self._wait_for_ask("0x21 go failed")
        else:
            raise CmdException("Go (0x21) failed")


    def cmdWriteMemory(self, addr, data):
        assert(len(data) <= 256)
        if self.cmdGeneric(0x31):
            mdebug(10, "*** Write memory command")
            #self.sp.write(self._encode_addr(addr))
            addr_bytes = self._encode_addr(addr)
            for addr_byte in addr_bytes:
                self.sp.write(bytes([addr_byte]))
                #print(hex(addr_byte))
                #time.sleep(0.001)
            self._wait_for_ask("0x31 address failed")
            #map(lambda c: hex(ord(c)), data)
            lng = (len(data)-1) & 0xFF
            mdebug(10, "    %s bytes to write" % [lng+1])
            self.sp.write(bytes([lng])) # len really
            #time.sleep(0.001)
            crc = lng #^ 0xFF
            for c in data:
                crc = crc ^ c
                self.sp.write(bytes([c]))
                #time.sleep(0.001)
            self.sp.write(bytes([crc]))
            #time.sleep(0.001)
            #print(hex(crc))
            #rx = self.sp.read(lng+4)
            #print(rx.hex())
            self._wait_for_ask("0x31 programming failed")
            mdebug(10, "    Write memory done")
        else:
            raise CmdException("Write memory (0x31) failed")


    def cmdEraseMemory(self, sectors = None):
        if self.extended_erase:
            return cmd.cmdExtendedEraseMemory(sectors)

        if self.cmdGeneric(0x43):
            mdebug(10, "*** Erase memory command")
            if sectors is None:
                # Global erase
                self.sp.write(bytes([0xFF]))
                self.sp.write(bytes([0x00]))
            else:
                # Sectors erase
                self.sp.write(bytes([(len(sectors)-1) & 0xFF]))
                crc = 0xFF
                for c in sectors:
                    crc = crc ^ c
                    self.sp.write(bytes([c]))
                self.sp.write(bytes([c]))
            self._wait_for_ask("0x43 erasing failed")
            mdebug(10, "    Erase memory done")
        else:
            raise CmdException("Erase memory (0x43) failed")

    def cmdExtendedEraseMemory(self,sectors = None):
        if self.cmdGeneric(0x44):
            mdebug(10, "*** Extended Erase memory command")
            if sectors is None:
                # Global mass erase
                self.sp.write(bytes([0xFF]))
                self.sp.write(bytes([0xFF]))
                # Checksum
                self.sp.write(bytes([0x00]))
            else:
                # Sectors erase
                sector_len = int(len(sectors)/2)-1
                step1 = (sector_len>>8) & 0xFF
                step2 = sector_len & 0xFF
                self.sp.write(bytes([step1]))
                self.sp.write(bytes([step2]))

                crc = step1 ^ step2
                for c in sectors:
                    crc = crc ^ c
                    self.sp.write(bytes([c]))
                self.sp.write(bytes([crc]))
            tmp = self.sp.timeout
            self.sp.timeout = 30
            print("Extended erase (0x44), this can take ten seconds or more")
            self._wait_for_ask("0x44 erasing failed")
            self.sp.timeout = tmp
            mdebug(10, "    Extended Erase memory done")
        else:
            raise CmdException("Extended Erase memory (0x44) failed")

    def cmdWriteProtect(self, sectors):
        if self.cmdGeneric(0x63):
            mdebug(10, "*** Write protect command")
            self.sp.write(bytes([(len(sectors)-1) & 0xFF]))
            crc = 0xFF
            for c in sectors:
                crc = crc ^ c
                self.sp.write(bytes([c]))
            self.sp.write(bytes([c]))
            self._wait_for_ask("0x63 write protect failed")
            mdebug(10, "    Write protect done")
        else:
            raise CmdException("Write Protect memory (0x63) failed")

    def cmdWriteUnprotect(self):
        if self.cmdGeneric(0x73):
            mdebug(10, "*** Write Unprotect command")
            self._wait_for_ask("0x73 write unprotect failed")
            self._wait_for_ask("0x73 write unprotect 2 failed")
            mdebug(10, "    Write Unprotect done")
        else:
            raise CmdException("Write Unprotect (0x73) failed")

    def cmdReadoutProtect(self):
        if self.cmdGeneric(0x82):
            mdebug(10, "*** Readout protect command")
            self._wait_for_ask("0x82 readout protect failed")
            self._wait_for_ask("0x82 readout protect 2 failed")
            mdebug(10, "    Read protect done")
        else:
            raise CmdException("Readout protect (0x82) failed")

    def cmdReadoutUnprotect(self):
        if self.cmdGeneric(0x92):
            mdebug(10, "*** Readout Unprotect command")
            self._wait_for_ask("0x92 readout unprotect failed")
            self._wait_for_ask("0x92 readout unprotect 2 failed")
            mdebug(10, "    Read Unprotect done")
        else:
            raise CmdException("Readout unprotect (0x92) failed")


# Complex commands section
    def read_id(self):
        self.sp.write(bytes([0x9F]))
        self.sp.write(bytes([0x60]))
        rx = self.sp.read(3)
        print(rx.hex())

    def readMemory(self, addr, lng):
        data = ''
        if usepbar:
            widgets = ['Reading: ', Percentage(),', ', ETA(), ' ', Bar()]
            pbar = ProgressBar(widgets=widgets,maxval=lng, term_width=79).start()
        
        while lng > 256:
            if usepbar:
                pbar.update(pbar.maxval-lng)
            else:
                mdebug(5, "Read %(len)d bytes at 0x%(addr)X" % {'addr': addr, 'len': 256})
            data = data + self.cmdReadMemory(addr, 256)
            addr = addr + 256
            lng = lng - 256
        if usepbar:
            pbar.update(pbar.maxval-lng)
            pbar.finish()
        else:
            mdebug(5, "Read %(len)d bytes at 0x%(addr)X" % {'addr': addr, 'len': 256})
        data = data + self.cmdReadMemory(addr, lng)
        return data

    def writeMemory(self, addr, data):
        lng = len(data)
        if usepbar:
            widgets = ['Writing: ', Percentage(),' ', ETA(), ' ', Bar()]
            pbar = ProgressBar(widgets=widgets, maxval=lng, term_width=79).start()
        
        offs = 0
        while lng > 256:
            if usepbar:
                pbar.update(pbar.maxval-lng)
            else:
                mdebug(5, "Write %(len)d bytes at 0x%(addr)X" % {'addr': addr, 'len': 256})
            self.cmdWriteMemory(addr, data[offs:offs+256])
            offs = offs + 256
            addr = addr + 256
            lng = lng - 256
        if lng>0:
            self.cmdWriteMemory(addr, data[offs:offs+lng] + bytes([0xFF] * (256-lng)) )
        if usepbar:
            pbar.update(pbar.maxval-lng)
            pbar.finish()
        else:
            mdebug(5, "Write %(len)d bytes at 0x%(addr)X" % {'addr': addr, 'len': 256})
        

def __init__(self) :
        pass


def usage():
    print("""Usage: %s [-hqVewvr] [-l length] [-p port] [-b baud] [-a addr] [-g addr] [file.bin]
    -h          This help
    -q          Quiet
    -V          Verbose
    -e          Erase
    -w          Write
    -v          Verify
    -r          Read
    -l length   Length of read
    -p port     Serial port (default: /dev/tty.usbserial-ftCYPMYJ)
    -b baud     Baud speed (default: 115200)
    -t parity   N(none),E(Even),O(Odd)
    -a addr     Target address
    -g addr     Address to start running at (0x08000000, usually)

    ./stm32loader.py -e -w -v example/main.bin

    """ % sys.argv[0])


if __name__ == "__main__":
    
    # Import Psyco if available
    try:
        import psyco
        psyco.full()
        print("Using Psyco...")
    except ImportError:
        pass

    conf = {
            'port': 'COM7',
            'baud': 115200,
            'parity': 'N',
            'address': 0x90000000,
            'read_id': 0,
            'erase': 0,
            'write': 0,
            'verify': 0,
            'read': 0,
            'go_addr':-1,
            'len':16,
        }

# http://www.python.org/doc/2.5.2/lib/module-getopt.html

    try:
        opts, args = getopt.getopt(sys.argv[1:], "hqVewvrip:t:b:a:l:g:")
    except(getopt.GetoptError, err):
        # print help information and exit:
        print( str(err) ) # will print something like "option -a not recognized"
        usage()
        sys.exit(2)

    QUIET = 5

    for o, a in opts:
        if o == '-V':
            QUIET = 10
        elif o == '-q':
            QUIET = 0
        elif o == '-h':
            usage()
            sys.exit(0)
        elif o == '-e':
            conf['erase'] = 1
        elif o == '-w':
            conf['write'] = 1
        elif o == '-v':
            conf['verify'] = 1
        elif o == '-r':
            conf['read'] = 1
        elif o == '-i':
            conf['read_id'] = 1
        elif o == '-p':
            conf['port'] = a
        elif o == '-b':
            conf['baud'] = eval(a)
        elif o == '-t':
            conf['parity'] = a
        elif o == '-a':
            conf['address'] = eval(a)
        elif o == '-g':
            conf['go_addr'] = eval(a)
        elif o == '-l':
            conf['len'] = eval(a)
        else:
            assert False, "unhandled option"

    cmd = CommandInterface()
    cmd.open(conf['port'], conf['baud'], conf['parity'])
    mdebug(10, "Open port %(port)s, baud %(baud)d" % {'port':conf['port'], 'baud':conf['baud']})
    try:
        #try:
        cmd.initChip()
        #except:
        #    print("Can't init. Ensure that BOOT0 is enabled and reset device")


        bootversion = cmd.cmdGet()
        mdebug(0, "Bootloader version %X" % bootversion)
        id = cmd.cmdGetID()
        mdebug(0, "Chip id: 0x%x (%s)" % (id, chip_ids.get(id, "Unknown")))
#    cmd.cmdGetVersion()
#    cmd.cmdGetID()
#    cmd.cmdReadoutUnprotect()
#    cmd.cmdWriteUnprotect()
#    cmd.cmdWriteProtect([0, 1])
        data = None
        
        if (conf['write'] or conf['verify'] or conf['erase']):
            with open(args[0],'rb') as f:
                data = f.read()
            #data = map(lambda c: ord(c), file(args[0], 'rb').read())

        if conf['erase']:
            erase_sectors = prepare_sectors(conf['address'],data)
            #print(erase_sectors)
            cmd.cmdEraseMemory(erase_sectors)

        if conf['write']:
            cmd.writeMemory(conf['address'], data)

        if conf['verify']:
            verify = cmd.readMemory(conf['address'], len(data))
            if(data.hex() == verify):
                print("Verification OK")
            else:
                print("Verification FAILED")
                print(str(len(data)) + ' vs ' + str(len(verify)/2) )
                same = 0
                unsame = 0
                for i in range(0, len(data)):
                    try:
                        if data[i] != int(verify[i*2:i*2+2],16):
                            unsame += 1
                            if(unsame == 256):
                                print("...")
                            elif(unsame < 256):
                                print( hex(i) + ': ' + hex(data[i]) + ' vs 0x' + verify[i*2:i*2+2] )
                        else:
                            same +=1
                    except:
                        print(f'verify:i={i}-'+verify[i*2:i*2+2])
                print(f"same={same}, difference={unsame}")

        if not conf['write'] and conf['read']:
            rdata = cmd.readMemory(conf['address'], conf['len'])
            #file(args[0], 'wb').write(''.join(map(chr,rdata)))

        if conf['go_addr'] != -1:
            cmd.cmdGo(conf['go_addr'])
        
        if conf['read_id']:
            cmd.read_id()

    finally:
        cmd.releaseChip()