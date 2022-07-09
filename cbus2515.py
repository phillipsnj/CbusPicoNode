# MicroPython CBUS/MCP2515 CAN controller driver
# (c)2021 Tony Witts

# 220604 - First commit to GitHub.
# 210322 - Handles Enumeration in and out
#        - debug parameter to print debug messages
#        - Handles Major Priority if message not getting through
# 210311-Nigel reports OK!
# 210302-Tidied
# 210227-Working
# 210220-A New Start

from machine import Timer
from micropython import const
from time import ticks_us, ticks_ms, ticks_diff, sleep
from binascii import hexlify, unhexlify
import uasyncio

STACK_LEN = const(50)
STACK_TOT = const(13 * STACK_LEN)
TX_TIMEOUT = const(100)

HEXDIGITS = "0123456789abcdefABCDEF"

# Register definitions
CANSTAT = const(0x0E)
CANCTRL = const(0x0F)
TEC = const(0x1C)
REC = const(0x1D)
CNF3 = const(0x28)
CNF2 = const(0x29)
CNF1 = const(0x2A)
CANINTE = const(0x2B)
CANINTF = const(0x2C)
EFLG = const(0x2D)

TXB0CTRL = const(0x30)
TXB0SIDH = const(0x31)
TXB0SIDL = const(0x32)
TXB0EID8 = const(0x33)
TXB0EID0 = const(0x34)
TXB0DLC = const(0x35)
TXB0D0 = const(0x36)
TXB0D1 = const(0x37)
TXB0D2 = const(0x38)
TXB0D3 = const(0x39)
TXB0D4 = const(0x3A)
TXB0D5 = const(0x3B)
TXB0D6 = const(0x3C)
TXB0D7 = const(0x3D)

RXB0CTRL = const(0x60)
RXB0SIDH = const(0x61)
RXB0SIDL = const(0x62)
RXB0EID8 = const(0x63)
RXB0EID0 = const(0x64)
RXB0DLC = const(0x65)
RXB0D0 = const(0x66)
RXB0D1 = const(0x67)
RXB0D2 = const(0x68)
RXB0D3 = const(0x69)
RXB0D4 = const(0x6A)
RXB0D5 = const(0x6B)
RXB0D6 = const(0x6C)
RXB0D7 = const(0x6D)

RXB1CTRL = const(0x60)
RXB1SIDH = const(0x61)
RXB1SIDL = const(0x62)
RXB1EID8 = const(0x63)
RXB1EID0 = const(0x64)
RXB1DLC = const(0x65)
RXB1D0 = const(0x66)
RXB1D1 = const(0x67)
RXB1D2 = const(0x68)
RXB1D3 = const(0x69)
RXB1D4 = const(0x6A)
RXB1D5 = const(0x6B)
RXB1D6 = const(0x6C)
RXB1D7 = const(0x6D)

# Bit definition masks
TXREQ = const(8)
EXIDE = const(8)
RXRTR = const(8)
IDE = const(8)
SRR = const(16)
RTR = const(64)
DLC = const(15)

# Command definitions
CMD_WRITE = const(0x02)
CMD_READ = const(0x03)
CMD_MODIFY = const(0x05)
CMD_READ_STATUS = const(0xA0)
CMD_RX_STATUS = const(0xB0)
CMD_RESET = const(0xC0)

# Timing constants
CNF = {
    8000000: (0x01, 0xB1, 0x85),
    16000000: (0x03, 0xF0, 0x06)
}


# Cbus2515 Class
class Cbus2515():
    def __init__(self, spi, cs, interrupt, osc=16000000, debug=False):
        self.initialised = False
        self.debug = debug
        self.buffer = bytearray(5)
        self.tx_buffer = bytearray(13)
        self.rx_stack = bytearray(STACK_TOT)
        self.rx_stack_mv = memoryview(self.rx_stack)
        self.stack_in = 0
        self.stack_out = 0
        self.id_stack = []
        self.enumerate = False
        self.data = [0 for i in range(8)]
        self.rate = 1024 * 1024
        self.cs = cs
        self.cs.init(self.cs.OUT, value=1)
        interrupt.init(interrupt.IN, interrupt.PULL_UP)
        interrupt.irq(trigger=interrupt.IRQ_FALLING, handler=self.can_irq)
        self.spi = spi
        self.cs(1)
        self.cs(0)
        self.spi.write(bytearray([CMD_RESET]))
        self.cs(1)
        if self.read_reg(CANCTRL) & 7 != 7:
            if self.debug: print("MCP2515 missing!")
            self.can_id_msg = ''
            return
        self.can_id = self.get_can_id()
        self.can_sid = bytearray([self.can_id >> 3, self.can_id << 5])
        self.can_id_msg = ":S" + hexlify(self.can_sid).decode() + "N;"
        if self.debug: print("ID:", self.can_id, self.can_sid, self.can_id_msg)
        self.send_pause = False
        self.init_can(osc)
        self.initialised = True

    def init_can(self, osc):
        for reg, data in (
                # Timing
                (CNF1, CNF[osc][0]),
                (CNF2, CNF[osc][1]),
                (CNF3, CNF[osc][2]),
                # Filters and Masks
                (RXB0CTRL, 0x60),  # Do not use Filters or Masks
                (RXB1CTRL, 0x60),  # Do not use Filters or Masks
                # Interrupts
                (CANINTE, 0x01),
        ):
            self.write_reg(reg, data)

    def get_can_id(self):
        try:
            with open("CAN_ID.ini") as f:
                can_id = f.read()
                if can_id:
                    can_id = int(can_id)
                else:
                    raise OSError
        except OSError:
            can_id = 12
            with open("CAN_ID.ini", 'w') as f:
                f.write(str(can_id))
        return can_id

    def save_can_id(self, can_id):
        try:
            with open("CAN_ID.ini", "w") as f:
                f.write(str(can_id))
        except OSError:
            if self.debug: print("Can't write ID file!")

    def can_enumerate(self, timer):
        self.enumerate = False
        timer.deinit()
        if self.debug: print("-can_enumerate: ", self.id_stack)
        for idh in range(16):
            for idl in range(32, 225, 32):
                if (idh, idl) not in self.id_stack:
                    self.can_id = int(idh * 8 + idl / 32)
                    self.can_sid = bytearray([idh, idl])
                    self.can_id_msg = ":S" + hexlify(self.can_sid).decode() + "N;"
                    self.save_can_id(self.can_id)
                    self.id_stack = []
                    if self.debug: print("-Enum:", (idh, idl), self.can_id)
                    return
        self.id_stack = []

    def can_irq(self, p):
        rx = self.read_regs(RXB0SIDH, 0x0D)
        rx[0] = rx[0] & 0x0F
        if rx[1] & SRR:  # Respond to Enumeration
            self.modify_reg(CANINTF, 1, 0)
            self.send(self.can_id_msg)  # Send our ID
            return
        if self.enumerate and rx[4] & 0x0F == 0:  # Stack zero length message IDs
            if self.debug: print("-ZL", rx[0], rx[1])  # when Enumerating
            self.id_stack.append((rx[0], rx[1]))
            self.modify_reg(CANINTF, 1, 0)
            return
        self.rx_stack_mv[self.stack_in:self.stack_in + 13] = rx
        self.stack_in += 0x0D
        if self.stack_in >= STACK_TOT:
            self.stack_in = 0
        self.modify_reg(CANINTF, 1, 0)
        if rx[0:2] == self.can_sid:  # CLASH! Our ID on Bus
            self.enumerate = True
            self.send(":SB020R;")
            _ = Timer(freq=10, mode=Timer.ONE_SHOT, callback=self.can_enumerate)

    def in_waiting(self):
        a = (self.stack_in - self.stack_out) // 13
        if a < 0: a += STACK_LEN
        return a

    def receive(self):
        if self.debug:
            print("\n-Receive-", end='')
            print(self.in_waiting(), end='')
            print("--------------------------")
        if self.in_waiting():
            rx_buffer = self.rx_stack[self.stack_out:self.stack_out + 13]
            self.stack_out += 13
            if self.stack_out >= 13 * STACK_LEN:
                self.stack_out = 0
            if self.debug: print("-rx_buffer:", rx_buffer)
            msg = ':'
            if rx_buffer[1] & IDE:
                msg += 'X'
                msg += hexlify(rx_buffer[0:4]).decode()
                if rx_buffer[4] & RTR:
                    msg += 'R'
                else:
                    msg += 'N'
            else:
                msg += 'S'
                msg += hexlify(rx_buffer[0:2]).decode()
                if rx_buffer[1] & SRR:
                    msg += 'R'
                else:
                    msg += 'N'
            n = int(rx_buffer[4]) & DLC
            rd = rx_buffer[5:5 + n]
            if self.debug: print("-no of data, rx_data:", n, rd)
            msg += hexlify(rd).decode()
            msg += ';'
            if self.debug: print("-msg:", msg.upper())
            return msg.upper()
        else:
            return ""

    def write_reg(self, reg, data):
        self.buffer = bytearray([CMD_WRITE, reg, data])
        self.spi.init(baudrate=self.rate)
        self.cs(1)
        self.cs(0)
        self.spi.write(self.buffer)
        self.cs(1)

    def read_reg(self, reg):
        self.buffer = bytearray([CMD_READ, reg, 0])
        self.spi.init(baudrate=self.rate)
        self.cs(1)
        self.cs(0)
        self.spi.write_readinto(self.buffer, self.buffer)
        self.cs(1)
        return self.buffer[2]

    def write_regs(self, reg, data):
        self.buffer = bytearray([CMD_WRITE, reg])
        self.spi.init(baudrate=self.rate)
        self.cs(1)
        self.cs(0)
        self.spi.write(self.buffer + data)
        self.cs(1)

    def read_regs(self, reg, num):
        self.buffer = bytearray([CMD_READ, reg]) + bytearray(num)
        self.spi.init(baudrate=self.rate)
        self.cs(1)
        self.cs(0)
        self.spi.write_readinto(self.buffer, self.buffer)
        self.cs(1)
        return self.buffer[2:]

    def modify_reg(self, reg, mask, data):
        self.buffer = bytearray([CMD_MODIFY, reg, mask, data])
        self.spi.init(baudrate=self.rate)
        self.cs(1)
        self.cs(0)
        self.spi.write(self.buffer)
        self.cs(1)

    def change_mode(self, mode):
        self.write_reg(CANCTRL, (mode << 5))
        start = ticks_ms()
        while self.read_reg(CANSTAT) >> 5 != mode:
            if ticks_diff(ticks_ms(), start) > TX_TIMEOUT:
                if self.debug: print("Mode Timeout!")
                return 1

    def read_rx_status(self):
        self.buffer = bytearray([CMD_RX_STATUS, 0])
        self.spi.init(baudrate=self.rate)
        self.cs(1)
        self.cs(0)
        self.spi.write_readinto(self.buffer, self.buffer)
        self.cs(1)
        return self.buffer[1]

    def send(self, msg=""):
        tx_buffer = self.tx_buffer
        if self.debug: print("\n-Send------------------------------")
        if self.debug: print("-msg:", msg)
        if len(msg) < 8:
            if self.debug: print("Message too short!")
            return 10
        if msg[0] != ':' or msg[-1] != ';':
            if self.debug: print("Message format wrong!")
            return 11
        if msg[1] == 'S':
            if not all(c in HEXDIGITS for c in msg[2:6] + msg[7:-1]):
                if self.debug: print("Non-Hex digits!")
                return 12
            tx_buffer[0] = (self.can_id >> 3)
            tx_buffer[0] |= int.from_bytes(unhexlify(msg[2:4]), 'little') & 0b00110000
            tx_buffer[1] = self.can_id << 5
            self.data = unhexlify(msg[7:-1])
            tx_buffer[4] = (msg[6] == 'R') * 64 + len(self.data)
            tx_buffer[5:5 + len(self.data)] = self.data
        elif msg[1] == 'X':
            if not all(c in HEXDIGITS for c in msg[2:10] + msg[11:-1]):
                if self.debug: print("Non-Hex digits!")
                return 12
            tx_buffer[0:4] = unhexlify(msg[2:10])
            tx_buffer[1] |= 0b00001000
            self.data = unhexlify(msg[11:-1])
            tx_buffer[4] = (msg[10] == 'R') * 64 + len(self.data)
            tx_buffer[5:5 + len(self.data)] = self.data
        else:
            if self.debug: print("Message not recognised!")
            return 3
        if self.debug: print("-Sending:", tx_buffer)
        self.write_regs(0x31, tx_buffer)
        self.write_reg(TXB0CTRL, 3)
        if self.read_reg(TXB0CTRL) & 3 != 3:  # Check interface
            if self.debug: print("MCP2515 missing!")
            return 9
        for MjPri in range(128, -1, -64):
            if self.read_reg(TXB0CTRL) & TXREQ:
                if self.debug: print("Tx buffer not available!")
                return 2
            self.modify_reg(TXB0SIDH, 0b11000000, MjPri)
            self.modify_reg(TXB0CTRL, TXREQ, TXREQ)  # Request transmisssion
            self.start = ticks_ms()
            while self.read_reg(TXB0CTRL) & TXREQ:
                if ticks_diff(ticks_ms(), self.start) > TX_TIMEOUT:
                    self.modify_reg(TXB0CTRL, TXREQ, 0)  # Abort transmisssion
                    if MjPri == 0:
                        if self.debug: print("Tx Timeout!")
                        return 1
            break
        self.modify_reg(CANINTF, 4, 0)
        return False

    def monitor(self):
        mon = self.read_regs(CNF3, 8)
        print()
        print(" Timing: " + hex(mon[2])[2:] + " " + hex(mon[1])[2:] + " " + hex(mon[0])[2:])
        print("   Mode: " + str((mon[6] & 0xE0) >> 5))
        print("   EFLG: " + hex(mon[5])[2:])
        print("   IntE: " + hex(mon[3])[2:] + "  IntF: " + hex(mon[4])[2:])
        print("   ICOD: " + str((mon[6] & 0x0E) >> 1))
        print("CANCTRL: " + hex(mon[7])[2:])