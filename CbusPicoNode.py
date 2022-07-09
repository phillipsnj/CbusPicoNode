from machine import Pin, SPI, Timer
import time
import cbus2515
import json


class picoNode:
    def __init__(self, nodeId, myFunction):
        self.nodeId = nodeId
        self.debug = True
        self.events = {}
        self.count = 0
        self.canId = 75
        self.priority1 = 2
        self.priority2 = 3
        self.manufId = 165
        self.moduleId = 255
        self.name = "PYTHON"
        self.minorVersion = "A"
        self.numEvents = 255
        self.numEventVariables = 0
        self.numNodeVariables = 0
        self.majorVersion = 1
        self.beta = 1  # 0  for normal version else beta version number
        self.consumer = False
        self.producer = True
        self.flim = False
        self.bootloader = False
        self.coe = False
        self.Function = myFunction
        print("New Node : " + str(nodeId))
        # self.Function(self.nodeId)
        self.SPI_ID = 0
        self.SPI_CLK = Pin(18)
        self.SPI_MOSI = Pin(19)
        self.SPI_MISO = Pin(16)

        self.SPI_CS = Pin(17)
        self.SPI_INT = Pin(20)
        self.OSC_2515 = 16000000
        self.button = Pin(16, Pin.IN, Pin.PULL_DOWN)
        # self.spi = SPI(self.SPI_ID, sck=self.SPI_CLK, mosi=self.SPI_MOSI, miso=self.SPI_MISO)
        self.spi = SPI(self.SPI_ID, sck=self.SPI_CLK, mosi=self.SPI_MOSI, miso=self.SPI_MISO, baudrate=10000000)
        print("SPI Configuration: " + str(self.spi) + '\n')  # Display SPI config

        self.can = cbus2515.Cbus2515(self.spi, self.SPI_CS, self.SPI_INT, osc=self.OSC_2515)
        time.sleep(0.2)
        self.can.change_mode(0)  # 0-Normal, 1-Sleep, 2-Loopback, 3-Listen Only, 4-Configuration
        # self.can.monitor()
        # self.header = self.get_header()
        # print("Header : "+self.header)
        self.intOut = 0
        self.chrOut = ""

    @staticmethod
    def pad(num, length):
        output = '0000000000' + hex(num)[2:]
        return output[length * -1:]

    @staticmethod
    def get_int(msg, start, length):
        return int(msg[start: start + length], 16)

    @staticmethod
    def get_str(msg, start, length):
        return msg[start: start + length]

    def get_op_code(self, msg):
        return self.get_str(msg, 7, 2)

    def get_node_id(self, msg):
        return int(self.get_str(msg, 9, 4), 16)

    def get_header(self):
        output = 0
        output = output + self.priority1
        output = output << 2
        output = output + self.priority2
        output = output << 7
        output = output + self.canId
        output = output << 5
        # print(str(output))
        # return str(output)
        # print (":S"+format(output, '02x')+"N")
        # return ":S"+format(output, '02x')+"N"
        return ":S" + hex(output)[2:] + "N"
        # return ":SB020N"

    def flags(self):
        flags = 0
        if self.consumer:
            flags += 1
        if self.producer:
            flags += 2
        if self.flim:
            flags += 4
        if self.bootloader:
            flags += 8
        if self.coe:
            flags += 16
        return flags

    def parameter(self, param):
        if self.debug:
            print("parameter : " + str(self.nodeId) + " : " + str(param) + " : " + str(self.parameters[param]))
        output = self.get_header() + "9B" + self.pad(self.nodeId, 4) + self.pad(param, 2) + self.parameters[param] + ";"
        if self.debug:
            print("parameter output : " + output)
        return output

    def pnn(self):
        flags = 0
        if self.consumer:
            flags += 1
        if self.producer:
            flags += 2
        if self.flim:
            flags += 4
        if self.bootloader:
            flags += 8
        if self.coe:
            flags += 16
        output = self.get_header() + "B6" + self.pad(self.nodeId, 4) + self.pad(self.manufId, 2) + self.pad(
            self.moduleId, 2) + self.pad(self.flags(), 2) + ";"
        self.send(output)

    def teach_long_event(self, node_id, event_id, variables):
        """
        Teaches a long CBUS event to the module
        :param node_id: node id of the event
        :param event_id: event od of the event
        :param variables: Variable that will be sent to the function when event
                is received. Can be String, number, list etc
        """
        new_id = self.pad(node_id, 4) + self.pad(event_id, 4)
        self.events[new_id] = variables
        if self.debug:
            print(self.events)

    def teach_short_event(self, event_id, variables):
        """
        Teaches a short CBUS event to the module
        :param event_id: event of the short event
        :param variables: Variable that will be sent to the function when event
               is received. Can be String, number, list etc
        """
        new_id = self.pad(0, 4) + self.pad(event_id, 4)
        self.events[new_id] = variables
        if self.debug:
            print(json.dumps(self.events, indent=4))

    def acon(self, event_id):
        """
        Sends a Accessory On Long Event to the CBUS Network
        :param event_id: Id for the event
        """
        output = self.get_header() + "90" + self.pad(self.nodeId, 4) + self.pad(event_id, 4) + ";"
        self.send(output)

    def acof(self, event_id):
        output = self.get_header() + "91" + self.pad(self.nodeId, 4) + self.pad(event_id, 4) + ";"
        self.send(output)

    def rloc(self, loco_id):
        output = self.get_header() + "40" + self.pad(loco_id, 4) + ";"
        self.send(output)

    def stmod(self, session_id, speed):
        output = self.get_header() + "47" + self.pad(session_id, 2) + self.pad(spee2, 4) + ";"
        self.send(output)

    def action_opcode(self, msg):
        def acc_on(msg):
            if self.debug:
                print("acc_on : " + msg + " Event : " + self.get_str(msg, 9, 8))
            if self.get_str(msg, 9, 8) in self.events:
                if self.debug:
                    print("Event is Known")
                # self.execute({'task': 'on', 'variables': self.events[self.get_str(msg, 9, 8)]})
                self.Function({'task': 'on', 'variables': self.events[self.get_str(msg, 9, 8)]})
            else:
                if self.debug:
                    print("Event is Unknown")

        def acc_off(msg):
            if self.debug:
                print("acc_off : " + msg)
            if self.get_str(msg, 9, 8) in self.events:
                if self.debug:
                    print("Event is Known")
                # self.execute({'task': 'off', 'variables': self.events[self.get_str(msg, 9, 8)]})
                self.Function({'task': 'off', 'variables': self.events[self.get_str(msg, 9, 8)]})
            else:
                if self.debug:
                    print("Event is Unknown")

        def paran(msg):
            parameter_id = self.get_int(msg, 13, 2)
            parameter_value = self.parameters[parameter_id]
            if self.debug:
                print("paran : " + msg + " nodeId : " + str(self.get_node_id(msg)))
            if self.get_node_id(msg) == self.nodeId:
                if self.debug:
                    print("paran for " + str(self.nodeId) +
                          " Parameter " + str(parameter_id) +
                          " Value : " + str(parameter_value))
                if self.debug:
                    print("Paran Output " + str(self.parameter(parameter_id)))
                # time.sleep(1)
                # self.parameter(parameter_id)
                self.send(str(self.parameter(parameter_id)))

        def qnn(msg):
            if self.debug:
                print("qnn : " + msg)
            self.pnn()

        def session_info(msg):
            if self.debug:
                print("E1 : PLOC - " + msg)
            self.Function({'task': 'dcc',
                           'sesson': self.get_str(msg, 9, 2),
                           'loco_id': self.get_str(msg, 11, 2)})

        opcode = self.get_op_code(msg)
        print("Opcode : " + opcode)
        action = {
            "90": acc_on,
            "91": acc_off,
            # "98": asc_on,
            # "99": asc_off,
            "E1": session_info,
            "73": paran,
            "0D": qnn
        }
        self.count += 1
        if self.debug:
            print("Msg Count" + str(self.count))
        if opcode in action:
            if self.debug:
                print("Processing Opcode : " + opcode)
            action[opcode](msg)
        else:
            if self.debug:
                print("Unknown Opcode : " + opcode)
            # self.Function(msg)

    def execute(self, msg):
        # self.Function(msg)
        print("Execute MSG : " + msg)
        self.action_opcode(msg)

    def send(self, msg):
        # print("Pico Node Send : " + msg)
        self.can.send(msg)

    def run(self):
        print("NODE RUN")
        while True:
            while self.can.in_waiting():
                # print(self.can.receive())
                self.execute(self.can.receive())
                # print("Check")
                time.sleep(0.1)
