import struct
import socket
import time
from abc import ABC, abstractmethod
from datetime import datetime
import logging
from rich.logging import RichHandler
import threading
import csv
import json
import os

FORMAT = '%(message)s'
logging.basicConfig(
    level=logging.INFO, format=FORMAT, datefmt="[%X]", handlers=[RichHandler()]
)

log = logging.getLogger("__name__")


class TMPacket(ABC):
    P_HEAD = b'\x24'  # '$'
    P_END1 = b'\x0D'  # '\r'
    P_END2 = b'\x0A'  # '\n'
    P_SEPR = b'\x2C'  # ','
    P_CSUM = b'\x2A'  # '*'
    P_LSEP = b'\x3B'  # ';'

    def __init__(self):
        self.data = None
        self.header = None
        self.length = None
        self.data_block = None
        self.checksum = None

    @abstractmethod
    def send(self, *args, **kwargs):
        pass

    @abstractmethod
    def recv(self):
        pass

    @abstractmethod
    def close(self):
        pass

    @staticmethod
    def checksum_calc(data_msg):
        # data_msg = data_msg.encode('utf-8')
        csum = 0
        for el in data_msg:
            csum ^= el
        return hex(csum)[2:].zfill(2).encode('utf-8').upper()

    def deserialize(self):
        max_ind = len(self.data)
        index = 0
        # Find the beginning of the packet
        while index < max_ind:
            if bytes([self.data[index]]) == TMPacket.P_HEAD:
                break
            index += 1

        index += 1
        index_i = index
        # Now index is the index of the first character of the header
        # Find packet header
        while index < max_ind:
            if bytes([self.data[index]]) == TMPacket.P_SEPR:
                index_end = index
                break
            index += 1

        # now index is at separator (',') after header
        self.header = self.data[index_i:index].decode('utf-8')
        # print(f"header: {self.header} \n")

        index += 1
        index_i = index
        # Now index is the first byte of the length of data
        # Find packet length
        while index < max_ind:
            if bytes([self.data[index]]) == TMPacket.P_SEPR:
                break
            index += 1

        # now index is at separator (',') after packet length
        self.length = int(self.data[index_i:index].decode('utf-8'))
        # print(f"Data length: {self.length} \n")

        index += 1
        index_i = index

        # Now index is the first byte of the data
        # Find packet data
        index = index + self.length
        self.data_block = self.data[index_i:index]
        # Checksum
        index += 1
        # print(f"csum: {bytes([self.data[index]]).decode('utf-8')} \n")
        # print(self.data[index + 1:index + 3].decode('utf-8'))
        try:
            if bytes([self.data[index]]) == TMPacket.P_CSUM:                    # if data length is greater than the number of bytes in the buffer return False
                self.checksum = self.data[index + 1:index + 3].decode('utf-8')
        except IndexError:
            return False

        index += 5
        c_sum = self.checksum_calc(f"{self.header},{self.length},".encode('utf-8') + self.data_block + b',')
        if int(self.checksum, 16) != int(c_sum, 16):
            print("Checksum is not correct")
        # print(f"Data: {self.data[index_i:index]} \n")
        # self.get_item_values(self.data[index_i:index])
        # index += 7

        del self.data[:index]
        return True
    @abstractmethod
    def parse_data(self):
        pass


class ethernet_table:
    def __init__(self, filename: str):
        self.state = {"Robot_Link": ["?", False],
                      "Current_Time": ["s", "2099-06-25T15:45:30.123"],
                      "dt": ["i", 0],
                      "Joint_Angle": ["f", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
                      "Coord_Base_Tool": ["f", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
                      }
        if not os.path.exists("ethernet_tables"):
            os.makedirs("ethernet_tables")
        self.dir = "ethernet_tables/"
        self.filename = filename.strip(".json") + ".json"
        self.load_ethernet_table()

    def load_ethernet_table(self):
        while True:
            try:
                self.state = json.load(open(f"{self.dir}{self.filename}")) | self.state
                return
            except FileNotFoundError:
                ans = input(
                    f"Could not find the ethernet table file named {self.filename}. Create {self.filename} file? y/n")
                if ans.lower() == 'y':
                    json.dump(self.state, open(f"{self.dir}{self.filename}", 'w'))
                    return
                else:
                    self.filename = input("Enter the filename of the ethernet table: ").strip(".json") + ".json"

    def save_ethernet_table(self):
        while True:
            ans = input(f"Save the new ethernet table as {self.filename}? y/n")
            if ans.lower().strip(" ") == 'y':
                json.dump(self.state, open(f"{self.dir}{self.filename}", 'w'))
                print("Ethernet table saved")
                return
            else:
                self.filename = input("Enter the new name for the table file: ").strip(".json") + ".json"


class TMSVR(TMPacket):

    def __init__(self, ip, table_name="Default.json"):
        super().__init__()

        self.buffer_size = 2048
        self.data = bytearray(b'')

        # For reading the ethernet table
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((ip, 5891))
        self.recv()
        self.table = ethernet_table(table_name)
        self.state = self.table.state
        self.check_ethernet_items()

        self.data_length = len(self.data)

        # For updating the state with a thread
        self.updating = True
        self.my_event = threading.Event()
        self.state_update_thread = None
        self.start_update()

        # For writing data to a file
        self.logging = False
        self.file_name = "temp_log.txt"
        self.file = None
        self.items = ["Current_Time"]


    def get_received_table_items(self):
        self.deserialize()
        item_names = []
        item_sizes = []
        max_indx = len(self.data_block)
        index = 0
        # Find ID
        while index < max_indx:
            if bytes([self.data_block[index]]) == TMPacket.P_SEPR:
                break
            index += 1

        if (self.data_block[:index].decode('utf-8') == "svr"):
            return

        index += 1
        index_i = index

        # Find Mode
        while index < max_indx:
            if bytes([self.data_block[index]]) == TMPacket.P_SEPR:
                break
            index += 1

        index += 1
        index_i = index
        while index < max_indx:
            index += 2

            item_length = struct.unpack('<H', self.data_block[index_i:index])[0]

            index_i = index
            index += item_length
            item_name = self.data_block[index_i:index].decode('utf-8')

            index_i = index
            index += 2
            item_length = struct.unpack('<H', self.data_block[index_i:index])[0]

            index += item_length

            item_names.append(item_name)
            item_sizes.append(item_length)

            index_i = index
        return item_names, item_sizes

    def check_ethernet_items(self):
        new_items = False
        received_table_names, received_item_sizes = self.get_received_table_items()
        i = 0
        for item in received_table_names:
            if item not in self.state:
                while True:
                    d_type = input(f"Please enter the data type of the item {item} ({received_item_sizes[i]} bytes): \n"
                                   "s - string\n"
                                   "f - float\n"
                                   "i - integer\n"
                                   "? - boolean\n").lower().strip(" ")

                    if d_type not in ['s', 'f', 'i', '?']:
                        print("Invalid data type")
                    else:
                        self.state[item] = [d_type, None]
                        break
                new_items = True
            i += 1
        if new_items:
            self.table.save_ethernet_table()

    def send(self, item_name, value, script_id="svr"):
        script_data = script_id + ",2,"
        if isinstance(value, list):
            value_str = ""
            for v in value:
                value_str = value_str + f"{v},"
            value_str = "{" + value_str[:-1] + "}"
            script_data = script_data + f"{item_name}=" + value_str + "\r\n"
        else:
            script_data = script_data + f"{item_name}={value}\r\n"
        # log.info(script_data)
        data_length = len(script_data)
        # Convert string to bytes
        self.data_block = "TMSVR" + "," + str(data_length) + ',' + script_data + ','
        data_msg = self.data_block.encode("utf-8")

        csum = self.checksum_calc(data_msg)
        msg = b'$' + data_msg + b'*' + csum + b'\r\n'
        # print(msg)
        self.sock.send(msg)

    def recv(self):
        self.data += (bytearray(self.sock.recv(self.buffer_size)))

    def close(self):
        if self.updating:
            self.stop_update()
        time.sleep(1)
        self.sock.close()

    def state_update(self):
        self.updating = True
        self.clear()
        while self.updating:
            self.recv()
            while len(self.data) > self.data_length:
                if self.deserialize():
                    self.parse_data()
                    if self.logging:
                        combined_list = []
                        for item in self.items:
                            var = self.state[item][1]
                            if isinstance(var, list):
                                combined_list.extend(var)
                            else:
                                combined_list.append(var)
                        writer = csv.writer(self.file)
                        writer.writerow(combined_list)
                else:
                    break
            if self.my_event.is_set():
                break

    def start_logging(self, filename=None, items: list = None, mode='a'):
        if filename is not None:
            self.file_name = filename
        if items is not None:
            self.items = items
        self.file = open(self.file_name, mode, newline='')
        self.logging = True

    def stop_logging(self):
        self.logging = False
        self.file.close()

    def start_update(self):
        self.clear()
        self.state_update_thread = threading.Thread(target=self.state_update)
        self.state_update_thread.start()

    def stop_update(self):
        if self.logging:
            self.stop_logging()
        self.my_event.set()
        self.updating = False

    def clear(self):
        while len(self.sock.recv(self.buffer_size)) == self.buffer_size:
            pass
        self.data = bytearray(b'')

    def parse_data(self):
        max_indx = len(self.data_block)
        index = 0
        # Find ID
        while index < max_indx:
            if bytes([self.data_block[index]]) == TMPacket.P_SEPR:
                break
            index += 1

        if (self.data_block[:index].decode('utf-8') == "svr"):
            return

        index += 1

        # Find Mode
        while index < max_indx:
            if bytes([self.data_block[index]]) == TMPacket.P_SEPR:
                break
            index += 1

        index += 1
        index_i = index
        while index < max_indx:
            index += 2

            item_length = struct.unpack('<H', self.data_block[index_i:index])[0]

            index_i = index
            index += item_length
            item_name = self.data_block[index_i:index].decode('utf-8')

            index_i = index
            index += 2
            item_length = struct.unpack('<H', self.data_block[index_i:index])[0]

            index_i = index
            index += item_length

            self.decoder(item_name, self.data_block[index_i:index])
            index_i = index

    @staticmethod
    def dt_calc(time1, time2):
        timestamp_format = "%Y-%m-%dT%H:%M:%S.%f"
        dt1 = datetime.strptime(time1, timestamp_format)
        dt2 = datetime.strptime(time2, timestamp_format)

        # Calculate the difference between the two datetime objects
        delta = dt2 - dt1
        if delta.total_seconds() < 0:
            # print("Time1 is greater than Time2")
            return 0
        else:
            # Convert the difference to milliseconds
            return delta.total_seconds() * 1000

    def decoder(self, item_name, bytes_data):
        data_type = self.state[item_name][0]
        value = None

        if data_type == 's':
            value = bytes_data.decode('utf-8')
            if item_name == "Current_Time":
                self.state["dt"][1] = self.dt_calc(self.state["Current_Time"][1], value)
        elif data_type == '?':
            value = struct.unpack('?', bytes_data)[0]
        elif data_type == 'f':
            n = int(len(bytes_data) / 4)
            if n != 1:
                value = list(struct.unpack(f'{n}f', bytes_data))
            else:
                value = struct.unpack('f', bytes_data)[0]
        elif data_type == 'i':
            n = int(len(bytes_data) / 4)
            if n != 1:
                value = list(struct.unpack(f'{n}i', bytes_data))
            else:
                value = struct.unpack('i', bytes_data)[0]
        else:
            print("Unknown data type")

        self.state[item_name][1] = value


class TMSCT(TMPacket):
    def __init__(self, ip):
        super().__init__()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((ip, 5890))
        self.data = ""

        self.header = "TMSCT"
        self.ID = 0

    def send(self, commands, script_id=None, queue=False):

        if script_id is None:
            script_id = self.ID


        script_data = f"{script_id},"

        if script_id == 0:
            queue_script = [f"QueueTag(9)"]
        else:
            queue_script = [f"QueueTag({script_id-1})"]

        if queue:
            script_data = script_data + queue_script[0] + "\r\n"

        if isinstance(commands, list):
            # if queue:
            #     commands.extend(queue_script)
            for command in commands:
                if command == commands[-1]:
                    script_data = script_data + command
                else:
                    script_data = script_data + command + "\r\n"
        else:
            script_data = script_data + commands + "\r\n"
            # if queue:
            #     script_data = script_data + "\r\n" + queue_script[0]

        data_length = len(script_data)
        # Convert string to bytes
        self.data_block = self.header + "," + str(data_length) + ',' + script_data + ','
        data_msg = self.data_block.encode("utf-8")

        if self.ID < 9:
            self.ID += 1
        else:
            self.ID = 0
        csum = self.checksum_calc(data_msg)
        msg = b'$' + data_msg + b'*' + csum + b'\r\n'
        self.sock.send(msg)

    def listen_ready(self):
        self.sock.send(b'$TMSTA,2,00,*41\r\n')
        self.data = self.sock.recv(2048).decode("utf-8")
        data_list = self.data.split(",")
        # print(data_list)
        while data_list[0] != "$TMSTA":
            self.data = self.sock.recv(2048).decode("utf-8")
            data_list = self.data.split(",")
        print(data_list)

        if data_list[3] == "true":
            return [True, data_list[4]]
        else:
            return [False, data_list[4]]

    def parse_data(self):
        pass

    def recv(self):
        self.sock.recv()
        # self.data = listen_node.data

    def close(self):
        self.sock.close()
