import tm_libraries.tm_packet as tm_packet
import tm_libraries.tm_motion_functions_V1_80 as tm_motion_functions_V1_80
import tm_libraries.onrobot_tm as OR
from pymodbus.client import ModbusTcpClient
from tm_libraries.rich_logging_format import rich_logger

log = rich_logger()

class TM_Robot:
    """
    A robot class to manage the connection and send commands to the physical or simulated robot.

    Methods:

    """
    home = [100, 0.0, 100, 180, 0.0, 0.0]
    gripper = None

    def __init__(self, ip, table_name="Default"):
        self.TMSCT = None
        self.ip = ip
        self.modbus = ModbusTcpClient(host=ip, port=502)
        self.modbus.connect()
        self.TMSVR = tm_packet.TMSVR(ip, table_name)
        log.info("Succesfully connected to robot Ethernet Slave and Modbus.")
        self.motion_functions = tm_motion_functions_V1_80.TM_Motion_Functions()
        self.gripper = None

        self._tcp_coord = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.home = self.tcp_coord

    def connect_listen_node(self, ip=None):
        if ip is not None:
            self.ip = ip
        self.TMSCT = tm_packet.TMSCT(self.ip)
        log.info("Connected to Listen Node")
        if self.TMSCT:
            return True
        else:
            return False

    @property
    def tcp_coord(self):
        x_response = self.modbus.read_input_registers(7025, count=12)
        self._tcp_coord = self.modbus.convert_from_registers(x_response.registers, self.modbus.DATATYPE.FLOAT32, "big")
        return self._tcp_coord

    @property
    def joints(self):
        x_response = self.modbus.read_input_registers(7013, count=12)
        self._joints = self.modbus.convert_from_registers(x_response.registers, self.modbus.DATATYPE.FLOAT32, "big")
        return self._joints

    def close_connection(self):
        # TODO: Add a check to see if TMSCT exists
        self.modbus.close()
        self.TMSVR.close()
        if self.TMSCT is not None:
            self.TMSCT.close()

    def gripper_setup(self, gripper_model: str = "VGC10"):
        if gripper_model.upper() == "VGC10":
            self.gripper = OR.TM_VGC10()
            self.TMSCT.send(self.gripper.setup_script)
        print(f"{gripper_model} gripper correctly setup")

    def grip(self):
        if self.gripper is None:
            print("Gripper not connected")
            return
        self.TMSCT.send(self.gripper.grip_script)
        # print("Gripping")

    def release(self):
        if self.gripper is None:
            print("Gripper not connected")
            return
        self.TMSCT.send(self.gripper.release_script)
        # print("Releasing")

    # def idle(self):
    #     if self.gripper is None:
    #         print("Gripper not connected")
    #         return
    #     self.TMSCT.send(self.gripper.idle_script())
    #     print("Idle")

    def svr_write(self, item, value):
        """Changes a value in the ethernet slave table through ethernet slave.
        :param item: string of the item name
        :param value: target value of the item
        :return:
        """
        self.TMSVR.send(item, value)

    def listen_svr_write(self, item, value):
        """Changes a value in the ethernet slave table through a script sent to the listen node.
        :param item:
        :param value:
        :return:
        """
        self.TMSCT.send(f"svr_write({item},{value})")

    def path_from_csv(self, file_path, speed):
        poses = []
        with open(file_path) as path:
            for pose in path.readlines():
                pose = pose.strip("\n")
                pose = pose.split(",")
                if len(pose) == 1:
                    pose = pose[0].split(" ")
                poses.append(pose)
            # print(poses)
        self.path(poses, speed)

    def path(self, poses, speed):
        self.line(poses, speed)
        print("Path in execution")

    def go_home(self, speed, home_p=None):
        if home_p is None:
            home_p = self.home
        self.home = home_p
        commands = self.ptp(home_p, speed)
        self.TMSCT.send(commands)
        print("Going home")

    def wait_queue_tag(self, mode=''):
        self.TMSCT.send(f"WaitQueueTag({mode})", queue=False)
        print("Waiting for queue tag")

    def queue_tag(self, tag_number):
        self.TMSCT.send(f"QueueTag({tag_number}, 1)", queue=False)
        print("Waiting for queue tag")

    def stop(self, mode=''):
        """Stop the robot and clear the buffer
        :param mode: 0 to stop the robot and clear the buffer, 1 to stop the robot and continue to the next script program, 2 to stop the robot and clear all script programs
        """
        self.TMSCT.send(f"StopAndClearBuffer({mode})", queue=False)
        print("Stopped")

    def exit(self, mode=''):
        """Exit the listen node
        :param mode: 0 to exit the listen node and go to the fail branch, 1 to exit the listen node and go to the pass branch
        """
        # print(f"ScriptExit({mode})")
        self.TMSCT.send(f"ScriptExit({mode})", queue=False)
        print("Exiting")

    def ptp(self, poses, speed, queue=False, **kwargs):
        """Builds the pline script using the poses provided

        :param data_format: string representing the data format. Can be "CPP" or "JPP"
        :type data_format: str
        :param precision_positioning: activate precision positioning
        :type precision_positioning: str
        :param poses: matrix (list[list]) of the poses (x, y, z, rx, ry, rz) or (J1, J2, J3, J4, J5, J6)
        :type poses: list|list[list]
        :param speed: speed expressed as percentage
        :type speed: int
        :param blending: blending value expressed in percentage
        :type blending: int
        :param time_acc: time interval to accelerate to top speed (ms)
        :type time_acc: int
        :return:
        """
        self.TMSCT.send(self.motion_functions.ptp(poses, speed, **kwargs), queue=queue)


    def pline(self, poses, speed, queue=False, **kwargs):
        """Builds the pline script using the poses provided

        :param data_format: string representing the data format. Can be "CAP" or
        :param poses: matrix (list of list) of the poses (x, y, z, rx, ry, rz)
        :type poses: list
        :param speed: target velocity in mm/s, list of integers with the same length of poses
        or single integer for constant velocity throughout
        :type speed: int|str|list[int]|list[str]
        :param blending: blending value expressed in percentage
        :type blending: int
        :param time_acc: time interval to accelerate to top speed (ms)
        :type time_acc: int
        :return:
        """
        self.TMSCT.send(self.motion_functions.pline(poses, speed, **kwargs), queue=queue)

    def line(self, poses, speed, queue=False, **kwargs):
        """Builds the pline script using the poses provided

        :param poses: matrix (list of list) of the poses (x, y, z, rx, ry, rz)
        :type poses: list
        :param speed: target velocity in mm/s, list of integers with the same length of poses
        or single integer for constant velocity throughout
        :type speed: int|str|list[int]|list[str]
        :param queue: boolean to determine if the command should be queued

        :return:
        """
        self.TMSCT.send(self.motion_functions.line(poses, speed, **kwargs), queue=queue)

    def circle(self, mid_point, end_point, speed, queue=False, **kwargs):

        self.TMSCT.send(self.motion_functions.circle(mid_point, end_point, speed, **kwargs), queue=queue)

    def move_ptp(self, poses, speed, queue=False, **kwargs):
        """Builds the pline script using the poses provided

        :param poses: matrix (list of list) of the poses (x, y, z, rx, ry, rz)
        :type poses: list
        :param speed: target velocity in mm/s, list of integers with the same length of poses
        or single integer for constant velocity throughout
        :type speed: int|str|list[int]|list[str]
        :return:
        """
        self.TMSCT.send(self.motion_functions.move_ptp(poses, speed, **kwargs), queue=queue)


    def move_line(self, poses, speed, queue=False, **kwargs):
        self.TMSCT.send(self.motion_functions.move_ptp(poses, speed, **kwargs), queue=queue)

    def wait(self, milli_sec):
        self.TMSCT.send([f"WaitFor({milli_sec})"])
        # not supported by the TMFlow 1.80


if __name__ == "__main__":
    robot = TM_Robot("127.0.0.1")
    # time.sleep(10)
    robot.close_connection()
