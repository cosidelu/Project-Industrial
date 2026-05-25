from abc import ABC, abstractmethod

class OnRobotGripper(ABC):
    @property
    @abstractmethod
    def setup_script(self):
        pass

    @property
    @abstractmethod
    def grip_script(target_vacuum):
        pass

    @property
    @abstractmethod
    def release_script(self):
        pass

class TM_VGC10(OnRobotGripper):
    def __init__(self, ip = "192.168.1.1"):
        self.ip = ip
        self.port = 502
        self.TCP_offset = "{0, -27, 179, 0, 0, 0}"

    @property
    def setup_script(self):
        # return (f"ModbusTCP VGC10 = \"{self.ip}\",502\r\nVGC10.Preset(\"command\",65,\"RO\",0,\"int16\")\r\n"
        #         f"VGC10.Preset(\"read\",65,\"RO\",258,\"int16\")\r\n"
        #         f"TCP[\"PRINGripper\"].Value = {self.TCP_offset}\r\n"
        #         "ChangeTCP(\"PRINGripper\")")

        return (f"TCP[\"PRINGripper\"].Value = {self.TCP_offset}\r\n"
                "ChangeTCP(\"PRINGripper\")")

    @property
    def grip_script(self,target_vacuum = 50):
        mode = 0x0100
        value = mode + target_vacuum
        message = [f"QueueTag(11, 1)\r\n"
                   f"modbus_write(\"mtcp_VGC10\",\"preset_command\", {value})"]
        return message

    @property
    def release_script(self):
        message = [f"QueueTag(12, 1)\r\n"
                   f"modbus_write(\"mtcp_VGC10\",\"preset_command\",0)"]
        return message

    @staticmethod
    def idle_script():
        message = ["modbus_write(\"mtcp_VGC10\",\"preset_command\",512)"]
        return message
