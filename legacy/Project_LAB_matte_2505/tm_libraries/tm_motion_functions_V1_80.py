import numpy as np

class MotionOptions:

    def set(self, motion_command, print_settings=False, **kwargs):
        """Set the options for the motion command"""
        motion_command = motion_command.lower()
        if hasattr(self, motion_command):       # check if the motion command exists
            if self.check_args(motion_command, kwargs.items()):   # check if the keys and values are valid
                a = getattr(self, motion_command)   # get the dictionary of the motion command
                for key, value in kwargs.items():   # set the values
                    if key == "data_format":
                        value = value.upper()
                    a[key] = value
                if print_settings:
                    print("\nCurrent options are set as:")
                    for key, value in a.items():
                        print(f"{key} = {value}")
                    print(self.data_format_parser(a["data_format"]))
                return True
        else:
            print(f"Invalid motion command: {motion_command}")
            return False

    def data_format_parser(self, data_format):
        return (f"The current data format takes coordinates as {self.coordinate_format[data_format[0]]}, the speed is "
                f"expressed as {self.speed_format[data_format[1]]}, the blending is expressed as "
                f"{self.blending_format[data_format[2]]}")

    def check_args(self, motion, args):
        """Check if the arguments are valid"""
        for key, value in args:
            if key == "data_format":
                if value.upper() not in self.supported_data_formats[motion]:
                    print(f"Invalid data format: {value}. Must be one of {self.supported_data_formats[motion]}")
                    return False
            elif key == "blending":
                if not isinstance(value, int):
                    print(f"Invalid blending value: {value}. Must be an integer")
                    return False
            elif key == "time_acc":
                if not isinstance(value, int):
                    print(f"Invalid time_acc value: {value}. Must be an integer")
                    return False
            elif key == "precision_positioning":
                if value not in ["true", "false"]:
                    print(f"Invalid precision_positioning value: {value}. Must be 'true' or 'false'")
                    return False
            else:
                print(f"Invalid option: {key}")
                return False
        return True

    coordinate_format = {
        "J": "JOINT ANGLES",
        "C": "current CARTESIAN REFERENCE frame coordinates",
        "T": "current TOOL REFERENCE frame coordinates"
    }

    speed_format = {
        "P": "PERCENTAGE",
        "A": "VELOCITY [mm/s] (affected by project speed)",
        "D": "VELOCITY [mm/s] (not affected by project speed, does not work in V1.8)",
    }

    blending_format = {
        "P": "PERCENTAGE",
        "D": "RADIUS [mm]",
    }

    supported_data_formats = {
        "ptp": ["JPP", "CPP"],
        "line": ["CPP", "CDP", "CAP", "CAR", "CDP", "CDR"],
        "pline": ["CAP", "CDP", "JAP", "JDP"],
        "circle": ["CPP", "CAP", "CDP"],
        "move_ptp": ["CPP", "TPP", "JPP"],
        "move_line": ["CAP", "TAP", "JAP"],
        "move_pline": ["CAP", "TAP", "JAP"],
    }

    ptp = {
        "data_format": "CPP",
        "blending": 100,
        "time_acc": 200,
        "precision_positioning": "true"
    }

    line = {
        "data_format": "CAP",
        "blending": 100,
        "time_acc": 200,
        "precision_positioning": "true"
    }

    pline = {
        "data_format": "CAP",
        "blending": 5,
        "time_acc": 200,
    }

    circle = {
        "data_format": "CAP",
        "blending": 100,
        "time_acc": 200,
        "arc_angle": 0,
        "precision_positioning": "true"
    }

    move_ptp = {
        "data_format": "CPP",
        "blending": 100,
        "time_acc": 200,
        "precision_positioning": "true"
    }

    move_line = {
        "data_format": "CAP",
        "blending": 100,
        "time_acc": 200,
        "precision_positioning": "true"
    }

    move_pline = {
        "data_format": "CAP",
        "blending": 100,
        "time_acc": 200,
        "precision_positioning": "true"
    }


class TM_Motion_Functions:
    def __init__(self):
        self.options = MotionOptions()

    def joint_limit_check(self):
        # TODO: check joint limits with inverse kinematics (although TMFlow already does this)
        return True

    @staticmethod
    def queue_tag(tag, wait=0):
        return f"QueueTag({tag}, {wait})"

    @staticmethod
    def wait_queue_tag(tag):
        return f"WaitQueueTag({tag})"
    #  V2.16 can be used without tag to wait for all tags to finish

    @staticmethod
    def stop(mode=''):
        """Stop the robot and clear the buffer
        :param mode: 0 to stop the robot and clear the buffer, 1 to stop the robot and continue to the next script
        program, 2 to stop the robot and clear all script programs
        """
        return f"StopAndClearBuffer({mode})"
    #    TODO: check if the modes work in the 1.8 version as well

    @staticmethod
    def pause():
        """Pause the robot"""
        return "Pause()"

    @staticmethod
    def resume():
        """Resume the robot"""
        return "Resume()"

    def ptp(self, poses, speed, **kwargs):
        """Builds the pline script using the poses provided

        :param poses: matrix (list[list]) of the poses (x, y, z, rx, ry, rz) or (J1, J2, J3, J4, J5, J6)
        :type poses: list|list[list]
        :param speed: speed expressed as percentage
        :type speed: int
        """
        self.options.set("ptp", **kwargs)
        data_format = self.options.ptp["data_format"]
        blending = self.options.ptp["blending"]
        time_acc = self.options.ptp["time_acc"]
        precision_positioning = self.options.ptp["precision_positioning"]

        poses_str = self.poses_to_str(poses)

        lines = []
        for pose in poses_str:
            lines.append(
                f"PTP({data_format},{pose},{speed},{time_acc},"
                f"{blending},{precision_positioning})")
        return lines

    def pline(self, poses, speed, **kwargs):
        """Builds the pline script using the poses provided

        :param poses: matrix (list of list) of the poses (x, y, z, rx, ry, rz)
        :type poses: list
        :param speed: target velocity in mm/s, list of integers with the same length of poses
        or single integer for constant velocity throughout
        :type speed: int|str|list[int]|list[str]
        """
        self.options.set("pline", **kwargs)
        data_format = self.options.pline["data_format"]
        blending = self.options.pline["blending"]
        time_acc = self.options.pline["time_acc"]

        poses_str = self.poses_to_str(poses)

        lines = []
        for pose in poses_str:
            lines.append(
                f"PLine({data_format},{pose},{speed},{time_acc},{blending})")
        return lines

    def line(self, poses, speed, **kwargs):
        """Builds the pline script using the poses provided

        :param poses: matrix (list of list) of the poses (x, y, z, rx, ry, rz)
        :type poses: list
        :param speed: target velocity in mm/s, list of integers with the same length of poses
        or single integer for constant velocity throughout
        :type speed: int|str|list[int]|list[str]

        :return:
        """
        self.options.set("line", **kwargs)
        data_format = self.options.line["data_format"]
        blending = self.options.line["blending"]
        time_acc = self.options.line["time_acc"]
        precision_positioning = self.options.line["precision_positioning"]

        poses_str = self.poses_to_str(poses)

        lines = []
        for pose in poses_str:
            lines.append(f"Line({data_format},{pose},{speed},{time_acc},{blending},{precision_positioning})")

        return lines

    # TODO: implement circle function
    def circle(self, mid_point, end_point, speed, **kwargs):
        self.options.set("circle", **kwargs)
        data_format = self.options.circle["data_format"]
        blending = self.options.circle["blending"]
        time_acc = self.options.circle["time_acc"]
        arc_angle = self.options.circle["arc_angle"]
        precision_positioning = self.options.circle["precision_positioning"]

        mid_point = self.poses_to_str(mid_point)
        end_point = self.poses_to_str(end_point)

        lines = [
            f"Circle({data_format},{mid_point[0]},{end_point[0]},"
            f"{speed},{time_acc},{blending},{arc_angle},{precision_positioning})"]
        return lines

    def move_ptp(self, poses, speed, **kwargs):
        """Builds the pline script using the poses provided

        :param poses: matrix (list of list) of the poses (x, y, z, rx, ry, rz)
        :type poses: list
        :param speed: target velocity in mm/s, list of integers with the same length of poses
        or single integer for constant velocity throughout
        :type speed: int|str|list[int]|list[str]
        :return:
        """
        self.options.set("move_ptp", **kwargs)
        data_format = self.options.move_ptp["data_format"]
        blending = self.options.move_ptp["blending"]
        time_acc = self.options.move_ptp["time_acc"]
        precision_positioning = self.options.move_ptp["precision_positioning"]

        poses_str = self.poses_to_str(poses)

        lines = []
        for pose in poses_str:
            lines.append(f"Move_PTP({data_format},{pose},{speed},{time_acc},{blending},{precision_positioning})")

        return lines

    def move_line(self, poses, speed, **kwargs):

        self.options.set("move_line", **kwargs)
        data_format = self.options.move_line["data_format"]
        blending = self.options.move_line["blending"]
        time_acc = self.options.move_line["time_acc"]
        precision_positioning = self.options.move_line["precision_positioning"]

        poses_str = self.poses_to_str(poses)

        lines = []
        for pose in poses_str:
            lines.append(f"Move_Line({data_format},{pose},{speed},{time_acc},{blending},{precision_positioning})")

        return lines

    def move_pline(self, poses, speed, **kwargs):

        self.options.set("move_pline", **kwargs)
        data_format = self.options.move_pline["data_format"]
        blending = self.options.move_pline["blending"]
        time_acc = self.options.move_pline["time_acc"]

        poses_str = self.poses_to_str(poses)

        lines = []
        for pose in poses_str:
            lines.append(f"Move_PLine({data_format},{pose},{speed},{time_acc},{blending})")

        return lines

    @staticmethod
    def exit(mode=''):
        """Exit the listen node
        :param mode: 0 to exit the listen node and go to the fail branch, 1 to exit the listen node and go to the pass branch
        """
        # print(f"ScriptExit({mode})")
        return f"ScriptExit({mode})"

    @staticmethod
    def wait(self, milli_sec):
        return f"WaitFor({milli_sec})"

    @staticmethod
    def poses_to_str(poses):
        if isinstance(poses, np.ndarray):
            poses = poses.tolist()
        arr = []
        if isinstance(poses, list) and isinstance(poses[0], list):
            for pose in poses:
                arr.append(f"{pose[0]},{pose[1]},{pose[2]},{pose[3]},{pose[4]},{pose[5]}")
        else:
            arr.append(f"{poses[0]},{poses[1]},{poses[2]},{poses[3]},{poses[4]},{poses[5]}")
        return arr


if __name__ == "__main__":
    tm = TM_Motion_Functions()
    # print(tm.ptp([[1, 2, 3, 4, 5, 6],[1, 2, 3, 4, 5, 6]], 100, blending=50, time_acc=10, precision_positioning="false"))
    # print(tm.line([[1, 2, 3, 4, 5, 6]], 100))
    # print(tm.pline([1, 2, 3, 4, 5, 6], 100, blending=50, time_acc=10, data_format="cPP"))
    # print(tm.circle([1, 2, 3, 4, 5, 6], [7, 8, 9, 10, 11, 12], 100))
    # print(tm.move_ptp([[1, 2, 3, 4, 5, 6]], 100))
    # print(tm.move_line([[1, 2, 3, 4, 5, 6]], 100))
    # print(tm.move_pline([[1, 2, 3, 4, 5, 6], [1, 2, 3, 4, 5, 6], [1, 2, 3, 4, 5, 6]], 100, data_format="tap"))
    # print(tm.wait(1000))
    # print(tm.exit(0))
    # print(tm.stop())
    # print(tm.pause())
    # print(tm.resume()
    a = []
    if a:
        print("True")

    if not a:
        print("False")
