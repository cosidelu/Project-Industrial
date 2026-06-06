import techman as tm
import time

def main():
    # Before running the TMFlow program, make sure the Modbus TCP and Ethernet slave are enabled in configuration->connection
    # You can find the IP address of the robot in system->network
    # The first time you run the program, you will be asked to create an ethernet table in order to use the ethernet slave.
    robot_ip = "127.0.0.1"
    TM5 = tm.TM_Robot(robot_ip)

    # Run the TMFlow program with the listen node before running the connect_listen_node function
    TM5.connect_listen_node()

    P1 = [370, 300, 60, 150, 0, 90]
    P2 = [370, 600, 60, 150, 0, 90]
    P3 = [500, 600, 60, 150, 0, 90]
    path = input("Press enter to start")

    for i in range(3):
        TM5.ptp(P1, 10)

        input("press enter to scan")
        TM5.line(P2, 100)
        time.sleep(0.5)

        P1[2] += 30
        P2[2] += 30

        time.sleep(1)
        joints = TM5.joints
        pose = TM5.tcp_coord
        print("Joint space: ", joints)
        print("Cartesian space: ", pose)

    input("press enter to draw circle")

    TM5.circle(P3, P1, 100)

    input("press enter to close connection")

    TM5.close_connection()

if __name__ == "__main__":
    main()