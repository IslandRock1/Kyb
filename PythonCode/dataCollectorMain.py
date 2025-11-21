from blockSystemIdentification import BlockSystemIdentification
from robotController import RobotController

def main():
    robot = RobotController()
    robot.calibrate()

    system_id = BlockSystemIdentification(robot, "wrist", "wrist_data_2_min.csv", blocks = 2)
    system_id.run()

    robot.moveWristPID(0.0, timeout=5.0)
    robot.close()
if __name__ == "__main__":
    main()