from robot_arm_interface.airbot_wrapper import AirbotWrapper


def main():
    arm = AirbotWrapper(url="localhost", port=50001)
    arm.connect()

    print("state:", arm.get_state())
    print("joint_pos:", arm.get_joint_pos())
    print("joint_vel:", arm.get_joint_vel())
    print("end_pose:", arm.get_end_pose())

    arm.disconnect()


if __name__ == "__main__":
    main()
