#!/usr/bin/python3
import signal
import sys
from threading import Thread
import rclpy
from frc_robot_utilities_ros2_py_node.BufferedROSMsgHandlerPy import BufferedROSMsgHandlerPy
from health_monitor_ros2_sample_node.generated.parameters import ParameterizedNode

from std_msgs.msg import String

from ck_ros2_msgs_node.msg import HealthMonitorStatus, HealthMonitorControl
from ck_ros2_base_msgs_node.msg import MotorStatusArray, MotorStatus

from typing import List

class LocalNode(ParameterizedNode):
    def __init__(self):
        super().__init__('health_monitor_ros2_sample_node')
        self.status_publisher = self.create_publisher(topic="HealthMonitorStatus", msg_type=HealthMonitorStatus, qos=10)

        self.fault_list = []

        self.control_subscriber = BufferedROSMsgHandlerPy(HealthMonitorControl)
        self.control_subscriber.register_for_updates("HealthMonitorControl")

        # self.arm_subscriber = BufferedROSMsgHandlerPy(Arm_Status)
        # self.arm_subscriber.register_for_updates("ArmStatus")

        # self.intake_subscriber = BufferedROSMsgHandlerPy(Intake_Status)
        # self.intake_subscriber.register_for_updates("IntakeStatus")

        self.motor_status_subscriber = BufferedROSMsgHandlerPy(MotorStatusArray)
        self.motor_status_subscriber.register_for_updates("MotorStatus")


        self.loop_thread = Thread(target=self.loop)
        self.loop_rate = self.create_rate(20, self.get_clock())


    def loop(self) -> None:
        """
        Periodic function for the health monitor node.
        """

        frame_counter = 0
        ros_fully_booted = False
        while rclpy.ok():
            if frame_counter % 10 == 0:
                ros_fully_booted = True
                node_list = self.get_node_names()
                for s in self.Parameters.node_checklist:
                    ros_fully_booted &= s in node_list

                motor_status : MotorStatusArray = self.motor_status_subscriber.get()
                if motor_status is not None:
                    motor_list : List[MotorStatus] = motor_status.motors
                    motor_id_list : List[int] = []

                    for m in motor_list:
                        motor_id_list.append(m.id)

                    for m_id in self.Parameters.motor_id_checklist:
                        ros_fully_booted &= m_id in motor_id_list
                else:
                    ros_fully_booted = False

                frame_counter = 0
            frame_counter += 1

            if self.control_subscriber.get() is not None:
                if self.control_subscriber.get().acknowledge:
                    self.fault_list.clear()

            # if self.arm_subscriber.get() is not None:
            #     arm_status_message : Arm_Status = self.arm_subscriber.get()
            #     ros_fully_booted &= arm_status_message.is_node_alive
            #     if arm_status_message.left_arm_base_remote_loss_of_signal:
            #         left_arm_base_remote_loss_of_signal = Fault()
            #         left_arm_base_remote_loss_of_signal.code = "Left Arm Base Remote Loss of Signal"
            #         left_arm_base_remote_loss_of_signal.priority = 1
            #         if left_arm_base_remote_loss_of_signal not in self.fault_list:
            #             self.fault_list.append(left_arm_base_remote_loss_of_signal)

            #     if arm_status_message.right_arm_base_remote_loss_of_signal:
            #         right_arm_base_remote_loss_of_signal = Fault()
            #         right_arm_base_remote_loss_of_signal.code = "Right Arm Base Remote Loss of Signal"
            #         right_arm_base_remote_loss_of_signal.priority = 1
            #         if right_arm_base_remote_loss_of_signal not in self.fault_list:
            #             self.fault_list.append(right_arm_base_remote_loss_of_signal)

            #     if arm_status_message.left_arm_base_reset_during_en:
            #         left_arm_base_reset_during_en = Fault()
            #         left_arm_base_reset_during_en.code = "Left Arm Base Reset During En"
            #         left_arm_base_reset_during_en.priority = 1
            #         if left_arm_base_reset_during_en not in self.fault_list:
            #             self.fault_list.append(left_arm_base_reset_during_en)

            #     if arm_status_message.right_arm_base_reset_during_en:
            #         right_arm_base_reset_during_en = Fault()
            #         right_arm_base_reset_during_en.code = "Right Arm Base Reset During En"
            #         right_arm_base_reset_during_en.priority = 1
            #         if right_arm_base_reset_during_en not in self.fault_list:
            #             self.fault_list.append(right_arm_base_reset_during_en)

            #     if arm_status_message.left_arm_base_hardware_ESD_reset:
            #         left_arm_base_hardware_ESD_reset = Fault()
            #         left_arm_base_hardware_ESD_reset.code = "Left Arm Base Hardware ESD Reset"
            #         left_arm_base_hardware_ESD_reset.priority = 1
            #         if left_arm_base_hardware_ESD_reset not in self.fault_list:
            #             self.fault_list.append(left_arm_base_hardware_ESD_reset)

            #     if arm_status_message.right_arm_base_hardware_ESD_reset:
            #         right_arm_base_hardware_ESD_reset = Fault()
            #         right_arm_base_hardware_ESD_reset.code = "Right Arm Base Hardware ESD Reset"
            #         right_arm_base_hardware_ESD_reset.priority = 1
            #         if right_arm_base_hardware_ESD_reset not in self.fault_list:
            #             self.fault_list.append(right_arm_base_hardware_ESD_reset)

            #     if arm_status_message.left_arm_base_supply_unstable:
            #         left_arm_base_supply_unstable = Fault()
            #         left_arm_base_supply_unstable.code = "Left Arm Base Supply Unstable"
            #         left_arm_base_supply_unstable.priority = 2
            #         if left_arm_base_supply_unstable not in self.fault_list:
            #             self.fault_list.append(left_arm_base_supply_unstable)

            #     if arm_status_message.right_arm_base_supply_unstable:
            #         right_arm_base_supply_unstable = Fault()
            #         right_arm_base_supply_unstable.code = "Right Arm Base Supply Unstable"
            #         right_arm_base_supply_unstable.priority = 2
            #         if right_arm_base_supply_unstable not in self.fault_list:
            #             self.fault_list.append(right_arm_base_supply_unstable)
            # else:
            #     ros_fully_booted = False

            # if self.intake_subscriber.get() is not None:
            #     intake_status : Intake_Status = self.intake_subscriber.get()
            #     ros_fully_booted &= intake_status.is_node_alive
            #     pass
            # else:
            #     ros_fully_booted = False

            status = HealthMonitorStatus()
            status.faults = sorted(self.fault_list, key=lambda fault: fault.priority, reverse=True)
            status.is_ros_fully_booted = ros_fully_booted
            self.status_publisher.publish(status)

            self.loop_rate.sleep()

def signal_handler(sig, frame):
    sys.exit(0)

def main(args=None):
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.init(args=args)
    node = LocalNode()
    node.loop_thread.start()
    rclpy.spin(node)
    rclpy.shutdown()
    node.loop_thread.join()


if __name__ == '__main__':
    main()