#!/usr/bin/env python3

from collections import deque
import numpy as np
import rospy
from rcb4.armh7interface import ARMH7Interface
from sensor_msgs.msg import JointState
import serial
import std_msgs.msg
import time

class UmoruArmController():
    def __init__(self):
        self.joint_name_to_id = {'joint_pitch':1, 'joint_yaw':3}
        self.interface = ARMH7Interface()
        device = rospy.get_param('~device', '')
        if device == '':
            self.interface.auto_open()
        else:
            self.interface.open(device)

        self.sub = rospy.Subscriber('~joint_state',
            JointState, queue_size=1,
            callback=self.callback)
        self.control_pressure = rospy.get_param('~control_pressure', True)
        self.air_board_ids = self.interface.search_air_board_ids() \
                                                   .tolist()
        self._scaled_pressure_publisher_dict = {}
        self.recent_pressures = {}
        self.min_pressures = {}
        self.max_pressures = {}
        for idx in self.air_board_ids:
            self.calibrate_sensor(idx)
        self.refill_lower_threshold = -2.8
        self.refill_upper_threshold = 1.2
        rospy.loginfo("servo id: {}".format(self.interface.search_servo_ids()))
        rospy.loginfo("air board id: {}".format(self.air_board_ids))

    def start_add_air(self, ids):
        if not isinstance(ids, list):
            ids = [ids]
        for i in self.air_board_ids:
            if i not in ids:
                self.interface.close_work_valve(i)
            self.interface.open_relay_valve(i)
        for i in ids:
            self.interface.open_work_valve(i)
        self.interface.start_pump()

    def stop_add_air(self, ids):
        if not isinstance(ids, list):
            ids = [ids]
        for i in ids:
            self.interface.close_work_valve(i)
        for i in self.air_board_ids:
            self.interface.close_relay_valve(i)
        self.interface.stop_pump()

    def start_remove_air(self, ids):
        if not isinstance(ids, list):
            ids = [ids]
        for i in self.air_board_ids:
            if i not in ids:
                self.interface.close_work_valve(i)
            self.interface.open_relay_valve(i)
        for i in ids:
            self.interface.open_work_valve(i)
        self.interface.open_air_connect_valve()

    def stop_remove_air(self, ids):
        if not isinstance(ids, list):
            ids = [ids]
        for i in ids:
            self.interface.close_work_valve(i)
        self.interface.close_air_connect_valve()
        for i in self.air_board_ids:
            self.interface.close_relay_valve(i)

    def refill(self, idx, threshold):
        rospy.loginfo(f"[refill] {idx}")
        start_time = time.time()
        self.start_add_air(idx)
        while True:
            pressure = self.read_pressure_sensor(idx)
            scaled_pressure = self.scaled_pressure(idx)
            rospy.loginfo(f"[refill] {pressure}")
            if scaled_pressure >= threshold or (time.time() - start_time) >= 4:
                break
        self.stop_add_air(idx)

    def publish_pressure(self):
        for idx in self.air_board_ids:
            try:
                key = f'{idx}'
                if key not in self._scaled_pressure_publisher_dict:
                    self._scaled_pressure_publisher_dict[key] = rospy.Publisher(
                        '/scaled_pressure/'+key, std_msgs.msg.Float32,
                        queue_size=1)
                    # Avoid 'rospy.exceptions.ROSException:
                    # publish() to a closed topic'
                    rospy.sleep(0.1)
                pressure = self.read_pressure_sensor(idx)
                scaled_pressure = self.scaled_pressure(idx)
                if scaled_pressure is not None:
                    self._scaled_pressure_publisher_dict[key].publish(
                        std_msgs.msg.Float32(data=scaled_pressure))
                    if self.control_pressure:
                        if scaled_pressure > -8 and scaled_pressure < self.refill_lower_threshold:
                            self.refill(idx, self.refill_upper_threshold)
            except serial.serialutil.SerialException as e:
                rospy.logerr('[publish_pressure] {}'.format(str(e)))

    def read_pressure_sensor(self, idx, force=False):
        if idx not in self.recent_pressures:
            # Initialize a deque for each new sensor index (id)
            self.recent_pressures[idx] = deque([], maxlen=5)

        while True:
            try:
                pressure = self.interface.read_pressure_sensor(idx)
                self.recent_pressures[idx].append(pressure)
                return pressure
            except serial.serialutil.SerialException as e:
                rospy.logerr('[read_pressure_sensor] {}'.format(str(e)))
                if force is True:
                    continue

    def average_pressure(self, idx):
        if idx in self.recent_pressures and len(self.recent_pressures[idx]) > 0:
            # Calculate the average for the specific sensor id
            return sum(self.recent_pressures[idx]) / len(self.recent_pressures[idx])
        else:
            rospy.logwarn('[average_pressure] No data available for id {}'.format(idx))
            return None

    def scaled_pressure(self, idx):
        return self.average_pressure(idx) - self.max_pressures[idx]
        # # Get the current pressure and scale it using min and max pressures
        # current_pressure = self.average_pressure(idx)

        # if idx in self.min_pressures and idx in self.max_pressures:
        #     min_pressure = self.min_pressures[idx]
        #     max_pressure = self.max_pressures[idx]

        #     if max_pressure > min_pressure:  # Avoid division by zero
        #         scaled_pressure = (current_pressure - min_pressure) / (max_pressure - min_pressure)
        #         return scaled_pressure
        #     else:
        #         rospy.logwarn(f"[get_scaled_pressure] Invalid calibration range for sensor {idx}")
        #         return None
        # else:
        #     # rospy.logwarn(f"[get_scaled_pressure] Calibration data not found for sensor {idx}")
        #     return None

    def calibrate_sensor(self, idx):
        rospy.loginfo(f"[calibrate_sensor] id:{idx}")
        # # Step 1: Remove air to reach atmospheric pressure
        # self.start_remove_air(idx)
        # input("Press Enter to stop removing air...")
        # self.stop_remove_air(idx)

        # # Save the minimum pressure reading as atmospheric pressure
        # for i in range(10):
        #     self.read_pressure_sensor(idx)
        # min_pressure = self.average_pressure(idx)
        # if min_pressure > 40:
        #     rospy.logwarn(f"Invalid calibration range for sensor {idx}")
        #     return
        # self.min_pressures[idx] = min_pressure
        # rospy.loginfo(f"[calibrate_sensor] Sensor {idx} min pressure (atmospheric): {min_pressure}")

        # Step 2: Add air for 8 seconds to reach maximum pressure
        self.start_add_air(idx)
        input("Press Enter to stop adding air...")
        self.stop_add_air(idx)

        # Save the maximum pressure reading
        for i in range(10):
            self.read_pressure_sensor(idx)
        max_pressure = self.average_pressure(idx)
        # if max_pressure <= min_pressure:
        #     rospy.logwarn(f"Invalid calibration range for sensor {idx}")
        #     return
        self.max_pressures[idx] = max_pressure
        rospy.loginfo(f"[calibrate_sensor] Sensor {idx} max pressure: {max_pressure}")

    def callback(self, msg):
        servo_ids = []
        angle_vector = []
        velocity_vector = []
        for name, position, velocity in zip(msg.name, msg.position, msg.velocity):
            if name not in self.joint_name_to_id:
                continue
            idx = self.joint_name_to_id[name]
            servo_ids.append(idx)
            angle_vector.append(position)
            velocity_vector.append(velocity)
            
        angle_vector = np.array(angle_vector)
        velocity_vector = np.array(velocity_vector)
        servo_ids = np.array(servo_ids, dtype=np.int32)
        self.interface.angle_vector(angle_vector, servo_ids, velocity_vector)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.interface.is_opened() is False:
                self.unsubscribe()
                rospy.signal_shutdown('Disconnected.')
                break
            self.publish_pressure()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('umoru_arm_controller')
    umoru_arm_controller = UmoruArmController()
    umoru_arm_controller.run()
