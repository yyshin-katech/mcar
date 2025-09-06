#!/usr/bin/env python

import rospy
import can
import cantools
import rospkg
import numpy as np

from std_msgs.msg import Float32, Float64
from mmc_msgs.msg import chassis_msg

PI = 3.141592
deg2rad = PI / 180.0

class CanListner(can.Listener):
    def __init__(self):
        print(' [*] DEFINE CAN LISTENER ...')

        rospy.init_node('chassis_can_publisher')

        print " [*] SET CAN DATABASE ..."
        rospack = rospkg.RosPack()
        can_path = rospack.get_path('can')

        dbpath = can_path + "/dbc/180518_HAD_v5.dbc"

        self.db = cantools.db.load_file(dbpath)

        print " [*] DEFINE PUBLISHER  ... "
        # publisher
        self.sensor_pub = rospy.Publisher('/sensors/chassis', chassis_msg, queue_size=1)
        self.sensor_msg = [[] for i in range(6)] # seven messages

        self.time_old = rospy.Time.now().to_sec()

    def time_click(self,disp = ''):
        self.time = rospy.Time.now().to_sec()
        # print(float("{0:.4f}".format(self.time - self.time_old))*1000)
        self.time_old = self.time

    def sensor_publish(self):
        temp_time = rospy.Time.now()
        decoded =[]
        # print('*** CHASSIS CAN PUBLISHED BELOW ***')

        for i in range(len(self.sensor_msg)):
            can_data = self.sensor_msg[i][1].data
            decoded.append(self.CHASSIS[self.sensor_msg[i][0]].decode(data=can_data))

        """Cmd"""


        """Chasiss 1"""
        # for k, decod in enumerate(decoded):
        #     print(k)
        #     print(decod)
        # print(decoded[1])
        yawrate_in_deg = decoded[0]['YawRate']
        yawrate_in_rad = yawrate_in_deg * deg2rad

        steering_angle_in_deg = decoded[0]['SteerAngle']

        """Chasiss 2"""
        autonomous_mode = np.uint8(decoded[1]['AutonomousModeReady']) # {3: 'Autonomous Mode', 2: 'Lateral Control', 1: 'Longitudinal Control', 0: 'Off'}
        steering_torque_in_Nm = decoded[1]['SteeringTorque']

        """Chasiss 3"""
        speed_in_kph = (decoded[2]['RlWheelSpeed'] + decoded[2]['RrWheelSpeed']) / 2.0
        speed_in_meterps = speed_in_kph / 3.6  # m/s

        """Chasiss 4"""
        steering_rate_in_deg_p_sec = decoded[3]['SteerRate']
        longitudinal_acc_in_mps2 = decoded[3]['LongAccel']
        lateral_acc_in_mps2 = decoded[3]['LateralAccel']

        """Chasiss 5"""
        gear_position = decoded[4]['GearPosition']
        driver_override = np.uint8(decoded[4]['DriverOverride'])
        brake_act = np.uint8(decoded[4]['BrakeAct'])
        cruise_switch_state = decoded[4]['CruiseSwState']

        """Chassis 6"""
        right_turn_signal = decoded[5]['TurnSigRight']
        left_turn_signal = decoded[5]['TurnSigLeft']
        hazard_signal = decoded[5]['Hazard']

        msg = chassis_msg()
        msg.time = temp_time

        msg.speed = speed_in_meterps
        msg.yawrate = yawrate_in_rad - 0.00436332222222

        msg.ax = longitudinal_acc_in_mps2
        msg.ay = lateral_acc_in_mps2

        msg.steering_angle = steering_angle_in_deg
        msg.steering_torque = steering_torque_in_Nm
        msg.steering_rate = steering_rate_in_deg_p_sec

        msg.autonomous_mode = autonomous_mode

        msg.gear_position = gear_position
        msg.driver_override = driver_override
        msg.brake_act = brake_act
        msg.cruise_switch_state = cruise_switch_state

        msg.right_turn_signal = right_turn_signal
        msg.left_turn_signal = left_turn_signal
        msg.hazard_signal = hazard_signal

        self.sensor_pub.publish(msg)
        # print("published")
        self.sensor_msg = [[] for i in range(6)]

    def on_message_received(self, msg):
        if msg.arbitration_id in self.CHASSIS_id:
            i = self.CHASSIS_id.index(msg.arbitration_id)
            self.sensor_msg[i] = [i, msg]
            # print(self.sensor_msg)
            if not [] in self.sensor_msg:
                print("hello")
                self.sensor_publish()




    def set_db(self):
        self.CHASSIS = []
        self.CHASSIS_id = []
        self.CHASSIS.append(self.db.get_message_by_name('Chassis1'))
        self.CHASSIS.append(self.db.get_message_by_name('Chassis2'))
        self.CHASSIS.append(self.db.get_message_by_name('Chassis3'))
        self.CHASSIS.append(self.db.get_message_by_name('Chassis4'))
        self.CHASSIS.append(self.db.get_message_by_name('Chassis5'))
        self.CHASSIS.append(self.db.get_message_by_name('Chassis6'))
        for i in range(len(self.CHASSIS)):
            self.CHASSIS_id.append(self.CHASSIS[i].frame_id)
        print self.CHASSIS_id



if __name__ == '__main__':
    try:
        listener = CanListner()
        print " [*] DEFINE CAN LISTENER AND NOTIFIER ..."
        bus = can.interface.Bus(channel='can0', bustype='socketcan')
        listener.set_db()
        notifier = can.Notifier(bus, [listener])

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr('Could not start Chassis_Can_Pubisher() node.')
