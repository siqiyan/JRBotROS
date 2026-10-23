#!/usr/bin/env python3
import rospy
import serial
import threading
import time
import tf
from sensor_msgs.msg        import BatteryState
from nav_msgs.msg           import Odometry
from geometry_msgs.msg      import Twist
from dataframe              import *
from dataframe_base         import compute_checksum, verify_checksum, update_checksum
from tf.transformations     import euler_from_quaternion, quaternion_from_euler

class JRBotBase:
    def __init__(self):
        rospy.init_node('jrbot_base')
        self.ser = serial.Serial(
            '/dev/jrbot',
            115200,
            timeout=0.1
        )
        self.lock = threading.Lock()
        self._command = Twist()
        self._command_ts = None

        # Publishers:
        self.battery_pub = rospy.Publisher(
            'battery_state',
            BatteryState,
            queue_size=1
        )
        self.odom_pub = rospy.Publisher(
            'odom', Odometry, queue_size=1
        )
        self.tf_br = tf.TransformBroadcaster()

        # Subscribers:
        self.cmd_vel_sub = rospy.Subscriber(
            'cmd_vel', Twist, self.cmd_vel_cb
        )

        # TODO: add services for: odom_reset, motor_on, motor_off, ramp_on,
        # ramp_off

        self.recv_sequence_id = 0
        self.send_sequence_id = 0
        self.lost_packet_counter = 0

        self.start_node()

    def start_node(self):
        status_frame_sz = ROSStatusFrame().get_size()
        # odom_frame_sz = ROSOdomFrame().get_size()
        # recv_sz = max(status_frame_sz, odom_frame_sz)
        recv_sz = status_frame_sz

        while not rospy.is_shutdown():
            recv_buf = self.ser.read(recv_sz)
            if recv_buf is None:
                continue

            # print('recv_buf:', recv_buf)
            if self.try_process_status_frame(recv_buf):
                self.send_ack()
            # elif self.try_process_odom_frame(recv_buf):
                # self.send_ack()

        self.ser.close()

    def preprocess_frame(self, frame):
        if not verify_checksum(frame):
            print('checksum mismatch')
            self.lost_packet_counter += 1
            return False

        # Count sequence id and packet loss:
        sequence_id = frame.get_var('sequence_id')
        if (self.recv_sequence_id + 1) != sequence_id:
            self.lost_packet_counter += 1
        self.recv_sequence_id = sequence_id

        return True

    def try_process_status_frame(self, recv_buf):
        status_frame = ROSStatusFrame()

        # Verify size:
        if len(recv_buf) != status_frame.get_size():
            return False

        status_frame.from_bytes(recv_buf)
        # Verify header:
        if status_frame.get_var('header') != FRAMEID_ROS_STATUS:
            print('status_frame header mismatch')
            return False

        if not self.preprocess_frame(status_frame):
            return False

        print('process status_frame')

        print('pwn left:{} right:{}'.format(
            status_frame.get_var('left_pwm'),
            status_frame.get_var('right_pwm')
        ))

        print('v_left:{} v_right:{}'.format(
            status_frame.get_var('v_left'),
            status_frame.get_var('v_right')
        ))

        print('left_err:{} right_err:{}'.format(
            status_frame.get_var('left_err'),
            status_frame.get_var('right_err')
        ))

        right_mode = None
        debug_code = status_frame.get_var('debug_code')
        if debug_code == 0:
            right_mode = 'sdby'
        elif debug_code == 1:
            right_mode = 'running'
        elif debug_code == 2:
            right_mode = 'failing'
        elif debug_code == 3:
            right_mode = 'rising'
        print('right mode:{}'.format(
            right_mode
        ))

        adc_value = status_frame.get_var('battery_adc_value')
        R1 = 100000.0
        R2 = 10000.0
        ref_voltage = 5.0
        adc_voltage  = (int(adc_value) * ref_voltage) / 1024.0
        in_voltage = adc_voltage*(R1+R2)/R2
        battery_msg = BatteryState()
        battery_msg.header.stamp = rospy.Time.now()
        battery_msg.header.frame_id = 'battery'
        battery_msg.voltage = in_voltage
        battery_msg.current = float('nan')
        battery_msg.charge = float('nan')
        battery_msg.capacity = float('nan')
        battery_msg.design_capacity = float('nan')
        battery_msg.percentage = (in_voltage - 10.8) / (12.6 - 10.8)
        battery_msg.present = True
        self.battery_pub.publish(battery_msg)

        odom_msg = Odometry()
        odom_msg.header.frame_id = 'odom'
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = status_frame.get_var('x')
        odom_msg.pose.pose.position.y = status_frame.get_var('y')
        quat = quaternion_from_euler(
            0.0,
            0.0,
            status_frame.get_var('yaw')
        )
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        self.odom_pub.publish(odom_msg)

        # Publish corresponding tf:
        self.tf_br.sendTransform(
            (
                odom_msg.pose.pose.position.x,
                odom_msg.pose.pose.position.y,
                0
            ),
            (
                odom_msg.pose.pose.orientation.x,
                odom_msg.pose.pose.orientation.y,
                odom_msg.pose.pose.orientation.z,
                odom_msg.pose.pose.orientation.w
            ),
            rospy.Time.now(),
            'base_link',
            'odom'
        )

        print('status_frame proc success!')

        return True

    def try_process_odom_frame(self, recv_buf):
        odom_frame = ROSOdomFrame()

        # Verify size:
        if len(recv_buf) != odom_frame.get_size():
            return False

        odom_frame.from_bytes(recv_buf)
        # Verify header:
        if odom_frame.get_var('header') != FRAMEID_ROS_ODOM:
            print('odom_frame header mismatch')
            return False

        if not self.preprocess_frame(odom_frame):
            return False

        print('process odom_frame')

        odom_msg = Odometry()
        odom_msg.header.frame_id = 'odom'
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = odom_frame.get_var('x')
        odom_msg.pose.pose.position.y = odom_frame.get_var('y')
        quat = quaternion_from_euler(0.0, 0.0, odom_frame.get_var('yaw'))
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        self.odom_pub.publish(odom_msg)
        print('odom_frame proc success!')

        return True

    def send_ack(self):
        ts = time.time()
        command_frame = ROSCommandFrame()

        command_frame.set_var('header', FRAMEID_ROS_COMMAND)
        command_frame.set_var('sequence_id', self.send_sequence_id)
        self.send_sequence_id += 1

        with self.lock:
            if (self._command_ts is None) or \
                (rospy.Time.now() - self._command_ts).to_sec() > 5.0:
                command_frame.set_var('cmd_x', 0.0)
                command_frame.set_var('cmd_yaw', 0.0)
            else:
                command_frame.set_var('cmd_x', self._command.linear.x)
                command_frame.set_var('cmd_yaw', self._command.angular.z)

        update_checksum(command_frame)
        self.ser.write(command_frame.to_bytes())
        duration = (time.time() - ts) * 1000.0
        print('ack sent, duration: {} ms'.format(duration))

    def cmd_vel_cb(self, msg: Twist):
        with self.lock:
            self._command = msg
            self._command_ts = rospy.Time.now()


if __name__ == '__main__':
    JRBotBase()
