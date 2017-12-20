#!/usr/bin/env python
#encoding: utf8
import sys, rospy, math, tf, time
from pimouse_ros.msg import MotorFreqs
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, Point
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.srv import TimedMotion
from nav_msgs.msg import Odometry

class Motor():
    def __init__(self):
        if not self.set_power(False): sys.exit(1)

        rospy.on_shutdown(self.set_power)
        self.sub_raw = rospy.Subscriber('motor_raw', MotorFreqs, self.callback_raw_freq)
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.callback_cmd_vel)
        self.sub_imu = rospy.Subscriber('imu/data_raw', Imu, self.callback_imu)
        self.srv_on = rospy.Service('motor_on', Trigger, self.callback_on)
        self.srv_off = rospy.Service('motor_off', Trigger, self.callback_off)
        self.srv_tm = rospy.Service('timed_motion', TimedMotion, self.callback_tm)
        self.last_time = rospy.Time.now()
        self.using_cmd_vel = False

        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
        self.bc_odom = tf.TransformBroadcaster()

        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.vx, self.vth = 0.0, 0.0

        rospy.set_param('imu_offset', -0.027)
        self.imu_offset = rospy.get_param('imu_offset')
        
        self.cur_time = rospy.Time.now()
        self.last_time = self.cur_time

    def onoff_response(self, onoff):
        d = TriggerResponse()
        d.success = self.set_power(onoff)
        d.message = "ON" if self.is_on else "OFF"
        return d

    def set_power(self, onoff = False):
        en = "/dev/rtmotoren0"
        try:
            with open(en, 'w') as f:
                f.write("1\n" if onoff else "0\n")
            self.is_on = onoff
            return True
        except:
            rospy.logerr("cannot write to " + en)

        return False

    def set_raw_freq(self, left_hz, right_hz):
        if not self.is_on:
            rospy.logerr("not enpowered")
            return

        try:
            with open("/dev/rtmotor_raw_l0", 'w') as lf, open("/dev/rtmotor_raw_r0", 'w') as rf:
                lf.write(str(int(round(left_hz))) + "\n")
                rf.write(str(int(round(right_hz))) + "\n")
        except:
            rospy.logerr("cannot write to rtmotor_raw_*")

    def callback_raw_freq(self, message):
        self.set_raw_freq(message.left_hz, message.right_hz)
    
    def callback_cmd_vel(self, message):
        if not self.is_on:
            return
        self.vx = message.linear.x
        #self.vth = message.angular.z

        forward_hz = 80000.0*message.linear.x/(9*math.pi)
        rot_hz = 400.0*message.angular.z/math.pi
        self.set_raw_freq(forward_hz-rot_hz, forward_hz + rot_hz)
        #self.using_cmd_vel = True
        #self.last_time = rospy.Time.now()

    def callback_imu(self, message):
        if not self.is_on:
            return
        self.vth = math.degrees(message.angular_velocity.z) - self.imu_offset
        #rospy.loginfo("vth: {}".format(self.vth))

    def callback_on(self, message): return self.onoff_response(True)
    def callback_off(self, message): return self.onoff_response(False)

    def callback_tm(self, message):
        if not self.is_on:
            rospy.logerr("not enpowered")
            return False

        dev = "/dev/rtmotor0"
        try:
            with open(dev, 'w') as f:
                f.write("%d %d %d\n" % (message.left_hz, message.right_hz, message.duration_ms))

        except:
            rospy.logerr("cannot write to " + dev)
            return False

        return True

    def send_odom(self):
        start = time.time()
        self.cur_time = rospy.Time.now()

        dt = self.cur_time.to_sec() - self.last_time.to_sec()
        self.x += self.vx * math.cos(self.th) * dt
        self.y += self.vx * math.sin(self.th) * dt
        self.th += self.vth * dt

        q = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.bc_odom.sendTransform((self.x, self.y, 0.0), q, self.cur_time, "base_link", "odom")

        odom = Odometry()
        odom.header.stamp = self.cur_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position = Point(self.x, self.y, 0)
        odom.pose.pose.orientation = Quaternion(*q)

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth

        self.pub_odom.publish(odom)

        self.last_time = self.cur_time
        #rospy.loginfo("odom_time: {}".format(time.time()-start))
        #rospy.loginfo("th: {}".format(self.th))

if __name__ == '__main__':
    rospy.init_node('motors')
    m = Motor()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        m.send_odom()
        rate.sleep()
