#!/usr/bin/env python3
from math import pi, cos, sin

import diagnostic_msgs
import diagnostic_updater
import roboclaw_driver.roboclaw_driver as roboclaw
# from roboclaw_driver.roboclaw_3 import Roboclaw as roboclaw
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
#from std_msgs.msg import Float32
from shared_messages.msg import MotorStatus

__author__ = "bwbazemore@uga.edu (Brad Bazemore)"


class Node:
    def __init__(self):

        self.ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
                       0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
                       0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
                       0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                       0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                       0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
                       0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
                       0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
                       0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
                       0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
                       0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
                       0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
                       0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                       0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                       0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
                       0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}

        rospy.init_node("roboclaw_node")
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to roboclaw")
        dev_name = rospy.get_param("~dev", "/dev/ttyACM0")
        baud_rate = int(rospy.get_param("~baud", "115200"))

        self.address = int(rospy.get_param("~address", "128"))
        if self.address > 0x87 or self.address < 0x80:
            rospy.logfatal("Address out of range")
            rospy.signal_shutdown("Address out of range")

        # TODO need someway to check if address is correct
        try:
            roboclaw.Open(dev_name, baud_rate)
        except Exception as e:
            rospy.logfatal("Could not connect to Roboclaw")
            rospy.logdebug(e)
            rospy.signal_shutdown("Could not connect to Roboclaw")

        #self.updater = diagnostic_updater.Updater()
        #self.updater.setHardwareID("Roboclaw")
        #self.updater.add(diagnostic_updater.FunctionDiagnosticTask("Vitals", self.check_vitals))

        try:
            version = roboclaw.ReadVersion(self.address)
        except Exception as e:
            rospy.logwarn("Problem getting roboclaw version")
            rospy.logdebug(e)
            pass

        if not version[0]:
            rospy.logwarn("Could not get version from roboclaw")
        else:
            rospy.logdebug(repr(version[1]))

        roboclaw.DutyAccelM1(self.address, 0, 0)
        roboclaw.DutyAccelM2(self.address, 0, 0)
        roboclaw.ResetEncoders(self.address)

        self.MAX_SPEED = float(rospy.get_param("~max_speed", "2.0"))
        self.TICKS_PER_METER = float(rospy.get_param("~tick_per_meter", "4342.2"))
        self.BASE_WIDTH = float(rospy.get_param("~base_width", "0.315"))

        self.last_set_speed_time = rospy.get_rostime()

        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
        self.volt_pub = rospy.Publisher('/roboclaw/voltage', MotorStatus, queue_size=10)

        rospy.sleep(1)

        rospy.logdebug("dev %s", dev_name)
        rospy.logdebug("baud %d", baud_rate)
        rospy.logdebug("address %d", self.address)
        rospy.logdebug("max_speed %f", self.MAX_SPEED)
        rospy.logdebug("ticks_per_meter %f", self.TICKS_PER_METER)
        rospy.logdebug("base_width %f", self.BASE_WIDTH)

    def run(self):
        rospy.loginfo("Starting motor drive")
        r_time = rospy.Rate(10)
        pubInterval = rospy.get_rostime()
        while not rospy.is_shutdown():

            #self.updater = diagnostic_updater.Updater()
            #self.updater.setHardwareID("Roboclaw")
            #self.updater.add(diagnostic_updater.FunctionDiagnosticTask("Vitals", self.check_vitals))
            
            if (rospy.get_rostime() - pubInterval).to_sec() > 2:
              try:
                pub = MotorStatus()
                pub.BatteryVoltage = float(roboclaw.ReadMainBatteryVoltage(self.address)[1] / 10.0)
                pub.M1Current = float(roboclaw.ReadCurrents(self.address)[1] / 100.0)
                pub.M2Current = float(roboclaw.ReadCurrents(self.address)[2] / 100.0)
                self.volt_pub.publish(pub)
              except OSError as e:
                rospy.logwarn("Diagnostics OSError: %d", e.errno)
                rospy.logdebug(e)
              pubInterval = rospy.get_rostime()

            if (rospy.get_rostime() - self.last_set_speed_time).to_sec() > 1:
                rospy.loginfo("Did not get command for 1 second, stopping")
                try:
                    roboclaw.DutyAccelM1(self.address, 0, 0)
                    roboclaw.DutyAccelM2(self.address, 0, 0)
                except OSError as e:
                    rospy.logerr("Could not stop")
                    rospy.logdebug(e)

            r_time.sleep()

    def cmd_vel_callback(self, twist):
        self.last_set_speed_time = rospy.get_rostime()

        linear_x = twist.linear.x
        vr = linear_x + twist.angular.z * self.BASE_WIDTH / 2.0  # m/s
        vl = linear_x - twist.angular.z * self.BASE_WIDTH / 2.0
        
        if vr > self.MAX_SPEED:
            vr = self.MAX_SPEED
        if vr < -self.MAX_SPEED:
            vr = -self.MAX_SPEED
        
        if vl > self.MAX_SPEED:
            vl = self.MAX_SPEED
        if vl < -self.MAX_SPEED:
            vl = -self.MAX_SPEED

        vr_ticks = int(vr * 32760.0/1.2)  # ticks/s
        vl_ticks = int(vl * 32760.0/1.2)

        rospy.loginfo("vr_ticks:%d vl_ticks: %d", vr_ticks, vl_ticks)

        try:
            # This is a hack way to keep a poorly tuned PID from making noise at speed 0
            if vr_ticks == 0 and vl_ticks == 0:
                roboclaw.DutyAccelM1(self.address, 0, 0)
                roboclaw.DutyAccelM2(self.address, 0, 0)
            else:
                roboclaw.DutyAccelM1(self.address, 32000, vr_ticks)
                roboclaw.DutyAccelM2(self.address, 32000, vl_ticks) # Negate for Cornelius
        except OSError as e:
            rospy.logwarn("SpeedM1M2 OSError: %d", e.errno)
            rospy.logdebug(e)

    # TODO: Need to make this work when more than one error is raised
    def check_vitals(self, stat):
        try:
            status = roboclaw.ReadError(self.address)[1]
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
            return
        state, message = self.ERRORS[status]
        stat.summary(state, message)
        stat.add("Running")
        try:
            stat.add("Main Batt V:", float(roboclaw.ReadMainBatteryVoltage(self.address)[1] / 10))
            #stat.add("Logic Batt V:", float(roboclaw.ReadLogicBatteryVoltage(self.address)[1] / 10))
            #stat.add("Temp1 C:", float(roboclaw.ReadTemp(self.address)[1] / 10))
            #stat.add("Temp2 C:", float(roboclaw.ReadTemp2(self.address)[1] / 10))
            stat.add("Curr1 A:", float(roboclaw.ReadCurrents(self.address)[1] / 100))
            stat.add("Curr2 A:", float(roboclaw.ReadCurrents(self.address)[2] / 100))
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
        return stat

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        rospy.loginfo("Shutting down")
        try:
            roboclaw.DutyAccelM1(self.address, 0, 0)
            roboclaw.DutyAccelM2(self.address, 0, 0)
        except OSError:
            rospy.logerr("Shutdown did not work trying again")
            try:
                roboclaw.DutyAccelM1(self.address, 0, 0)
                roboclaw.DutyAccelM2(self.address, 0, 0)
            except OSError as e:
                rospy.logerr("Could not shutdown motors!!!!")
                rospy.logdebug(e)


if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
