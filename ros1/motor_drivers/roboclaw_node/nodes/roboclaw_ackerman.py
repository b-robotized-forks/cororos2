#!/usr/bin/env python
from math import pi, cos, sin, atan2

import diagnostic_msgs
import diagnostic_updater
import roboclaw_driver.roboclaw_driver as roboclaw
import rospy
import tf
from time import sleep
from shared_messages.msg import *

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

    rospy.init_node("roboclaw_node", log_level=rospy.DEBUG)
    rospy.on_shutdown(self.shutdown)
    rospy.loginfo("Connecting to roboclaw")
    dev_name = rospy.get_param("~dev", "/dev/ttyACM0")
    baud_rate = int(rospy.get_param("~baud", "115200"))

    self.address = int(rospy.get_param("~address", "128"))
    if self.address > 0x87 or self.address < 0x80:
      rospy.logfatal("Address out of range")
      rospy.signal_shutdown("Address out of range")

    # TODO need someway to check if address is correct
    # try:
    roboclaw.Open(dev_name, baud_rate)
    # except Exception as e:
      # rospy.logfatal("Could not connect to Roboclaw")
      # rospy.logdebug(e)
      # rospy.signal_shutdown("Could not connect to Roboclaw")

    self.updater = diagnostic_updater.Updater()
    self.updater.setHardwareID("Roboclaw 2x60A")
    self.updater.add(diagnostic_updater.
             FunctionDiagnosticTask("Vitals", self.check_vitals))

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

    self.headingRange = roboclaw.ReadM2PositionPID(self.address)[6:]
    roboclaw.DutyM1(self.address, 0)																		## Replace with SpeedM1
    roboclaw.ResetEncoders(self.address)
    roboclaw.SpeedAccelDeccelPositionM2(self.address, 0, 0, 0, int((self.headingRange[0] + self.headingRange[1])/2), 0)

    self.last_set_speed_time = rospy.get_rostime()
    self.MAX_SPEED = float(rospy.get_param("~max_speed", "2.0"))
    self.TICKS_PER_METER = float(rospy.get_param("~tick_per_meter", "4342.2"))
    self.HEADING_ANGLE_RANGE = float(rospy.get_param("~heading_angle_range", "90"))

    rospy.Subscriber("/cmd_vel", Velocity, self.cmd_vel_callback)
    self.volt_pub = rospy.Publisher('/roboclaw/voltage', MotorStatus, queue_size=10)

    rospy.sleep(1)

    rospy.logdebug("dev %s", dev_name)
    rospy.logdebug("baud %d", baud_rate)
    rospy.logdebug("address %d", self.address)
    rospy.logdebug("max_speed %f", self.MAX_SPEED)
    rospy.logdebug("heading_angle_range %f", self.HEADING_ANGLE_RANGE)
  
  def cmd_vel_callback(self, vel):
    self.last_set_speed_time = rospy.get_rostime()
	
    target_heading = vel.heading * 180.0 / 3.1415926;
    if(target_heading < -(self.HEADING_ANGLE_RANGE/2.0)):
      target_heading = -(self.HEADING_ANGLE_RANGE/2.0)
    if(target_heading > (self.HEADING_ANGLE_RANGE/2.0)):
      target_heading = self.HEADING_ANGLE_RANGE/2.0

    target_position = int(float(self.headingRange[1] - self.headingRange[0])*((float(target_heading) / float(self.HEADING_ANGLE_RANGE)) - 0.5) + self.headingRange[1])

    #v_ticks = int(vel.linear_velocity * self.TICKS_PER_METER)  # ticks/s											## Uncomment when SpeedM1
    v_ticks = int(vel.linear_velocity / 7.0 * 32767.0)

    rospy.logdebug("target_position:%d v_ticks: %d", target_position, v_ticks)

    try:
      # This is a hack way to keep a poorly tuned PID from making noise at speed 0
      if v_ticks is 0:
        roboclaw.DutyAccelM1(self.address, 0, 0)
      else:
        roboclaw.DutyAccelM1(self.address, 32000, v_ticks)
      roboclaw.SpeedAccelDeccelPositionM2(self.address, 0, 0, 0, target_position, 0)
    except OSError as e:
      rospy.logwarn("Speed OSError: %d", e.errno)
      rospy.logdebug(e)

  def run(self):
    rospy.loginfo("Starting motor drive")
    r_time = rospy.Rate(10)
    pubInterval = rospy.get_rostime()
    while not rospy.is_shutdown():
            
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

      if (rospy.get_rostime() - self.last_set_speed_time).to_sec() > 5:
        rospy.loginfo("Did not get command for 1 second, stopping")
        try:
          roboclaw.DutyAccelM1(self.address, 64000, 0)														## Replace with SpeedM1
          roboclaw.SpeedAccelDeccelPositionM2(self.address, 0, 0, 0, int((self.headingRange[0] + self.headingRange[1])/2), 0)
        except OSError as e:
          rospy.logerr("Could not stop")
          rospy.logdebug(e)

      r_time.sleep()

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
    try:
      stat.add("Main Batt V:", float(roboclaw.ReadMainBatteryVoltage(self.address)[1] / 10.0))
      stat.add("Logic Batt V:", float(roboclaw.ReadLogicBatteryVoltage(self.address)[1] / 10.0))
      stat.add("Temp1 C:", float(roboclaw.ReadTemp(self.address)[1] / 10.0))
      stat.add("Temp2 C:", float(roboclaw.ReadTemp2(self.address)[1] / 10.0))
    except OSError as e:
      rospy.logwarn("Diagnostics OSError: %d", e.errno)
      rospy.logdebug(e)
    return stat

  # TODO: need clean shutdown so motors stop even if new msgs are arriving
  def shutdown(self):
    rospy.loginfo("Shutting down")
    try:
      roboclaw.DutyAccelM1(self.address, 32000, 0)															## Replace with SpeedM1
      roboclaw.SpeedAccelDeccelPositionM2(self.address, 0, 0, 0, int((self.headingRange[0] + self.headingRange[1])/2), 0)
    except OSError:
      rospy.logerr("Shutdown did not work trying again")
      try:
        roboclaw.DutyAccelM1(self.address, 32000, 0)														## Replace with SpeedM1
        roboclaw.SpeedAccelDeccelPositionM2(self.address, 0, 0, 0, int((headingRange[0] + headingRange[1])/2), 0)
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
