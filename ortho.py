#!/usr/bin/env python

#----------------------------------------------------------------------------
#  eve.py                    mnagy@voalte.com                      oct-2013
#----------------------------------------------------------------------------
#  This program requires the following python library be installed which in
#  turn depends on a system package named libusb-1.0.  Typical installation
#  of both might be:
#
#     sudo apt-get install libusb-1.0
#     sudo pip install libusb1
#
#  To enable 3d visualizations, you also need vpython.  Get it by:
#
#     sudo apt-get install python-visual     (Version 1:5.12-1.3)
#
#  This program coordinates i/o between two Pololu Maestro 12-Channel USB
#  Servo Controllers, which will be referred to as Robot ('Robot') and
#  Control Box ('CntBx').
#
#  The Robot uses the following channels:
#
#     0) servo output - right arm
#     1) servo output - front arm
#     2) servo output - left arm
#     6) digital output - ultrasonic sensor 'trigger' (ping)
#     7) analog input - ultrasonic sensor 'echo'
#
#  The Control Box uses the following channels (0=left to 3=right):
#
#     0) digital output - led 0
#     1) digital output - led 1
#     2) digital output - led 2
#     3) digital output - led 3
#     4) digital input - button 0
#     5) digital input - button 1
#     6) digital input - button 2
#     7) digital input - button 3
#     8) analog input - knob 0
#     9) analog input - knob 1
#    10) analog input - knob 2
#    11) analog input - knob 3
#
#  Servo outputs take 16-bit values from 4000 (min) to 8000 (max).  Analog
#  inputs return 16-bit values from 0 to 1023.  Both ranges are remapped to
#  0-255 for consistency by our low-level i/o routines.
#
#  The ultrasonic sensor expects a trigger pulse of at least 10uS.  On the
#  falling edge of this trigger (ping) it sets its output (echo) high and
#  emits a pulse and listens for the echo.  When it hears the echo it sets
#  the echo output low, meaning that the width of the echo pulse corresponds
#  to the distance measured at the ratio of 58uS per cm.  Since we can't
#  resolve such timing accurately, an R/C integrator was inserted into the
#  signal path with values chosen so that distances of up to 10cm would map
#  to peak voltages within the limits of the analog input range.  The peak
#  voltage will occur approximately 500uS after the falling edge of the
#  ping pulse and will decay to roughly zero voltage after around 80ms.  The
#  net is you shouldn't call the GetDistance() routine more than around
#  10 times a second or measurements will blur together.
#
#  The robot servo controller was pre-configured from a Windows 7 system
#  running the Pololu Maestro control panel application.  Specifically,
#  channels 0/1/2 were configured for servo output with a minimum value
#  of 992, a default value of 1000, and a maximum value of 1504.  In
#  addition, speed was set to 0 (unlimited) and acceleration to 16 for
#  the three servo channels.  Channel 6 was set as an output and channel
#  7 was set as an input.
#
#  The control box servo controller was likewise configured with channels
#  0/1/2/3 as outputs, channels 4/5/6/7 as digital inputs, and channels
#  8/9/10/11 as analog inputs.
#
#----------------------------------------------------------------------------

import struct         # unpack
import time           # sleep
import math           # sin, cos, pi, sqrt
import numpy          # deg2rad
import usb1           # libusb1
import sys            # stdout

# from visual import *  # vpython

#----------------------------------------------------------------------------
#  The usb vendor and product ids for the pololu maestro 12-channel servo
#  controller which is used in both the robot and the control box.
#----------------------------------------------------------------------------

VID = 0x1ffb # pololu robotics
PID = 0x008a # maestro 12 servo controller

#----------------------------------------------------------------------------
#  The serial numbers of the specific maestro servo controllers used in the
#  robot and the control box.
#----------------------------------------------------------------------------

RobotSerial = '59243'
CntBxSerial = '68800'

#----------------------------------------------------------------------------
#  A set of minimal classes to which we can add stuff to as needed.
#----------------------------------------------------------------------------

class ServoController():
  def __init__(self, name, handle):
    self.name = name
    self.handle = handle
    self.data = None

class RobotSize():
  def __init__(self, e, f, re, rf):
    self.e = e
    self.f = f
    self.re = re
    self.rf = rf

class RobotPos():
  def __init__(self, x0, y0, z0):
    self.x0 = x0
    self.y0 = y0
    self.z0 = z0

class RobotAngle():
  def __init__(self, theta1, theta2, theta3):
    self.theta1 = theta1
    self.theta2 = theta2
    self.theta3 = theta3

#----------------------------------------------------------------------------
#  The usb control transfer request values for the two operations we need
#  to perform on the maestro servo controllers.  SET_TARGET lets us position
#  the servos, while GET_SERVO_SETTINGS lets us read inputs.
#----------------------------------------------------------------------------

REQUEST_SET_TARGET         = 0x85
REQUEST_GET_SERVO_SETTINGS = 0x87

#----------------------------------------------------------------------------
#  The GET_SERVO_SETTINGS transfer returns an array of data for all of the
#  servos in one data block.  These format strings let us unpack it.
#----------------------------------------------------------------------------

FormatInput = 'HHHB' # position:16, target:16, speed:16, acceleration:8
FormatInput12 = '<'+(FormatInput*12) # little-endian, 12 input channels

#----------------------------------------------------------------------------
#  Get the current status of all 12 input channels on a maestro controller.
#  This call returns a list of 4 integer values per channel (position,
#  target, speed and acceleration) repeated for each channel.  We return a
#  list instead of a tuple to allow individual elements to be modified.
#----------------------------------------------------------------------------

def GetInputs(HandleData):
  HandleData.data = list(struct.unpack(FormatInput12, HandleData.handle.controlRead(0xc0, REQUEST_GET_SERVO_SETTINGS, 0, 0, 7*12)))

#----------------------------------------------------------------------------
#  Get the value (0..255) of an input on a maestro controller.  This call
#  remaps the raw input values from their native range (0..1023). 
#----------------------------------------------------------------------------

def GetInput(HandleData, Channel):
  return HandleData.data[Channel*4]/4

#----------------------------------------------------------------------------
#  Get the value (0 or 255) of an button on a maestro controller.
#----------------------------------------------------------------------------

def GetButton(HandleData, Channel):
  if GetInput(HandleData, Channel) > 127:
    return True
  return False

#----------------------------------------------------------------------------
#  Set the value (0..255) of an output on a maestro controller.  This range
#  is remapped to (4000..8000) before transmission to the controller.  We
#  require InputData to be a list, not a tuple, so we can assign to its
#  target value for the specified channel to avoid redundant outputs of the
#  same value to the same channel.
#----------------------------------------------------------------------------

def SetOutput(HandleData, Channel, Value):
  if GetInput(HandleData, Channel) != Value:
    HandleData.handle.controlWrite(0x40, REQUEST_SET_TARGET, (Value*15)+4088, Channel, 0, 0)
    HandleData.data[Channel*4] = Value*4

#----------------------------------------------------------------------------
#  Turn a light on or off (True or False).
#----------------------------------------------------------------------------

def SetLight(HandleData, Channel, On):
  if On:
    SetOutput(HandleData, Channel, 255)
  else:
    SetOutput(HandleData, Channel, 0)

#----------------------------------------------------------------------------
#  Show some device-level data (valid before device is open).
#----------------------------------------------------------------------------

def ShowDevice(Device):
  print 'VID %04x PID %04x'   % (Device.getVendorID(), Device.getProductID())
  print '  Serial [%s]'       % (Device.getSerialNumber())
  print '  Manufacturer [%s]' % (Device.getManufacturer())
  print '  Product [%s]'      % (Device.getProduct     ())

#----------------------------------------------------------------------------
#  Show the status of a servo input channel.
#----------------------------------------------------------------------------

def ShowServo(HandleData, Channel, Name):
  Position, Target, Speed, Acceleration = HandleData.data[Channel*4:Channel*4+4]
  print('  %5s: Position %d, Target %d, Speed %d, Acceleration %d' % (Name, Position, Target, Speed, Acceleration))

#----------------------------------------------------------------------------
#  Show the status of an analog input channel.
#----------------------------------------------------------------------------

def ShowInput(HandleData, Channel, Name):
  print('  %5s: Value %d' % (Name, GetInput(HandleData, Channel)))

#----------------------------------------------------------------------------
#  Show the status of an digital input channel.
#----------------------------------------------------------------------------

def ShowButton(HandleData, Channel, Name):
  print('  %5s: Value %d' % (Name, GetButton(HandleData, Channel)))

#----------------------------------------------------------------------------
#  Show the status of a ping/echo distance transducer channel pair.
#----------------------------------------------------------------------------

def ShowDistance(HandleData, PingChannel, EchoChannel):
  SetOutput(HandleData, PingChannel, 255)
  time.sleep(0.001)
  SetOutput(HandleData, PingChannel, 0)
  Vmax = 0
  for i in range(0,4):
    GetInputs(HandleData)
    Value = GetInput(HandleData, EchoChannel)
    if Vmax < Value:
      Vmax = Value
  print('  distance %d' % (Vmax))

#----------------------------------------------------------------------------
#  Show the status of all robot channels.
#----------------------------------------------------------------------------

def ShowRobot(HandleData):
  GetInputs(HandleData)
  ShowServo(HandleData, 0, 'right')
  ShowServo(HandleData, 1, 'front')
  ShowServo(HandleData, 2, 'left' )
  ShowDistance(HandleData, 6, 7)

#----------------------------------------------------------------------------
#  Show the status of all control box channels.
#----------------------------------------------------------------------------

def ShowCntBx(HandleData):
  GetInputs(HandleData)
  ShowButton(HandleData,  4, 'button-0')
  ShowButton(HandleData,  5, 'button-1')
  ShowButton(HandleData,  6, 'button-2')
  ShowButton(HandleData,  7, 'button-3')
  ShowInput (HandleData,  8, 'knob-0'  )
  ShowInput (HandleData,  9, 'knob-1'  )
  ShowInput (HandleData, 10, 'knob-2'  )
  ShowInput (HandleData, 11, 'knob-3'  )
  SetLight(HandleData, 0, True )
  SetLight(HandleData, 1, False)
  SetLight(HandleData, 2, True )
  SetLight(HandleData, 3, False)

#----------------------------------------------------------------------------
#  Some constants used by the reverse kinematics math.
#----------------------------------------------------------------------------

SQRT3 = math.sqrt(3.0)
SIN120 = SQRT3 / 2.0
COS120 = -0.5
TAN60 = SQRT3
SIN30 = 0.5
TAN30 = 1.0 / SQRT3

#----------------------------------------------------------------------------
#  Forward kinematics calculation.  Given the three servo angles (in
#  degrees), return the coordinates of the end effector in the globals
#  x0, y0 and z0.  Return 0 for success or -1 for error (non-existing
#  point).
#----------------------------------------------------------------------------

def RkForward(Robot):
  Robot.pos.x0 = Robot.pos.y0 = Robot.pos.z0 = 0.0
  t = (Robot.size.f - Robot.size.e) * TAN30 / 2.0
  rad1 = numpy.deg2rad(Robot.angle.theta1)
  rad2 = numpy.deg2rad(Robot.angle.theta2)
  rad3 = numpy.deg2rad(Robot.angle.theta3)
  y1 = -(t + Robot.size.rf*math.cos(rad1))
  z1 =     - Robot.size.rf*math.sin(rad1)
  y2 =  (t + Robot.size.rf*math.cos(rad2)) * SIN30
  x2 = y2 * TAN60
  z2 = - Robot.size.rf * math.sin(rad2)
  y3 = (t + Robot.size.rf*math.cos(rad3))*SIN30
  x3 = -y3 * TAN60
  z3 = -Robot.size.rf * math.sin(rad3)
  dnm = (y2-y1)*x3-(y3-y1)*x2
  w1 = y1*y1 + z1*z1
  w2 = x2*x2 + y2*y2 + z2*z2
  w3 = x3*x3 + y3*y3 + z3*z3
  # x = (a1*z + b1)/dnm
  a1 = (z2-z1) * (y2-y2) - (z3-z1)*(y2-y2)
  b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0
  # y = (a2*z + b2)/dnm
  a2 = -(z2-z1)*x3+(z3-z1)*x2
  b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0
  # a*z^2 + b*z + c = 0
  a = a1*a1 + a2*a2 + dnm*dnm
  b = 2.0*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm)
  c = (b2-y1*dnm)*(b2-y1*dnm)+b1*b1 + dnm*dnm*(z1*z1 - Robot.size.re*Robot.size.re)
  #discriminant
  d = b*b - 4.0*a*c
  if d < 0.0:
    return -1 # non-existent point
  Robot.pos.z0 = -0.5*(b + math.sqrt(d))/a
  Robot.pos.x0 = (a1*Robot.pos.z0 + b1) / dnm
  Robot.pos.y0 = (a2*Robot.pos.z0 + b2) / dnm
  return 0

#----------------------------------------------------------------------------
#  Inverse kinematics helper function.  Calculates angle theta (for yz-pane).
#----------------------------------------------------------------------------

def RkAngle(Robot, x0, y0, z0):
  y1 = -0.5 * 0.57735 * Robot.size.f # f/2 * tg 30
  y0 = y0 - 0.5 * 0.57735 * Robot.size.e # shift center to edge
  # z = a + b*y
  a = (x0*x0 + y0*y0 + z0*z0 +Robot.size.rf*Robot.size.rf - Robot.size.re*Robot.size.re - y1*y1)/(2*z0)
  b = (y1-y0)/z0
  # discriminant
  d = -(a+b*y1)*(a+b*y1)+Robot.size.rf*(b*b*Robot.size.rf+Robot.size.rf)
  if d < 0:
    sys.stdout.write('!')
    sys.stdout.flush()
    return False # non-existing point
  yj = (y1 - a*b - math.sqrt(d))/(b*b + 1) # choosing outer point
  zj = a + b*yj
  if yj > y1:
    ax = 180.0
  else:
    ax = 0.0
  Robot.theta = 180.0*math.atan(-zj/(y1 - yj))/math.pi + ax
  #print('theta = %f' % (Robot.theta))
  return True

#----------------------------------------------------------------------------
#  Inverse kinematics calculation.  Given the three coordinates of
#  the end effector, return the servo angles necessary to achieve
#  that position in the globals x0, y0 and z0 (in degrees).  Return
#  0 for success or -1 for error (no solution).
#----------------------------------------------------------------------------

def RkInverse(Robot):
  Robot.angle.theta1 = Robot.angle.theta2 = Robot.angle.theta3 = 0.0
  status = RkAngle(Robot, Robot.pos.x0, Robot.pos.y0, Robot.pos.z0)
  if status:
    Robot.angle.theta1 = Robot.theta
    status = RkAngle(Robot, Robot.pos.x0*COS120 + Robot.pos.y0*SIN120, Robot.pos.y0*COS120-Robot.pos.x0*SIN120, Robot.pos.z0)  # rotate coords to +120deg
  if status:
    Robot.angle.theta2 = Robot.theta
    status = RkAngle(Robot, Robot.pos.x0*COS120 - Robot.pos.y0*SIN120, Robot.pos.y0*COS120+Robot.pos.x0*SIN120, Robot.pos.z0)  # rotate coords to -120deg
  if status:
    Robot.angle.theta3 = Robot.theta
  return status

#----------------------------------------------------------------------------
#  Set up and validate reverse-kinematics environment and calculations.
#----------------------------------------------------------------------------

def RkValidate(Robot, CntBx):
  Robot.size = RobotSize(e=115.0, f=457.3, re=232.0, rf=112.0)
  Robot.pos = RobotPos(x0=0.0, y0=0.0, z0=-200.0)
  Robot.angle = RobotAngle(theta1=0.0, theta2=0.0, theta3=0.0)
  Robot.theta = 0.0
  print('pos %f %f %f' % (Robot.pos.x0, Robot.pos.y0, Robot.pos.z0))
  print('  angle %f %f %f' % (Robot.angle.theta1, Robot.angle.theta2, Robot.angle.theta3))
  if not RkInverse(Robot):
    print('RkInverse error!')
  print('pos %f %f %f' % (Robot.pos.x0, Robot.pos.y0, Robot.pos.z0))
  print('  angle %f %f %f' % (Robot.angle.theta1, Robot.angle.theta2, Robot.angle.theta3))
  RkForward(Robot) # assure coherency
  print('pos %f %f %f' % (Robot.pos.x0, Robot.pos.y0, Robot.pos.z0))
  print('  angle %f %f %f' % (Robot.angle.theta1, Robot.angle.theta2, Robot.angle.theta3))

  # Run the kinematics calculations forward and backward to test.

  Angle0 = Robot.angle # grab initial angle & position
  Pos0 = Robot.pos

  RkInverse(Robot)
  RkForward(Robot)

  Angle1 = Robot.angle # grab final angle & position
  Pos1 = Robot.pos

  # Check that the initial and final postions and angles match.

  if (abs(Angle0.theta1-Angle1.theta1) < 0.0001) and \
     (abs(Angle0.theta2-Angle1.theta2) < 0.0001) and \
     (abs(Angle0.theta3-Angle1.theta3) < 0.0001):
    if (abs(Pos0.x0-Pos1.x0) < 0.0001) and \
       (abs(Pos0.y0-Pos1.y0) < 0.0001) and \
       (abs(Pos0.z0-Pos1.z0) < 0.0001):
      return True
    print('*** RK validation failure - position!')
  else:
    print('*** RK validation failure - angle!')
  return False

#----------------------------------------------------------------------------
#  Map the 3 knobs to x/y/z axis movements of the robot actuator.  The x/y
#  axis are center-zero cw right/back, while the z axis is left-zero cw-down.
#----------------------------------------------------------------------------

def OrthoControl(Robot, CntBx):
  floor = 0
  delta = 1.0
  print("********************************************************");
  print("* Ortho mode active - press right two buttons to exit! *");
  print("********************************************************");
  while True:
    time.sleep(0.1)
    GetInputs(CntBx)
    Robot.pos.x0 = -100 + ((GetInput(CntBx,  8)/255.0) * 200) # -100...100
    Robot.pos.y0 = -100 + ((GetInput(CntBx,  9)/255.0) * 200) # -100...100
    Robot.pos.z0 = -100 - ((GetInput(CntBx, 10)/255.0) * 200) # -100..-300
    RkInverse(Robot)
    #print('pos %4.0f %4.0f %4.0f -> angle %4.0f %4.0f %4.0f' % (Robot.pos.x0, Robot.pos.y0, Robot.pos.z0, Robot.angle.theta1, Robot.angle.theta2, Robot.angle.theta3))
    a0 = int(Robot.angle.theta1)
    a1 = int(Robot.angle.theta2)
    a2 = int(Robot.angle.theta3)
    #print a0, a1, a2
    if GetButton(CntBx, 6):
      SetOutput(Robot, 0, a1)
      SetOutput(Robot, 1, a0)
      SetOutput(Robot, 2, a2)
    if GetButton(CntBx, 6) and GetButton(CntBx, 7):
      return True

#----------------------------------------------------------------------------
#  These two global tuples will store the Handle and Data instances for the
#  two servo controllers.
#----------------------------------------------------------------------------

Robot = None
CntBx = None

#----------------------------------------------------------------------------
#  Find the robot and the control box and run the main loop.
#----------------------------------------------------------------------------

def DoRobotCtlBox():
  global Robot, CntBx
  context = usb1.USBContext()
  for Device in context.getDeviceList(skip_on_error=True):
    if Device.getVendorID() == VID:
      if Device.getProductID() == PID:
        ShowDevice(Device)
        if Device.getSerialNumber().endswith(RobotSerial):
          Robot = ServoController('Robot', Device.open())
        if Device.getSerialNumber().endswith(CntBxSerial):
          CntBx = ServoController('CntBx', Device.open())
  if Robot and CntBx:
    print 'Robot'
    ShowRobot(Robot)
    print 'CntBx'
    ShowCntBx(CntBx)
    GetInputs(Robot)
    print("********************************************************");
    print("* Press left two buttons together to enter ortho mode! *");
    print("********************************************************");
    while True:
      GetInputs(CntBx)
      SetOutput(Robot, 0, GetInput(CntBx,  8))
      SetOutput(Robot, 1, GetInput(CntBx,  9))
      SetOutput(Robot, 2, GetInput(CntBx, 10))
      if GetButton(CntBx, 4) and GetButton(CntBx, 5):
        return True
  return False

#----------------------------------------------------------------------------
#  Kick things off.
#----------------------------------------------------------------------------

if DoRobotCtlBox():
  if RkValidate(Robot, CntBx):
    OrthoControl(Robot, CntBx)

#----------------------------------------------------------------------------
#  End
#----------------------------------------------------------------------------
