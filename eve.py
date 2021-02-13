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
#----------------------------------------------------------------------------

import usb1, struct, time

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
  HandleData['data'] = list(struct.unpack(FormatInput12, HandleData['handle'].controlRead(0xc0, REQUEST_GET_SERVO_SETTINGS, 0, 0, 7*12)))

#----------------------------------------------------------------------------
#  Get the value (0..255) of an input on a maestro controller.  This call
#  remaps the raw input values from their native range (0..1023). 
#----------------------------------------------------------------------------

def GetInput(HandleData, Channel):
  return HandleData['data'][Channel*4]/4

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
    HandleData['handle'].controlWrite(0x40, REQUEST_SET_TARGET, (Value*15)+4088, Channel, 0, 0)
    HandleData['data'][Channel*4] = Value*4

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
  Position, Target, Speed, Acceleration = HandleData['data'][Channel*4:Channel*4+4]
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
#  These two global tuples will store the Handle and Data instances for the
#  two servo controllers.
#----------------------------------------------------------------------------

Robot = None
CntBx = None

#----------------------------------------------------------------------------
#  Find the robot and the control box and run the main loop.
#----------------------------------------------------------------------------

def DoRobotCtlBox():
  context = usb1.USBContext()
  for Device in context.getDeviceList(skip_on_error=True):
    if Device.getVendorID() == VID:
      if Device.getProductID() == PID:
        ShowDevice(Device)
        if Device.getSerialNumber().endswith(RobotSerial):
          Robot = dict(handle=Device.open(), data=None)
        if Device.getSerialNumber().endswith(CntBxSerial):
          CntBx = dict(handle=Device.open(), data=None)
  if Robot and CntBx:
    print 'Robot'
    ShowRobot(Robot)
    print 'CntBx'
    ShowCntBx(CntBx)
    GetInputs(Robot)
    print("********************************************************");
    print("* Press left two buttons together for gracefully exit! *");
    print("********************************************************");
    while True:
      GetInputs(CntBx)
      SetOutput(Robot, 0, GetInput(CntBx,  8))
      SetOutput(Robot, 1, GetInput(CntBx,  9))
      SetOutput(Robot, 2, GetInput(CntBx, 10))
      if GetButton(CntBx, 4) and GetButton(CntBx, 5):
        break

#----------------------------------------------------------------------------
#  Kick things off.
#----------------------------------------------------------------------------

DoRobotCtlBox()

#----------------------------------------------------------------------------
#  End
#----------------------------------------------------------------------------
