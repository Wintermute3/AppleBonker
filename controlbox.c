//----------------------------------------------------------------------------
//  Make sure libusb-dev is installed:
//
//    dpkg --get-selections | grep libusb
//
//  If it is not, then install it:
//
//    sudo apt-get install libusb-dev
//
//  Compile and link this program (assuming libusb-1.0 is installed) using:
//
//    gcc controlbox.c -o controlbox `pkg-config --libs --cflags libusb-1.0`
//----------------------------------------------------------------------------

/*
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
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/time.h>
#include <libusb.h>
#include <stdint.h>
#include <time.h>

//----------------------------------------------------------------------------
//  Structures used to monitor and control the maestro controllers.
//----------------------------------------------------------------------------

#pragma pack(1)

typedef struct ServoSetting_s {
    uint16_t Position    ;
    uint16_t Target      ;
    uint16_t Speed       ;
    uint8_t  Acceleration;
} ServoSetting_t;

typedef struct Maestro12_s {
  ServoSetting_t        Servos[12];
  libusb_device_handle* Handle    ;
} Maestro12_t, * Maestro12_p;

#pragma pack()

//----------------------------------------------------------------------------
//  Global constants and pointers to maestro controller structures.
//----------------------------------------------------------------------------

#define REQUEST_SET_TARGET          0x85
#define REQUEST_GET_SERVO_SETTINGS  0x87

static Maestro12_p Robot, CntBx;

//----------------------------------------------------------------------------
//  Move data to/from a maestro controller.  Timeout is 1000ms.
//----------------------------------------------------------------------------

static void ControlTransfer(Maestro12_p Maestro,
    uint8_t  RequestType,
    uint8_t  Request    ,
    uint16_t Value      ,
    uint16_t Index      ,
    uint8_t* Data       , // data buffer
    uint16_t Length       // length of data buffer
  ) {
  libusb_control_transfer(Maestro->Handle, RequestType, Request, Value, Index, Data, Length, 1000);
}

//----------------------------------------------------------------------------
//  Get the current status of all inputs on a maestro controller.
//----------------------------------------------------------------------------

static void GetInputs(Maestro12_p Maestro) {
  ControlTransfer(Maestro, 0xC0, REQUEST_GET_SERVO_SETTINGS, 0, 0, (uint8_t *) Maestro->Servos, sizeof(ServoSetting_t)*12);
}

//----------------------------------------------------------------------------
//  Get the value (0..255) of an input on a maestro controller.  This call
//  returns the value read by the most recent call to GetInputs(), remapped
//  from the range (0..1023).
//----------------------------------------------------------------------------

static uint8_t GetInput(Maestro12_p Maestro,
    uint8_t Channel // channel index 0..11
  ) {
  return (uint8_t) (Maestro->Servos[Channel].Position >> 2);
}

//----------------------------------------------------------------------------
//  Get the value (0 or 255) of an button on a maestro controller.  This call
//  returns the value read by the most recent call to GetInputs().
//----------------------------------------------------------------------------

static uint8_t GetButton(Maestro12_p Maestro,
    uint8_t Channel // channel index 0..11
  ) {
  return (GetInput(Maestro, Channel) & 0x80) ? 0xff : 0x00;
}

//----------------------------------------------------------------------------
//  Set the value (0..255) of an output on a maestro controller.  This range
//  is remapped to (4000..8000) before transmission to the controller.
//----------------------------------------------------------------------------

static void SetOutput(Maestro12_p Maestro,
    uint8_t Channel , // channel index 0..11
    uint8_t Position  // position target
  ) {
  uint16_t Value = (((uint16_t) Position) * 15) + 4088;
  ControlTransfer(Maestro, 0x40, REQUEST_SET_TARGET, Value, Channel, 0, 0);
}

//----------------------------------------------------------------------------
//  Return an ASCIIZ string pointer to the serial number of a usb device.
//----------------------------------------------------------------------------

static char * GetSerialNumber(libusb_device_handle* Handle, uint8_t SerialIndex) {
  static uint8_t Buffer[40];
  libusb_get_string_descriptor_ascii(Handle, SerialIndex, Buffer, sizeof(Buffer));
  return Buffer;
}

//----------------------------------------------------------------------------
//  Return function of robot channel.
//----------------------------------------------------------------------------

static char * RobotChannelName(int i) {
  switch(i) {
    case 0: return "right";
    case 1: return "front";
    case 2: return "left" ;
    case 6: return "ping" ; // 1ms pulse to trigger echo
    case 7: return "echo" ; // echo peak value correlates to distance
  }
  return "****";
}

//----------------------------------------------------------------------------
//  Return function of control box channel.
//----------------------------------------------------------------------------

static char * CntBxChannelName(int i) {
  switch(i) {
    case  0: return "led-0"   ;
    case  1: return "led-1"   ;
    case  2: return "led-2"   ;
    case  3: return "led-3"   ;
    case  4: return "button-0"; // normally high
    case  5: return "button-1"; // normally high
    case  6: return "button-2"; // normally high
    case  7: return "button-3"; // normally high
    case  8: return "knob-0"  ;
    case  9: return "knob-1"  ;
    case 10: return "knob-2"  ;
    case 11: return "knob-3"  ;
  }
  return "****";
}

//----------------------------------------------------------------------------
//  Display servo parameters on robot.
//----------------------------------------------------------------------------

static void ShowServos(void) {
  int i;
  GetInputs(Robot);
  for (i = 0; i < 3; i++) {
    printf("      [%1d.%-5s]: Pos[%4d] Target[%4d] Speed[%1d] Acceleration[%2d]\n", i, RobotChannelName(i),
      Robot->Servos[i].Position     ,
      Robot->Servos[i].Target       ,
      Robot->Servos[i].Speed        ,
      Robot->Servos[i].Acceleration);
} }

//----------------------------------------------------------------------------
//  Exercise ultrasonic distance sensor on robot.  Output a 1ms pulse on the
//  ping channel, then monitor the echo channel and take the highest value
//  returned during the echo window.
//----------------------------------------------------------------------------

static uint8_t GetDistance(void) {
  uint8_t i, v, vmax = 0;
  SetOutput(Robot, 6,  0); usleep(1000);
  SetOutput(Robot, 6,255); usleep(1000);
  SetOutput(Robot, 6,  0);
  for (i = 0; i < 8; i++) {
    GetInputs(Robot);
    v = GetInput(Robot,7);
    if (vmax < v) {
      vmax = v;
  } }
  return vmax;
}

//----------------------------------------------------------------------------
//  Constants to identify the controllers for the robot and control box.
//----------------------------------------------------------------------------

#define VID_POLOLU  0x1ffb

#define PID_MAESTRO_06  0x0089
#define PID_MAESTRO_12  0x008a
#define PID_MAESTRO_18  0x008b
#define PID_MAESTRO_24  0x008c

#define SN_ROBOT  "59243"
#define SN_CNTBX  "68800"

//----------------------------------------------------------------------------
//  Look for the robot and control box on the usb bus.
//----------------------------------------------------------------------------

static void UsbScanDevices(libusb_device** DeviceList) {
  libusb_device* Device;
  int i = 0;
  while ((Device = DeviceList[i++]) != NULL) {
    struct libusb_device_descriptor Descriptor;
    int r = libusb_get_device_descriptor(Device, &Descriptor);
    if (r < 0) {
      fprintf(stderr, "failed to get device descriptor");
      return;
    }
    if ((Descriptor.idVendor == VID_POLOLU) && (Descriptor.idProduct == PID_MAESTRO_12)) {
      libusb_device_handle * Handle;
      printf("%04x:%04x (bus %d, device %d)\n",
        Descriptor.idVendor, Descriptor.idProduct,
          libusb_get_bus_number(Device), libusb_get_device_address(Device));
      if (libusb_open(Device, &Handle)) {
        perror("Open error");
      } else {
        char * Serial = GetSerialNumber(Handle, Descriptor.iSerialNumber);
        printf("  Serial Number-> '%s'\n", Serial);
        if (strstr(Serial, SN_ROBOT)) {
          if (Robot = (Maestro12_p) calloc(sizeof(Maestro12_t), 1)) {
            Robot->Handle = Handle;
            printf("    Robot\n");
            ShowServos();
            printf("      Distance %d\n", GetDistance());
          }
        } else {
          if (strstr(Serial, SN_CNTBX)) {
            if (CntBx = (Maestro12_p) calloc(sizeof(Maestro12_t), 1)) {
              CntBx->Handle = Handle;
              printf("    CntBx\n");
            }
          } else {
            libusb_close(Handle);
} } } } } }

//----------------------------------------------------------------------------
//  Open the usb interface and get list of devices to examine.
//----------------------------------------------------------------------------

static int UsbScan( void) {
  libusb_device** DeviceList;
  if (libusb_init(NULL) < 0) {
    return 0;
  }
  if (libusb_get_device_list(NULL, &DeviceList) < 0) {
    return 0;
  }
  UsbScanDevices(DeviceList);
  libusb_free_device_list(DeviceList, 1);
  return 1;
}

//----------------------------------------------------------------------------
//  Do i/o with the control box.
//----------------------------------------------------------------------------

static int DoCntBx(void) {
  static int i = 0;
  GetInputs(CntBx);
  SetOutput(CntBx, 0, GetInput(CntBx, 8) & 0x80 ? 255-GetButton(CntBx,4) : GetButton(CntBx,4));
  SetOutput(CntBx, 1, GetInput(CntBx, 9) & 0x80 ? 255-GetButton(CntBx,5) : GetButton(CntBx,5));
  SetOutput(CntBx, 2, GetInput(CntBx,10) & 0x80 ? 255-GetButton(CntBx,6) : GetButton(CntBx,6));
  SetOutput(CntBx, 3, GetInput(CntBx,11) & 0x80 ? 255-GetButton(CntBx,7) : GetButton(CntBx,7));
  return (GetButton(CntBx,4) && GetButton(CntBx,5)) ? 0 : 1;
}

//----------------------------------------------------------------------------
//  Do i/o with the robot.
//----------------------------------------------------------------------------

static void DoRobot(void) {
  static int i = 0;
  SetOutput(Robot, 0, GetInput(CntBx,  8));
  SetOutput(Robot, 1, GetInput(CntBx,  9));
  SetOutput(Robot, 2, GetInput(CntBx, 10));
  if (i++ % 100 == 0) {
    printf("\r<%03d>", GetDistance());
    fflush(stdout);
} }

//----------------------------------------------------------------------------
//  Monitor the control box and control the robot.
//----------------------------------------------------------------------------

int main(void) {
  if (UsbScan()) {
    if (CntBx) {
      printf("********************************************************\n");
      printf("* Press left two buttons together for gracefully exit! *\n");
      printf("********************************************************\n");
      while (DoCntBx()) {
        if (Robot) {
          DoRobot();
        } else {
          printf("%3d %3d %3d %3d\n",
            GetInput(CntBx,  8),
            GetInput(CntBx,  9),
            GetInput(CntBx, 10),
            GetInput(CntBx, 11)
          );
        }
        usleep(10000); // 10ms
  } } }
  libusb_exit(NULL);
  printf("\n");
  return 0;
}

//----------------------------------------------------------------------------
//  End
//----------------------------------------------------------------------------
