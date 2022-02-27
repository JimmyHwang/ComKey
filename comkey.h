#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <malloc.h>
#include <fcntl.h>
#include <linux/input.h>

#define EV_PRESSED 1
#define EV_RELEASED 0
#define EV_REPEAT 2

class SERIAL_PORT_CLASS {
public:
  char *DevicePath;
  int DeviceHandle;
  char *ReceiveBuffer;
  int ReceivePointer;
  
  // Default constructor
  SERIAL_PORT_CLASS(char *device);

  int Init();
  int Send(char *data);
  void Tick();
  
private:

};


class KEYBOARD_CLASS {
public:
  char *DevicePath;
  int DeviceHandle;
  int KeyCount;
  
  // Default constructor
  KEYBOARD_CLASS(char *device);

  int Init();
  int Send(char *data);
  void SendKey(int key_code);
    
private:

};
