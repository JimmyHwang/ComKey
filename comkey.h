#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <malloc.h>

class SERIAL_PORT_CLASS {
public:
  int DeviceHandle;
  char *SerialPortDevice;
  char *ReceiveBuffer;
  int ReceivePointer;
  
  // Default constructor
  SERIAL_PORT_CLASS(char *device);

  int Init();
  int Send(char *data);
  void Tick();
  
private:

};
