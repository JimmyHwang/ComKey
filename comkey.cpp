#include "comkey.h"

#define TRUE                    1
#define FALSE                   0

bool gVerboseFlag = FALSE;
bool gDaemonFlag = FALSE;
char gSerialPortDevice[] = "/dev/ttyS1";
SERIAL_PORT_CLASS *gSerialPortObj = NULL;

//-----------------------------------------------------------------------------
// Common Functions
//-----------------------------------------------------------------------------
/* reads from keypress, doesn't echo */
int getch(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}

int kbhit() {
  static const int STDIN = 0;
  static bool initialized = false;

  if (! initialized) {
    // Use termios to turn off line buffering
    termios term;
    tcgetattr(STDIN, &term);
    term.c_lflag &= ~ICANON;
    tcsetattr(STDIN, TCSANOW, &term);
    setbuf(stdin, NULL);
    initialized = true;
  }

  int bytesWaiting;
  ioctl(STDIN, FIONREAD, &bytesWaiting);
  return bytesWaiting;
}

int set_interface_attribs (int fd, int speed, int parity)
{
  struct termios tty;
  if (tcgetattr (fd, &tty) != 0)
  {
    printf ("error %d from tcgetattr", errno);
    return -1;
  }

  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;         // disable break processing
  tty.c_lflag = 0;                // no signaling chars, no echo,
  // no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_cc[VMIN]  = 0;            // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr (fd, TCSANOW, &tty) != 0)
  {
    printf ("error %d from tcsetattr", errno);
    return -1;
  }
  return 0;
}

void
set_blocking (int fd, int should_block)
{
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0)
  {
    printf ("error %d from tggetattr", errno);
    return;
  }

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  if (tcsetattr (fd, TCSANOW, &tty) != 0)
  printf ("error %d setting term attributes", errno);
}

//-----------------------------------------------------------------------------
// SERIAL_PORT_CLASS
//-----------------------------------------------------------------------------
SERIAL_PORT_CLASS::SERIAL_PORT_CLASS(char *device) {
  int len;
  
  DeviceHandle = 0;
  do {
    //
    // Initialize receive buffer & data
    //
    ReceiveBuffer = (char *)malloc(0x100);
    if (ReceiveBuffer == NULL) {
      printf("Error: SERIAL_PORT_CLASS() allocate failed 1\n");
      break;
    }
    ReceivePointer = 0;
    //
    // Copy device name
    //
    len = strlen(device);
    SerialPortDevice = (char *)malloc(len+1);
    if (SerialPortDevice == NULL) {
      printf("Error: SERIAL_PORT_CLASS() allocate failed 2\n");
      break;
    }
    strcpy(SerialPortDevice, device);
    break;
  } while (TRUE);
}

int SERIAL_PORT_CLASS::Init() {
  int fd;

  fd = open (SerialPortDevice, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    //printf ("error %d opening %s: %s", errno, portname, strerror (errno));
  } else {
    printf ("Info: Initialize serial port......\n");
    set_interface_attribs (fd, B9600, 0); // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                 // set no blocking
    DeviceHandle = fd;
  }
  
  return fd;
}

int SERIAL_PORT_CLASS::Send(char *data) {
  int len;
  
  len = strlen(data);
  write (DeviceHandle, data, len);    // send 7 character greeting
  usleep ((7 + 25) * 100);             // sleep enough to transmit the 7 plus
  
  return len;
}

void SERIAL_PORT_CLASS::Tick() {
  char buf[0x100];
  int n;
  
  n = read (DeviceHandle, buf, sizeof buf);  // read up to 100 characters if ready to read
  if (n > 0) {
    // printf("<n=%d>", n);
    memcpy (ReceiveBuffer+ReceivePointer, buf, n);
    ReceivePointer += n;    
  }
}

//-----------------------------------------------------------------------------
// Main()
//-----------------------------------------------------------------------------
int MainLoop() {
  gSerialPortObj->Tick();
  if (gSerialPortObj->ReceivePointer > 0) {
    printf("%s", gSerialPortObj->ReceiveBuffer);
    gSerialPortObj->ReceivePointer = 0;
  }
  fflush(stdout); 
  usleep(100*1000); // 1/10 second
}

int main(int argc, char **argv) {
  pid_t pid;
  int cmd_opt = 0;
  
  printf("COMKEY Version 1.00\n");

  // fprintf(stderr, "argc:%d\n", argc);
  while(1) {
      // fprintf(stderr, "proces index:%d\n", optind);
    cmd_opt = getopt(argc, argv, "vdh?");

    /* End condition always first */
    if (cmd_opt == -1) {
      break;
    }

    /* Print option when it is valid */
    if (cmd_opt != '?') {
      printf("option:-%c\n", cmd_opt);
    }

    /* Lets parse */
    switch (cmd_opt) {
      /* No args */
      case 'v':
        gVerboseFlag = TRUE;
        break;

      /* Single arg */
      case 'd':
        gDaemonFlag = TRUE;        
        break;

      default:
        printf("Not supported option\n");
        break;
    }
  }

  /* Do we have args? */
  if (gVerboseFlag) {
    if (argc > optind) {
      int i = 0;
      for (i = optind; i < argc; i++) {
        printf("argv[%d] = %s\n", i, (char *)argv[i]);
      }
    }
  }
  
  //
  // Startup
  //
  gSerialPortObj = new SERIAL_PORT_CLASS(gSerialPortDevice);
  gSerialPortObj->Init();
  
  if (gDaemonFlag == TRUE) {
    pid = fork();               // PID > 0 代表是父程序
    if (pid == -1) {            // PID == -1 代表 fork 出錯
      printf("Error: fork() is failed\n");
      return 0;    
    } else if (pid == 0) {      // PID == 0 代表是子程序
      printf("Info: PID is %d\n", getpid());
      while (TRUE) {
        MainLoop();
      }
      return 0;
    }
  } else {                      
    printf("Info: Press any key to quit\n");
    while (!kbhit()) {
      MainLoop();
    }
    getch(); 
  }
  
  return 0;
}