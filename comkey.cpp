#include "comkey.h"

#define TRUE                    1
#define FALSE                   0
#define DEBUG_FLAG              0

bool gVerboseFlag = FALSE;
bool gDaemonFlag = FALSE;
int gComPortNum = 0;
char gSerialPortDevice[] = "/dev/ttyS1";
SERIAL_PORT_CLASS *gSerialPortObj = NULL;
char gKeyboardDevice[] = "/dev/input/event1";
KEYBOARD_CLASS *gKeyboardObj = NULL;

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

short char_to_keycode(char c, int *shift_flag) {
  short keycode;
  int sflag = 0;
  
  switch(c) {
    // these two are on many keyboard views on Android
  case ' ': keycode = KEY_SPACE; break;
  case '.': keycode = KEY_DOT; break;

    // normal keyboard
  case 'a': case 'A': keycode = KEY_A; break;
  case 'b': case 'B': keycode = KEY_B; break;
  case 'c': case 'C': keycode = KEY_C; break;
  case 'd': case 'D': keycode = KEY_D; break;
  case 'e': case 'E': keycode = KEY_E; break;
  case 'f': case 'F': keycode = KEY_F; break;
  case 'g': case 'G': keycode = KEY_G; break;
  case 'h': case 'H': keycode = KEY_H; break;
  case 'i': case 'I': keycode = KEY_I; break;
  case 'j': case 'J': keycode = KEY_J; break;
  case 'k': case 'K': keycode = KEY_K; break;
  case 'l': case 'L': keycode = KEY_L; break;
  case 'm': case 'M': keycode = KEY_M; break;
  case 'n': case 'N': keycode = KEY_N; break;
  case 'o': case 'O': keycode = KEY_O; break;
  case 'p': case 'P': keycode = KEY_P; break;
  case 'q': case 'Q': keycode = KEY_Q; break;
  case 'r': case 'R': keycode = KEY_R; break;
  case 's': case 'S': keycode = KEY_S; break;
  case 't': case 'T': keycode = KEY_T; break;
  case 'u': case 'U': keycode = KEY_U; break;
  case 'v': case 'V': keycode = KEY_V; break;
  case 'w': case 'W': keycode = KEY_W; break;
  case 'x': case 'X': keycode = KEY_X; break;
  case 'y': case 'Y': keycode = KEY_Y; break;
  case 'z': case 'Z': keycode = KEY_Z; break;

    // special chars on Android keyboard, page 1
  case '1': keycode = KEY_1; break;
  case '2': keycode = KEY_2; break;
  case '3': keycode = KEY_3; break;
  case '4': keycode = KEY_4; break;
  case '5': keycode = KEY_5; break;
  case '6': keycode = KEY_6; break;
  case '7': keycode = KEY_7; break;
  case '8': keycode = KEY_8; break;
  case '9': keycode = KEY_9; break;
  case '0': keycode = KEY_0; break;

  case '@': keycode = KEY_2; sflag = 1; break; // with SHIFT
  case '#': keycode = KEY_3; sflag = 1; break; // with SHIFT
    //case '€': keycode = KEY_5; break; // with ALTGR; not ASCII
  case '%': keycode = KEY_5; sflag = 1; break; // with SHIFT
  case '&': keycode = KEY_7; sflag = 1; break; // with SHIFT
  case '*': keycode = KEY_8; sflag = 1; break; // with SHIFT; alternative is KEY_KPASTERISK
  case '-': keycode = KEY_MINUS; break; // alternative is KEY_KPMINUS
  case '+': keycode = KEY_EQUAL; break; // with SHIFT; alternative is KEY_KPPLUS
  case '(': keycode = KEY_9; sflag = 1; break; // with SHIFT
  case ')': keycode = KEY_0; sflag = 1; break; // with SHIFT

  case '!': keycode = KEY_1; sflag = 1; break; // with SHIFT
  case '"': keycode = KEY_APOSTROPHE; sflag = 1; break; // with SHIFT, dead key
  case '\'': keycode = KEY_APOSTROPHE; sflag = 1; break; // dead key
  case ':': keycode = KEY_SEMICOLON; sflag = 1; break; // with SHIFT
  case ';': keycode = KEY_SEMICOLON; break;
  case '/': keycode = KEY_SLASH; break;
  case '?': keycode = KEY_SLASH; sflag = 1; break; // with SHIFT

  case ',': keycode = KEY_COMMA; break;

    // special chars on Android keyboard, page 2
  case '~': keycode = KEY_GRAVE; sflag = 1; break; // with SHIFT, dead key
  case '`': keycode = KEY_GRAVE; break; // dead key
  case '|': keycode = KEY_BACKSLASH; sflag = 1; break; // with SHIFT
    // missing because there's no ASCII code:  •, √, π, ÷, ×
  case '{': keycode = KEY_LEFTBRACE; sflag = 1; break; // with SHIFT
  case '}': keycode = KEY_RIGHTBRACE; sflag = 1; break; // with SHIFT

    // note: TAB key is handled elsewhere
    // missing because there's no ASCII code: £, ¥
  case '$': keycode = KEY_4; sflag = 1; break; // with SHIFT
    // missing because there's no ASCII code: °
  case '^': keycode = KEY_6; sflag = 1; break; // with SHIFT, dead key
  case '_': keycode = KEY_MINUS; sflag = 1; break; // with SHIFT
  case '=': keycode = KEY_EQUAL; break;
  case '[': keycode = KEY_LEFTBRACE; break;
  case ']': keycode = KEY_RIGHTBRACE; break;

    // missing because there's no ASCII code:  ™, ®, ©, ¶
  case '\\': keycode = KEY_BACKSLASH; break;
  case '<': keycode = KEY_COMMA; sflag = 1; break; // with SHIFT
  case '>': keycode = KEY_DOT; sflag = 1; break; // with SHIFT

    // missing because there's no ASCII code:  „, …

  default: keycode = -1;
  }
  
  //
  // Process & return shift key status
  //
  if (sflag == 0 && c >= 'A' && c <= 'Z') {
    sflag = 1;
  }
  *shift_flag = sflag;
  
  return keycode;
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
    DevicePath = (char *)malloc(len+1);
    if (DevicePath == NULL) {
      printf("Error: SERIAL_PORT_CLASS() allocate failed 2\n");
      break;
    }
    strcpy(DevicePath, device);
    break;
  } while (TRUE);
}

int SERIAL_PORT_CLASS::Init() {
  int fd;

  fd = open (DevicePath, O_RDWR | O_NOCTTY | O_SYNC);
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
    memcpy (ReceiveBuffer+ReceivePointer, buf, n);
    ReceivePointer += n;    
    *(ReceiveBuffer+ReceivePointer) = 0;
  }
}

//-----------------------------------------------------------------------------
// KEYBOARD_CLASS
//-----------------------------------------------------------------------------
KEYBOARD_CLASS::KEYBOARD_CLASS(char *device) {
  int len;
  
  Mode = 0;
  KeyCount = 0;
  DeviceHandle = 0;
  do {
    //
    // Copy device name
    //
    len = strlen(device);
    DevicePath = (char *)malloc(len+1);
    if (DevicePath == NULL) {
      printf("Error: KEYBOARD_CLASS() allocate failed 2\n");
      break;
    }
    strcpy(DevicePath, device);
    break;
  } while (TRUE);
}

int KEYBOARD_CLASS::Init() {
  int fd;
  if (Mode == 0) {
    if ((fd = open(DevicePath, O_RDWR)) > 0) {
      DeviceHandle = fd;
    } else {
      printf("Error: Open keyboard device failed\n");
    }
  }
  return 0;
}

void KEYBOARD_CLASS::SendKey(int key_code, int shift_flag) {
  struct input_event event;

  if (Mode == 1) {
    DeviceHandle = open(DevicePath, O_RDWR);
  }
  
  event.type = EV_KEY;
  
  if (shift_flag) {
    event.value = EV_PRESSED;
    event.code = KEY_LEFTSHIFT;
    write(DeviceHandle, &event, sizeof(struct input_event));
  }
  
  event.value = EV_PRESSED;
  event.code = key_code;
  write(DeviceHandle, &event, sizeof(struct input_event));

  event.value = EV_RELEASED;
  event.code = key_code;
  write(DeviceHandle, &event, sizeof(struct input_event));    
  
  if (shift_flag) {
    event.value = EV_RELEASED;
    event.code = KEY_LEFTSHIFT;
    write(DeviceHandle, &event, sizeof(struct input_event));
  }
  
  event.type = EV_SYN;
  event.code = SYN_REPORT;
  event.value = 0;
  write(DeviceHandle, &event, sizeof(event));
  
  if (Mode == 1) {
    close(DeviceHandle);
  }
}

int KEYBOARD_CLASS::Send(char *data) {
  struct input_event event;
  int len;
  char ch;
  int key_code;
  int fd;
  int shift_flag = false;
  
  len = strlen(data);
  for (int i=0; i<len; i++) {
    ch = data[i];
    if (ch == 0x0A) {
      key_code = KEY_ENTER;
    } else {
      key_code = char_to_keycode(ch, &shift_flag);
    }
    SendKey(key_code, shift_flag);
  }
}

//-----------------------------------------------------------------------------
// Main()
//-----------------------------------------------------------------------------
int MainLoop() {
  gSerialPortObj->Tick();
  if (gSerialPortObj->ReceivePointer > 0) {
    gKeyboardObj->Send(gSerialPortObj->ReceiveBuffer);
    gSerialPortObj->ReceivePointer = 0;
  }
  usleep(100*1000); // 1/10 second
}

int main(int argc, char **argv) {
  pid_t pid;
  int cmd_opt = 0;
  int com;
  char ch;
  int quit;
  
  printf("---------------------\n");
  printf(" COMKEY Version 1.00\n");
  printf("---------------------\n");

  while(1) {
    cmd_opt = getopt(argc, argv, "vdh?s:");

    /* End condition always first */
    if (cmd_opt == -1) {
      break;
    }

#if DEBUG_FLAG == 1
    /* Print option when it is valid */
    if (cmd_opt != '?') {
      printf("option:-%c\n", cmd_opt);
    }
#endif
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

      case 's':
        com = (int)strtol(optarg, NULL, 10);
        if (com == 1 || com == 2) {
          gComPortNum = com;
        } else {
          printf("Error: Invalid COM Port\n");
        }
        break;

      default:
        printf("Error: Not supported option\n");
        break;
    }
  }

  printf("Verbose Flag  = %d\n", gVerboseFlag);
  printf("Daemon Flag   = %d\n", gDaemonFlag);
  printf("COM Port      = %d\n", gComPortNum);

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
  if (gComPortNum == 0) {
    printf("Error: Invalid COM port number\n");
    return 0;    
  } else {
    sprintf(gSerialPortDevice, "/dev/ttyS%d", gComPortNum);
  }
  printf("Serial Device = %s\n", gSerialPortDevice);

  gSerialPortObj = new SERIAL_PORT_CLASS(gSerialPortDevice);
  gSerialPortObj->Init();
  gKeyboardObj = new KEYBOARD_CLASS(gKeyboardDevice);
  gKeyboardObj->Init();
    
  if (gDaemonFlag == TRUE) {
    pid = fork();               // PID > 0 代表是父程序
    if (pid == -1) {            // PID == -1 代表 fork 出錯
      printf("Error: fork() is failed\n");
      return 0;    
    } else if (pid == 0) {      // PID == 0 代表是子程序
      printf("Info: Run with Daemon Mode, PID is %d\n", getpid());
      while (TRUE) {
        MainLoop();
      }
      return 0;
    }
  } else {        
    printf("Info: Press ESC or ~ to quit\n");
    quit = 0;
    do {
      while (!kbhit()) {
        MainLoop();
      }
      ch = getch();
      if (ch == 0x1B || ch == '~') {
        quit = 1;
      }
    } while (quit == 0);
    printf("\n");
  }
  
  return 0;
}