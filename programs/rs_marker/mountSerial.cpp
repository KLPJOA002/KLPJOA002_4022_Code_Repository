#include "mountSerial.h"
#include <chrono>
#include <thread>

int mountSerial::serial_write(char *w_buf, uint32_t w_buf_len, char *r_buf, uint32_t r_buf_len) {
  char msg_buf[20];
  char in_buf[20];
  // ToDo: do something else 
  strcpy(msg_buf, w_buf);
  memset(&in_buf, '\0', sizeof(in_buf));
  write(serial_port, msg_buf, strlen(msg_buf));

  //ToDo: polling delay for 1 second
  //std::this_thread::sleep_for(std::chrono::milliseconds(10));

  int num_bytes = read(serial_port, &in_buf, sizeof(in_buf));
  if (num_bytes < 0) {
    printf("Error reading: %s\n", strerror(errno));
    close(serial_port);
    return 1;
    }
  strcpy(r_buf, in_buf);
  return 0;
}

// find arduino serial port, make path absolute, 
// establish terminal connection
mountSerial::mountSerial() {

  namespace fs = std::filesystem;
  fs::path linkPath;
  // ToDo: Make this less shit
  const fs::path serialPorts{"/dev/serial/by-id/"};
  for (auto const& dir_entry : fs::directory_iterator{serialPorts}) {
    if (strstr((dir_entry.path()).c_str(), "Arduino")) {
      linkPath = dir_entry.path();
    }
  }

  char serial_port_addr[256];
  strcpy(serial_port_addr, fs::canonical(linkPath).c_str());
  serial_port = open(serial_port_addr, O_RDWR);

  printf("Serial Port: %i\n", serial_port);
  if (serial_port < 0) {
    printf("Unable to find Arduino");
    printf("Error %i from open: %s\n", errno, strerror(errno));
  } else {
    printf("Connection Established at %s\n", serial_port_addr);
  }

  fcntl(serial_port, F_SETFL, 0);

  // Read in existing settings, and handle any error
  if (tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      close(serial_port);
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 20;    // Wait for up to 2s (10 deciseconds)
  tty.c_cc[VMIN] = 0;

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      close(serial_port);
  }

  // ToDo: Clear buffer

}

mountSerial::~mountSerial() {
  close(serial_port);
}

int mountSerial::homeCamera(char* read_buf)  {
  char msg_buf[] = "H0";
  serial_write(msg_buf, 3, read_buf, 10);
  return 0;
}

int mountSerial::setCameraAngle(char *read_buf, float angle) {
  char msg_buf[10];
  sprintf(msg_buf, "S%05.1f", angle);
  serial_write(msg_buf, 3, read_buf, 10);
  return 0;
}

float mountSerial::getCameraAngle(char *read_buf) {
  char msg_buf[] = "G";
  serial_write(msg_buf, 2, read_buf, 10);
  return 0;
}

