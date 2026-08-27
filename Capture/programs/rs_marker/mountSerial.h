#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <fcntl.h> 
#include <errno.h> 
#include <termios.h> 
#include <unistd.h> 
#include <filesystem>

class mountSerial {
  private:
    int serial_port;
    int serial_write(char *w_buf, uint32_t w_buf_len, char *r_buf, uint32_t r_buf_len);
  public:
    mountSerial();
    ~mountSerial();
    int homeCamera(char *read_buf);
    int setCameraAngle(char *read_buf, float angle);
    float getCameraAngle(char *read_buf);
    struct termios tty;
};
