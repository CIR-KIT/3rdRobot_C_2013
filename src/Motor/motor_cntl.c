#include "_motor_cntl.h"
#include "motor_cntl.h"
/* speed 2 ~ 4 */
void forward(int fd, struct ccmd *cmd, int speed)
{
  cmd->offset[0] = 65535; //5V CH101 PIN2
  cmd->offset[1] = 32767; //0V CH102 PIN2
 if(ioctl(fd, URBTC_COUNTER_SET) < 0){
    fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");
    exit(1);
  }
  if(write(fd, cmd, sizeof(*cmd)) < 0){
    fprintf(stderr, "write cmd error\n");
    exit(1);
  }
  usleep(5000);
  cmd->offset[0] = 65535; //5V CH101 PIN2
  cmd->offset[1] = 32767 + speed * 6553; //CH102 PIN2
 if(ioctl(fd, URBTC_COUNTER_SET) < 0){
    fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");
    exit(1);
  }
  if(write(fd, cmd, sizeof(*cmd)) < 0){
    fprintf(stderr, "write cmd error\n");
    exit(1);
  }
}

void stop(int fd, struct ccmd *cmd)
{
  cmd->offset[0] = 65535; //5V CH101 PIN2
  cmd->offset[1] = 32767; //0V CH102 PIN2
 if(ioctl(fd, URBTC_COUNTER_SET) < 0){
    fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");
    exit(1);
  }
  if(write(fd, cmd, sizeof(*cmd)) < 0){
    fprintf(stderr, "write cmd error\n");
    exit(1);
  }
}

void back(int fd, struct ccmd *cmd)
{
  stop(fd, cmd);
  usleep(10000);
  cmd->offset[0] = 32767; //0V CH101 PIN2
  cmd->offset[1] = 32767; //CH102 PIN2
 if(ioctl(fd, URBTC_COUNTER_SET) < 0){
    fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");
    exit(1);
  }
  if(write(fd, cmd, sizeof(*cmd)) < 0){
    fprintf(stderr, "write cmd error\n");
    exit(1);
  }
  sleep(1);
  cmd->offset[0] = 32767; //0V CH101 PIN2
  cmd->offset[1] = 52426; //CH102 PIN2
 if(ioctl(fd, URBTC_COUNTER_SET) < 0){
    fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");
    exit(1);
  }
  if(write(fd, cmd, sizeof(*cmd)) < 0){
    fprintf(stderr, "write cmd error\n");
    exit(1);
  }
}
