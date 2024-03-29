#include "_imcs_set.h"
#include "imcs_set.h"

void set_imcs03(int fd, struct uddr *cmd){
  cmd->PBDDR = 0xff;	//PB0~7 is output port.
  cmd->PADDR = 0x0;	//PA0~7 is input port.
  if(ioctl(fd, USC_DDR_SET) < 0){
    fprintf(stderr, "ioctl: USC_DDR_SET error\n");
  }
  if(write(fd, cmd, sizeof(*cmd)) < 0){
    fprintf(stderr, "write error : USC_DDR_SET\n");
    exit(1);
  }
  if(ioctl(fd, USC_DR_SET) < 0){
    fprintf(stderr, "ioctl: USC_DR_SET error in SP thread\n");
    exit(1);
  }
  if (ioctl(fd, USC_CONTINUOUS_READ) < 0){
    fprintf(stderr, "ioctl: USC_CONTINUOUS_READ error\n");
    exit(1);
  }
  if(ioctl(fd, USC_BUFREAD) < 0){
    fprintf(stderr, "ioctl : imcs03 USC_BUFREAD error\n");
    exit(1);
  }
}

void set_imcs01(int fd, struct ccmd *cmd, struct uout *obuf)
{
  int i = 0;
  if(ioctl(fd, URBTC_CONTINUOUS_READ) < 0){
    fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");
    exit(1);
  }
  if(ioctl(fd, URBTC_BUFREAD) < 0){
    fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");
    exit(1);
  }

  cmd->selout = SET_SELECT | CH0 | CH1 | CH2 | CH3;// All PWM.
  cmd->selin = SET_SELECT; // All input using for encoder count.
  cmd->setoffset = CH0 | CH1 | CH2 | CH3;
  cmd->offset[0] = cmd->offset[1] = cmd->offset[2] = cmd->offset[3] = 58981; //1/2
  cmd->setcounter = CH0 | CH1 | CH2 | CH3;
  cmd->counter[1] = -3633; //(-1)*67[deg]*(1453/27), initialize.
  cmd->counter[2] = 0;
  cmd->posneg = SET_POSNEG | CH0 | CH1 | CH2 | CH3;//POS PWM out.
  cmd->breaks = SET_BREAKS | CH0 | CH1 | CH2 | CH3;//No Brake;
  cmd->magicno = 0x00;

  if(ioctl(fd, URBTC_COUNTER_SET) < 0){
    fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");
    exit(1);
  }
  if(write(fd, cmd, sizeof(*cmd)) < 0){
    fprintf(stderr, "Write error set imcs01\n");
    exit(1);
  }
  cmd->setcounter = 0;
  if(write(fd, cmd, sizeof(*cmd)) < 0){
    fprintf(stderr, "Write error set imcs01\n");
    exit(1);
  }
  for(i = 0; i < 4; i++){
    obuf->ch[i].x = 0;
    obuf->ch[i].d = 0;
    obuf->ch[i].kp = 0;
    obuf->ch[i].kpx = 1;
    obuf->ch[i].kd = 0;
    obuf->ch[i].kdx = 1;
    obuf->ch[i].ki = 0;
    obuf->ch[i].kix = 1;
  }
  if(ioctl(fd, URBTC_DESIRE_SET) < 0){
    fprintf(stderr, "ioctl: URBTC_DESIRE_SET error\n");
    exit(1);
  }
  if(write(fd, obuf, sizeof(*obuf)) < 0){
    fprintf(stderr, "Write error forward\n");
    exit(1);
  }
}
