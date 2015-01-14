#include <fcntl.h>          /* for open */
#include <stdio.h>
#include <unistd.h>         /* for read, close */
#include <sys/ioctl.h>      /* for ioctl */
#include <sys/types.h>      /* for open */
#include <sys/stat.h>       /* for open */

#include <signal.h>
#include <stdlib.h>			//add t.miyo

#include "driver/usc.h"          /* Linux specific part */
#include "driver/usensorc.h"        /* OS independent part */

#undef __BIG_ENDIAN

void exit_program();

int quit_flag = 1;
int fd;

int main(int argc, char **argv){
  struct udata buf;
  struct uddr  cmd;
  struct udr   cmd1;
  int i;
  char *dev = "/dev/usc0";
  
  signal(SIGINT, exit_program);

  if (argc>1)
	dev = argv[1];

  /* デバイスのオープン */
  if ((fd = open(dev, O_RDWR)) == -1) {
    fprintf(stderr, "%s: Open error\n", dev);
    exit(1);
  }

#if 1
  /* DDRレジスタの書き込み */
  cmd.P9DDR = 0xff;
  cmd.PADDR = 0xff;
  cmd.PBDDR = 0xff;
  if (ioctl(fd, USC_DDR_SET) < 0){
    fprintf(stderr, "ioctl: USC_DDR_SET error\n");
    exit(1);
  }

  if (write(fd, &cmd, sizeof(cmd)) < 0) {
    fprintf(stderr, "write error\n");
    exit(1);
  }
#endif

#if 1
  /* DRレジスタの書き込み */
  cmd1.retval=1;
 cmd1.P9DR = 0x00;
 cmd1.PADR = 0x00;
 cmd1.PBDR = 0x00;
 if (ioctl(fd, USC_DR_SET) < 0){
    fprintf(stderr, "ioctl: USC_DR_SET error\n");
    exit(1);
  }

  if (write(fd, &cmd1, sizeof(cmd1)) < 0) {
    fprintf(stderr, "write error\n");
    exit(1);
  }
#endif

  //chg t.miyo
  ioctl(fd,USC_CONTINUOUS_READ);

  /* DRレジスタの読み出し */
 if (ioctl(fd, USC_BUFREAD) < 0){
    fprintf(stderr, "ioctl: USC_CONTINUOUS_READ error\n");
    exit(1);
  }

  while(quit_flag) {
    if ((i = read(fd, &buf, sizeof(buf))) != sizeof(buf)) {
      fprintf(stderr, "Warning: read size mismatch (%d!=%d).\n", i, sizeof(buf));
      continue;
    }
    for (i=0; i<8; i++) {
#if __BYTE_ORDER == __BIG_ENDIAN
      buf.ad[i] = (0xff & buf.ad[i])<<8 | (0xff00 & buf.ad[i])>>8;
#endif
      buf.ad[i] = buf.ad[i] >> 5;
    }
#if __BYTE_ORDER == __BIG_ENDIAN
    buf.time = (0xff & buf.time)<<8 | (0xff00 & buf.time)>>8;
#endif
#if 1
    printf("%d %d %d %d %d %d %d %d %d %d %x %x %x\n",
	   buf.time, buf.magicno, buf.ad[0], buf.ad[1], buf.ad[2], buf.ad[3],
	   buf.ad[4], buf.ad[5], buf.ad[6], buf.ad[7],
	   buf.P9DR, buf.PADR, buf.PBDR);
#else
	printf("%d\n", buf.time);
#endif
  }

  close(fd);

  return 0;
}

void exit_program(sig, code, scp, addr)
int sig;
int code;
struct sigcontext *scp;
char *addr;
{
  quit_flag = 0;
  fprintf(stderr, "kill signal is received\n");
}
