#include <stdio.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>

#include "driver/usc.h"

/* メイン関数 */
int main(int argc, char** argv)
{
  int fd;
  int vendor, product;
  int status;

  if((fd = open("/dev/usc0", O_RDWR)) == -1){
    perror("device open error\n");
    exit(1);
  }

  if(ioctl(fd, USC_GET_VENDOR, &vendor) == -1){
    perror("ioctl error\n");
    exit(1);
  }
  printf("Vendor %x\n", vendor);

  if(ioctl(fd, USC_GET_PRODUCT, &product) == -1){
    perror("ioctl error\n");
    exit(1);
  }
  printf("Product %x\n", product);



  if(ioctl(fd, USC_REQUEST_READ) == -1){
    perror("ioctl error\n");
    exit(1);
  }

  if(ioctl(fd, USC_GET_READ_STATUS, &status) == -1){
    perror("ioctl error\n");
    exit(1);
  }
  printf("read status %d\n", status);




  if(ioctl(fd, USC_CONTINUOUS_READ) == -1){
    perror("ioctl error\n");
    exit(1);
  }

  if(ioctl(fd, USC_GET_READ_STATUS, &status) == -1){
    perror("ioctl error\n");
    exit(1);
  }
  printf("read status %d\n", status);




  if(ioctl(fd, USC_DR_SET) == -1){
    perror("ioctl error\n");
    exit(1);
  }

  if(ioctl(fd, USC_GET_WRITE_STATUS, &status) == -1){
    perror("ioctl error\n");
    exit(1);
  }
  printf("write status %d\n", status);



  if(ioctl(fd, USC_DDR_SET) == -1){
    perror("ioctl error\n");
    exit(1);
  }

  if(ioctl(fd, USC_GET_WRITE_STATUS, &status) == -1){
    perror("ioctl error\n");
    exit(1);
  }
  printf("write status %d\n", status);
 
  close(fd);

  return 0;
}
