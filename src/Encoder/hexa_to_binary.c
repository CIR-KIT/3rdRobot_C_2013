#include <stdio.h>
#include <stdlib.h>
#include "encoder.h"

void hexa_to_binary(const char hexa, int *binary)
{
  static int i = 0;
  static char temp[8];
  static const int fixed_hexa[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
  for(i=0;i<8;i++){
    temp[i] = hexa & fixed_hexa[i];
    if(temp[i] != 0x00){
      temp[i] = 0x01;
    }
    binary[i] = (int)temp[i];
  } 
}