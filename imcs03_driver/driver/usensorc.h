struct udata{
  unsigned short time; /* 内部カウンタ (1ms周期)の値 */
  unsigned short magicno; /* EEPROM 内の数字 */
  unsigned short ad[8];/* A/D コンバータの値(10bit) */
  unsigned char P9DR;
  unsigned char PADR;
  unsigned char PBDR;
  char dmy[41];
};

struct udr{
  unsigned char retval;
  unsigned char P9DR;
  unsigned char PADR;
  unsigned char PBDR;
  char dmy[60];
};

struct uddr{
  unsigned char P9DDR;
  unsigned char PADDR;
  unsigned char PBDDR;
  char dummy[61];
};
