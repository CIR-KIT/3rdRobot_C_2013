struct udata{
  unsigned short time; /* $BFbIt%+%&%s%?(B (1ms$B<~4|(B)$B$NCM(B */
  unsigned short magicno; /* EEPROM $BFb$N?t;z(B */
  unsigned short ad[8];/* A/D $B%3%s%P!<%?$NCM(B(10bit) */
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
