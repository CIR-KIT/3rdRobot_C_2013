PACKAGE	= USB_SENSOR
SRCS	= h8test.c uread.c uwrite.c
HEADS	= usc.h usensorc.h
OBJS	= $(SRCS:.c=.o)

FILES	= Makefile $(HEADS) $(SRCS)
VER	= `date +%Y%m%d`

TARGET=h8test uwrite uread
CC = gcc
KCFLAGS= -D__KERNEL__ -I$(INCLUDE) -Wall -Wstrict-prototypes -O \
         -pipe -DMODULE

CFLAGS= -O -static -Wall

INCLUDE= /usr/src/linux/include

# etc
SHELL	= /bin/sh
RM	= rm -f
PROF	= gprof

all: $(TARGET)

tar:
	@echo $(PACKAGE)-$(VER) > .package
	@$(RM) -r `cat .package`
	@mkdir `cat .package`
	@ln $(FILES) `cat .package`
	tar cvf - `cat .package` | gzip -9 > `cat .package`.tar.gz
	@$(RM) -r `cat .package` .package

clean:
	rm -f $(TARGET) core *~ \#*\# *.o
