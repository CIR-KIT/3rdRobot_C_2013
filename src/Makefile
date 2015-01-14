#==============================================================================
#  Date   : 2013/09/09 ver.1.0
#           2013/10/16 ver.1.5
#  Author : Ryodo Tanaka
#  ChangeLog: (emacs)C-x 4 a
#  
#==============================================================================

#マクロ定義
CC 		= g++
LRF_INC 	= -I/usr/local/include/urg_c
LRF_LIB 	= -lurg_c
OPENCV_INC = `pkg-config --cflags opencv`
OPENCV_LIB = `pkg-config --libs opencv`

CFLAGS		= $(OPENCV_INC) $(LRF_INC)
CPPFLAGS	= $(CFLAGS)
LDLIBS		= $(OPENCV_LIB) $(LRF_LIB) -lm -pthread
.PHONY : clean

#===============================================================================
#  何かソースを更新したら...
#
#  1. VPATH にそのソースの置いてあるディレクトリの相対パスを追加する。
#     追加するときは : をつけて追加すること。
#
#  2. SUB_SRC に追加したソースの.cファイルを追加する。
#     追加するときはスペースを開けて追加すること。
#  
#  以上が、ソースを追加した時の使い方である。
#  デバッグをしたい時は DEBUG のコメントアウトをとる。
#  オプションを付けたい時は OPTION のコメントアウトをとる。
#  どちらも何か追加したければ、スペースを開けて追加すること。
#===============================================================================
PROG		= robot

OPTION		= -Wall
DEBUG		= -g -O0 

OBJDIR = ./bin/

VPATH		= $(OBJDIR):./tmp:./GPS:./InputCheck:./Encoder:./LRF:./iMCs_series:./Motor:./TCP_IP/Server

MAIN_SRC	= robot.c
SUB_SRC		= gps.c lrf.c inputcheck.c encoder.c imcs_set.c motor_cntl.c server.c

SRCS		= $(MAIN_SRC) $(SUB_SRC)
# TAGS: $(SRCS)
# 	etags *.[ch]
OBJS		= $(SRCS:%.c=%.o)
HEADS		= $(SRCS:%.c=%.h) $(SUB_SRC:%.c=_%.h)

all: $(PROG) $(%.o) $(PROG).o

%.o : %.c %.h _%.h
	$(CC) -c $(OPTION) $(DEBUG) $< $(CFLAGS) $(LDLIBS) -o $(OBJDIR)$@ 

$(PROG).o : $(PROG).c $(PROG).h lrf.h
	$(CC) -c $(OPTION) $(DEBUG) $< $(CFLAGS) $(LDLIBS) -o $(OBJDIR)$(PROG).o

$(PROG) : $(OBJS) $(HEADS)
	$(CC) -o $@ $(OPTION) $(DEBUG) $(OBJS:%.o=$(OBJDIR)%.o) $(CFLAGS) $(LDLIBS)


clean : 
	rm $(OBJS:%.o=$(OBJDIR)%.o)
