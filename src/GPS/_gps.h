#if !defined(_GPS_)
#define _GPS_
///////////////////////////////////////////////////////////////
// ************** GPS heada file *****************************
///////////////////////////////////////////////////////////////
/*===========
   include
============*/
// Standard
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <assert.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <errno.h>
#include <sys/time.h>
// OpenCV 2.3.1
/* #include <iostream> */
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#define CIRCLE_RADIUS 2
#define TARGET_DOMAIN 0.000003
#define HDOP 4 
#define LA_0 36.0
#define LO_0 139.0
#define PI 3.1415926535

int GPS_arg = 0;
int CharCount=0;               	//GPSからの読み込み文字数
int receive_flag=0;		//受信確認フラグ
int convert_flag[2];
int GPSoutput=0;               //ファイル出力フラグ
int NofGPSdata=0;              //GPSデータ数
int pointx=300, pointy=200;

char T[10];  //生データ時間
char S;      //生データ有効無効フラグ
char La[12]; //緯度 2文字が[度] 後の部分の単位は[分]
char Lo[12];//経度 3文字が[度] 後の部分の単位は[分]
char Deg[12];
char Vel[12];
char henkaku[12];
char henni[12];
char N[10];
//char V[10];
//char E[10];
//char UTC[12];
char HDop[10];

double dis_point;

#endif
