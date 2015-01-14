/*===========
   include
============*/
// Standard
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>
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
#include <sys/socket.h>	/* socket()、connect()、send()、recv()に必要 */
#include <arpa/inet.h>	/* sockaddr_in、inet_addr()に必要 */
// OpenCV 2.3.1
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
// LRF
#include "urg_sensor.h"
#include "urg_utils.h"
//GPS
#include "./GPS/gps.h"
//input check
#include "./InputCheck/inputcheck.h"
//LRF
#include "./LRF/lrf.h"
//iMCs03
#include "../imcs03_driver/driver/usc.h"
#include "../imcs03_driver/driver/usensorc.h"
//iMCs01
#include "../imcs01_driver/driver/urbtc.h"
#include "../imcs01_driver/driver/urobotc.h"
//iMCs set
#include "./iMCs_series/imcs_set.h"
//Encoder
#include "./Encoder/encoder.h"
//Motor
#include "./Motor/motor_cntl.h"
//TCP/IP
#include "./TCP_IP/Server/server.h"
//mutex
static pthread_mutex_t gps_data_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t now_gps_data_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t lrf_data_mtx[3] = {PTHREAD_MUTEX_INITIALIZER,
					  PTHREAD_MUTEX_INITIALIZER,
					  PTHREAD_MUTEX_INITIALIZER};
static pthread_mutex_t input_angle_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t desire_angle_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t rd_cnt_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t pthread_flag_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t imcs01_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t motionflag_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t rear_enc_mtx = PTHREAD_MUTEX_INITIALIZER;
//STRUCT
typedef struct 
{
  bool lrf;
  bool gps;
  bool sequence;
  bool odometory;
}Pthread_Flag;

#define ENCMODE 1
#define GPSMODE 2

/* Grobal variables */
int quit_flag = 1;
Pthread_Flag pthread_flag;
double desire_angle = 0;
int  gps_log_num = 0;
int motionflag = 0;
int nextTP = 0;

/*-- GPS --*/
char gps_buf[256] = {'\0'}; //分割元
GPS_Data TRGdata[150];
GPS_Data NowGdata;
CvPoint2D32f WORLD_0 = {0, 0};
double* value;

/*-- LRF --*/
urg_t urg[NUM_OF_LRF];
int lrf_data_max[NUM_OF_LRF];
int lrf_data_max_cpy[NUM_OF_LRF];
long** lrf_data;
CvPoint2D32f** lrf_data_xy;
long** lrf_data_cpy;
CvPoint2D32f** lrf_data_xy_cpy;
const char* lrf_device[] = {"/dev/ttyACM0","/dev/ttyACM1","/dev/ttyACM2"};

/*-- Odometory --*/
double RD_cnt = 67;
double rear_enc = 0;
double integral_enc_meter = 0;
GPS_Data odo_state;
const double machineL = 0.96;//0.96;
double velocity = 0;
double w_k = 0;

/*-- SP motor --*/
int input_angle = 0;

/*-- imcs03 --*/
const char *imcs03 = "/dev/usc0";
int fd_imcs03 = 0;
struct uddr cmd03_uddr;
struct udr cmd03_udr;
struct udata buf03;

/*-- imcs01 --*/
const char *imcs01 = "/dev/urbtc1";
struct uin buf01;
int fd_imcs01 = 0;
struct uout cmd01_uout;
struct ccmd cmd01_ccmd;

/*-- TCP/IP --*/
int serv_sock[NUM_OF_TCP];
int clnt_sock[NUM_OF_TCP];
struct sockaddr_in ClntAddr[NUM_OF_TCP];
lrf_cmd lrf_cmd_data;