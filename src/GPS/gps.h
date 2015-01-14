#if !defined(_GPS)
#define _GPS

#define RESET_DIST 4

#define PI 3.1415926535
#define GPS_DEVICE "/dev/ttyUSB0"
#define GPS_ARRAY_NUM 25
#define GPGGATIME 1
#define GPGGALA 2
#define GPGGALO 4
#define GPGGAHDOP 8
#define GPGGASATELITES 7
#define GPGGAQUALITY 6
#define MOVINGAVERAGENUM 3

typedef struct 
{
  double time;		//測定時間
  int status;	        //有効確認フラグ
  double la;		//緯度 latitude
  double lo;		//経度 longitude
  CvPoint2D32f pt;     //メートルに変換したもの
  double hdop; // HDOP = 0.0 to 9.9 Smaller and Smaller is good.
  int sate; // Number of satelites used in position computation.
  int qua; //qualitry indicater, 0 = no position, 1 = gps only, 2 = differentially or 9 = position computed using almanac.
  double r;
  double dir;
  int nowpoint;
}GPS_Data;

const char GPS_LOG_DATA[] = "../resources/GPS_Log131117.csv";
const char delimiter[] = ",";

int SplitString(char *buf[], const char *str, const char *delimiter);
int set_GTargetGPS(CvPoint2D32f *zero, GPS_Data *trgdata, const char *gps_log_data);
double vlum_convertK(char* row);
double calc_around_pi(double radian);
CvPoint2D32f GPS2m(double lad, double lod, CvPoint2D32f* pt_0);
double kit_map(int target_point, const GPS_Data *target_data, GPS_Data now_data);
int ChangeTargetPoint(const GPS_Data *target_point, GPS_Data now, int nextTP, int NUMofTP);
//double MovingDirectionAverage(GPS_Data now_data);
double MovingDirectionAverage(GPS_Data now_data, double* value, int loop_num);
double average_angle(double data[],int size);
int ChangeTargetPoint2(const GPS_Data *target_point, GPS_Data now, int nextTP, int NUMofTP);
#endif //_GPS
