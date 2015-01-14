#ifndef LRF_HEAD
#define LRF_HEAD
#define FIRST_LRF_RANGE -45
#define LAST_LRF_RANGE 225
#define NUM_OF_LRF 3
#define MAXRANGE 5000
#define SIDERANGE 650
#define SIDELIMITRANGE 500
#define LIMITMAXRANGE 1200
#define COSINETHRESHOLD 1500
#define LRF_FRONT_TOP 0
#define LRF_FRONT_BOTTOM 1
#define LRF_REAR 2
#define BOTTOM_LRF_THRES_CNT 60
#define BOTTOM_LRF_THRES_LONG 90
#define BOTTOM_LRF_SIDE_THRES 500
#define REAR_SIDERANGE 500
#define REAR_DANGERRANGE_SHORT 300
#define REAR_DANGERRANGE_LONG 3000
#define REAR_LRF_THRES_CNT 400
#define REAR_LRF_THRES_LONG 10
#define REAR_LRF_THRES_PERSON_LONG 150
#define REAR_LRF_THRES_PERSON_CNT 20

extern const char* lrf_device[];
const long connect_baudrate = 115200;
const char connect_address[] = "192.168.0.20";
const long connect_port = 10940;

typedef struct{
  int cnt;
  int first_step;
  int last_step;
  double first_range;
  double last_range;
  double angle;
  double angleABS;
}free_area;
// prototype
double obstacle_avoidance(long* lrfdata, CvPoint2D32f* lrfdataxy, int lrf_data_max, double target_angle, int maxrange);
double obstacle_avoidance2(long* lrfdata, CvPoint2D32f* lrfdataxy, int lrf_data_max, double target_angle, int maxrange, double nowangle);
int check_front_obstacle(long* lrfdata, CvPoint2D32f* lrfdataxy, int lrf_data_max); /* 0 is clear, 1 is aru. */
int check_front_obstacle2(long* lrfdata, CvPoint2D32f* lrfdataxy, int lrf_data_max); /* 0 is clear, 1 is aru. */
int check_back_obstacle(long* lrfdata, CvPoint2D32f* lrfdataxy, int lrf_data_max);
int check_back_new(long* lrfdata, CvPoint2D32f* lrfdataxy, int lrf_data_max);
#endif
