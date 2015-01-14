#define NUM_OF_TCP 3

typedef struct{
  int use;            //use target_human_finder or not
  int area_number;   //area number
  int plate;          //find plate flag
  int human;          //find human flag
  int  num_of_plate;   //number of found plate
  int  num_of_human;   //number of found human
  int* plate_dir;
  int* human_dir;
} thf_cmd;

typedef struct{
  long lrf_data_max;
  long lrf_data[1081];
} lrf_cmd;

extern int serv_sock[NUM_OF_TCP];
extern int clnt_sock[NUM_OF_TCP];
extern struct sockaddr_in ClntAddr[NUM_OF_TCP];


void tcp_error(const char*);
void tcp_retry(const char*);
int server_setup(void);
int lrf_cmd2char(char* lrf_cmd_str, const lrf_cmd lrf_cmd_data);

