#include "robot.h"
#define GPS
#define SEQUENCE		/* This depend on GPS. */
#define LRF
#define ETHERLRF
#define SPMOTOR  /* This depend on ODOMETORY and IMCS03. */
#define ODOMETORY  /* This depend on IMCS01. */
#define SERVER
#define IMCS03
#define IMCS01
#define debug
#define FORWARD 1
#define BACK -1
#define STOP 0

static void
handler(int sig)
{
  int i = 0;
  fprintf(stderr, "\nKill signal received\n");
  printf("enc_meter = %lf\n",integral_enc_meter);	/* using iMCs01 CN107, rear */
#ifdef IMCS03
  close(fd_imcs03);
#endif
#ifdef IMCS01
  stop(fd_imcs01, &cmd01_ccmd);	/* motor stop */
  close(fd_imcs01);
#endif
#ifdef LRF
  for(i=0; i<NUM_OF_LRF; i++){
    urg_close(&urg[i]);
  }
#endif
  _exit(1);
}
/*++++++++++++++++++++++++++++++++++++++++++++++*/
/*    GPSthread                                 */
/*++++++++++++++++++++++++++++++++++++++++++++++*/
static void *
GPSthread(void *arg)
{
  int gps_fd = 0;
  struct termios newtio;
  //GPS open
  gps_fd = open(GPS_DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
  fcntl(gps_fd,F_SETFL,0);
  printf("gps_start : %d\n",gps_fd);
  if(gps_fd < 0){
    fprintf(stderr,"GPS open error\n");
    exit(-1);
  }
  //Setting
  memset(&newtio, 0, sizeof(newtio));
  newtio.c_cflag     = CS8 | CLOCAL | CREAD | CRTSCTS;
  newtio.c_cc[VTIME] = 0;
  newtio.c_lflag     = ICANON;
  newtio.c_iflag     = IGNPAR | ICRNL;
  newtio.c_oflag     = 0; 
  cfsetispeed(&newtio, B19200);    //受信ボーレイト設定  B19200
  cfsetospeed(&newtio, B19200);    //送信ボーレイト設定
  tcsetattr(gps_fd, TCSANOW, &newtio);
  pthread_mutex_lock(&pthread_flag_mtx);
  pthread_flag.gps = true;
  pthread_mutex_unlock(&pthread_flag_mtx); 
  //Get start
  while(1){
    usleep(10000);
    pthread_mutex_lock(&gps_data_mtx);
    read(gps_fd, gps_buf, sizeof(gps_buf));
    pthread_mutex_unlock(&gps_data_mtx);
  }
  return NULL;
}

/*++++++++++++++++++++++++++++++++++++++++++++++*/
/*    Sequence thread                           */
/*++++++++++++++++++++++++++++++++++++++++++++++*/
static void *
Sequencethread(void *arg)
{
  /* Wait for GPS thread */
  while(1){
    pthread_mutex_lock(&pthread_flag_mtx);
    if(pthread_flag.gps == true){
      pthread_mutex_unlock(&pthread_flag_mtx);
      break;
    }else{
      pthread_mutex_unlock(&pthread_flag_mtx);
    }
  }
  pthread_mutex_lock(&pthread_flag_mtx);
  pthread_flag.sequence = true;
  pthread_mutex_unlock(&pthread_flag_mtx);
  /* sequence main loop */
  while(1){
    pthread_mutex_lock(&now_gps_data_mtx);
    pthread_mutex_lock(&desire_angle_mtx);
    nextTP = ChangeTargetPoint(TRGdata, NowGdata, nextTP, gps_log_num);
    /* printf("nextTP = %d\n", nextTP); */
    NowGdata.nowpoint = nextTP -1;
    if(NowGdata.qua == 0){	/* if GPS cannot catch satellite */
      desire_angle = 0;
    }else{
      desire_angle = kit_map(nextTP, TRGdata, NowGdata);
    }
    pthread_mutex_unlock(&desire_angle_mtx);
    pthread_mutex_unlock(&now_gps_data_mtx);
    usleep(5000);
  }
  return NULL;
}

/*++++++++++++++++++++++++++++++++++++++++++++++*/
/*    SplitGPSDatathread                        */
/*++++++++++++++++++++++++++++++++++++++++++++++*/
static void *
SplitGPSDatathread(void *arg)
{
  int ret, i = 0;
  char *raw_gps_data[GPS_ARRAY_NUM];
  char *last_raw_gps_data[GPS_ARRAY_NUM];
  GPS_Data LastGData;
  GPS_Data tempData;
  GPS_Data odo_state_cpy;
  double value[MOVINGAVERAGENUM];
  double velo_cpy = 0;
  int loop_num = 0;
  int mode = 0;
  /* get memory */
  for(i = 0; i < GPS_ARRAY_NUM; i++){
    raw_gps_data[i] = (char *)malloc(256);
    last_raw_gps_data[i] = (char *)malloc(256);
  }
  /* initialize */
  for(i=0; i<MOVINGAVERAGENUM; i++){
    value[i] = 0;
  }
  LastGData.time = 0;
  tempData.pt.x = 0;
  tempData.pt.y = 0;
  /* Wait for GPS thread */
  while(1){
    pthread_mutex_lock(&pthread_flag_mtx);
    if(pthread_flag.gps == true){
      pthread_mutex_unlock(&pthread_flag_mtx);
      break;
    }else{ 
      pthread_mutex_unlock(&pthread_flag_mtx);
    }
  }
  /* main loop */
  while(1){
    pthread_mutex_lock(&gps_data_mtx);
    ret = SplitString(raw_gps_data, gps_buf, delimiter); /* Copy gps_buf to raw_gps_data. */
    pthread_mutex_unlock(&gps_data_mtx);

    pthread_mutex_lock(&rear_enc_mtx);
    odo_state_cpy = odo_state;
    velo_cpy = velocity;
    pthread_mutex_unlock(&rear_enc_mtx);
    
    if(ret < 14){		/* GPGGA mode will return 15 data element */
      /* pthread_mutex_lock(&now_gps_data_mtx); */
      /* printf("ret == %d\n",ret); */
      //NowGdata = odo_state_cpy;
      /* NowGdata.dir = calc_around_pi(MovingDirectionAverage(NowGdata, value, loop_num)); */
      /* pthread_mutex_unlock(&now_gps_data_mtx); */
      /* mode = ENCMODE; */
      continue;
    }else{
      pthread_mutex_lock(&now_gps_data_mtx);
      NowGdata.time = atof(raw_gps_data[GPGGATIME]);
      if(NowGdata.time != LastGData.time){
	NowGdata.time = atof(raw_gps_data[GPGGATIME]);
	NowGdata.la   = vlum_convertK(raw_gps_data[GPGGALA]); /* latitude(ex. 33.0) */
	NowGdata.lo   = vlum_convertK(raw_gps_data[GPGGALO]); /* longitude(ex. 134.0) */
	NowGdata.hdop = atof(raw_gps_data[GPGGAHDOP]);	    /* HDOP */
	NowGdata.sate = atoi(raw_gps_data[GPGGASATELITES]); /* num of satelite */
	NowGdata.qua  = atoi(raw_gps_data[GPGGAQUALITY]);	  /* return 0,1,2 or 9 */
	NowGdata.pt   = GPS2m(NowGdata.la, NowGdata.lo, &WORLD_0);	/*from la lo to meter. */
	if(NowGdata.hdop < 5){				       /* GPS could recieve data. */
	  if(velo_cpy != 0){ /* robot moved (on program) */
	    if(NowGdata.hdop <= 2.5){
	      pthread_mutex_lock(&rear_enc_mtx);
	      NowGdata.dir = atan2(NowGdata.pt.y - LastGData.pt.y, NowGdata.pt.x - LastGData.pt.x);
	      LastGData = NowGdata;
	      odo_state = NowGdata;
	      pthread_mutex_unlock(&rear_enc_mtx);
	    }else{
	      tempData.pt.x = (odo_state_cpy.pt.x + NowGdata.pt.x*(6-NowGdata.hdop))/(1+6-NowGdata.hdop);
	      tempData.pt.y = (odo_state_cpy.pt.y + NowGdata.pt.y*(6-NowGdata.hdop))/(1+6-NowGdata.hdop);
	      NowGdata.pt = tempData.pt;
	      NowGdata.dir = atan2(NowGdata.pt.y - LastGData.pt.y, NowGdata.pt.x - LastGData.pt.x);
	      LastGData = NowGdata;
	      value[0] = odo_state_cpy.dir;
	      value[1] = odo_state_cpy.dir;
	      value[2] = NowGdata.dir;
	      NowGdata.dir = average_angle(value, MOVINGAVERAGENUM);
	      /* NowGdata.dir = MovingDirectionAverage(NowGdata, value, loop_num); */
	      pthread_mutex_lock(&rear_enc_mtx);
	      odo_state = NowGdata;
	      pthread_mutex_unlock(&rear_enc_mtx);
	    }
	  }else{
	    if(loop_num == 0){
	      LastGData = NowGdata;
	      loop_num++;
	    }else{
	      NowGdata = LastGData;
	    }
	  }
	}else{
	  NowGdata = odo_state_cpy;
	  /* NowGdata.dir = MovingDirectionAverage(NowGdata, value, loop_num); */
	  /* NowGdata.dir = calc_around_pi(NowGdata.dir); */
	  /* NowGdata.dir =MovingDirectionAverage(NowGdata, value, loop_num); */
	  LastGData = NowGdata;
	  mode = ENCMODE;
	}
      }else{			/* could not recieved. */
	NowGdata = odo_state_cpy;
       	/* NowGdata.dir = MovingDirectionAverage(NowGdata, value, loop_num); */
      	/* NowGdata.dir = calc_around_pi(NowGdata.dir); */
	LastGData = NowGdata;
	mode = ENCMODE;
      }
      pthread_mutex_unlock(&now_gps_data_mtx);
    }
    for(i=0; i<300; i++){
      usleep(1000);
    }
    /* sleep(1); */
  }
  for(i = 0; i < GPS_ARRAY_NUM; i++){
    free(raw_gps_data[i]);
    free(last_raw_gps_data[i]);
  }
  return NULL;
}

/*++++++++++++++++++++++++++++++++++++++++++++++*/
/*    LRFthread                                 */
/*++++++++++++++++++++++++++++++++++++++++++++++*/
static void *
LRFthread(void *arg)
{
  double radian = 0;
  int ret = 0;
  int i = 0;
  int j = 0;
  int k = 0;
  long templrfdata[1080] = {0};
  size_t size;
  /* Set LRF connection. */
  ret = urg_open(&urg[LRF_FRONT_TOP], URG_SERIAL, lrf_device[LRF_FRONT_TOP], connect_baudrate);
  if(ret < 0){
    printf("URG_SERIAL[LRF_FRONT_TOP] open error\n");
    exit(1);
  }

  ret = urg_open(&urg[LRF_FRONT_BOTTOM], URG_SERIAL, lrf_device[LRF_FRONT_BOTTOM], connect_baudrate);
  if(ret < 0){
    printf("URG_SERIAL[LRF_FRONT_BOTTOM] open error\n");
    exit(1);
  }
  ret = urg_open(&urg[LRF_REAR], URG_SERIAL, lrf_device[LRF_REAR], connect_baudrate);
  if(ret < 0){
    printf("URG_SERIAL[LRF_FRONT_BOTTOM] open error\n");
    exit(1);
  }

#ifdef ETHERLRF
  /* ret = urg_open(&urg[LRF_REAR], URG_ETHERNET, connect_address, connect_port); */
  /* if(ret < 0){ */
  /*   printf("URG_ETHERNET open error\n"); */
  /*   exit(-1); */
  /* } */

#endif
  /* Get memory for lrf data. */
  size = NUM_OF_LRF*(sizeof(long));
  lrf_data = (long **)alloca(size);
  lrf_data_cpy = (long **)alloca(size);
  lrf_data_xy = (CvPoint2D32f **)alloca(size);
  lrf_data_xy_cpy = (CvPoint2D32f **)alloca(size);
  for(i = 0; i < NUM_OF_LRF; i++){
    lrf_data_max[i] = urg_max_data_size(&urg[i]);
    size = lrf_data_max[i]*(sizeof(long));
    lrf_data[i] = (long *)alloca(size);
    lrf_data_cpy[i] = (long *)alloca(size);
    lrf_data_xy[i] = (CvPoint2D32f *)alloca(size);
    lrf_data_xy_cpy[i] = (CvPoint2D32f *)alloca(size);
  }

  pthread_mutex_lock(&pthread_flag_mtx);
  pthread_flag.lrf = true; 	/* Ready OK */
  pthread_mutex_unlock(&pthread_flag_mtx);
  /* Main loop start. */
  for(;;){
    for(i = 0; i < NUM_OF_LRF; i++){
      ret = urg_start_measurement(&urg[i], URG_DISTANCE, 1, 0);
      pthread_mutex_lock(&lrf_data_mtx[i]);
      lrf_data_max[i] = urg_get_distance(&urg[i], lrf_data[i], NULL);
      /* for(j=0;j<lrf_data_max[i];j++){ */
      /* 	templrfdata[j] = lrf_data[i][j]; */
      /* } */
      /* lrf_data_max[i] = urg_get_distance(&urg[i], lrf_data[i], NULL); */
      /* for(j=0;j<lrf_data_max[i];j++){ */
      /* 	templrfdata[j] &= lrf_data[i][j]; */
      /* } */
      /* lrf_data_max[i] = urg_get_distance(&urg[i], lrf_data[i], NULL); */
      /* for(j=0;j<lrf_data_max[i];j++){ */
      /* 	templrfdata[j] &= lrf_data[i][j]; */
      /* } */
      /* for(j=0;j<lrf_data_max[i];j++){ */
      /* 	lrf_data[i][j] = templrfdata[j]; */
      /* } */

      for(j = 0; j < lrf_data_max[i]; j++){
	radian = urg_index2rad(&urg[i], j);
	lrf_data_xy[i][j].x = lrf_data[i][j]*cos(radian);
	lrf_data_xy[i][j].y = lrf_data[i][j]*sin(radian);
      }
      pthread_mutex_unlock(&lrf_data_mtx[i]);
    }
  }
  exit(1);
  return NULL;
}

#ifdef SERVER
static void *
Serverthread(void *arg)
{
  static  int i=0,z=0;
  lrf_cmd lrf_cmd_data;
  char lrf_cmd_str[5000];
  char recv_flg;
  unsigned int Client_Bytes = 0;
  int Send_Bytes = 0;

  server_setup();

  for(z=0; z<3; z++){
    Client_Bytes = sizeof(ClntAddr);
    if( (clnt_sock[z] = accept(serv_sock[z], (struct sockaddr*)&ClntAddr, &Client_Bytes)) < 0 )
      tcp_error("accept()");
    puts("Accepted");
  } 

  while(1){

    for(z=0; z<3; z++){
      //  printf("start : %d\n", z);
      pthread_mutex_lock(&lrf_data_mtx[z]);
      lrf_cmd_data.lrf_data_max = lrf_data_max[z];
      for(i=0; i<1081; i++){
	lrf_cmd_data.lrf_data[i] = lrf_data[z][i];
      }
      pthread_mutex_unlock(&lrf_data_mtx[z]);
     
      lrf_cmd2char(lrf_cmd_str, lrf_cmd_data);
     
      for(i=0; i<5000; i+=Send_Bytes){
	//printf("core : %d\n", z);
	while(1){
	  if((Send_Bytes = send(clnt_sock[z], &lrf_cmd_str[i], 250, 0)) != 250)
	    tcp_retry("send()"); 
	  else break;
	}
      }

      while(1){
	if(recv(clnt_sock[z], &recv_flg, sizeof(char), 0) < 0)
	  tcp_retry("recv()");
	else if(recv_flg == '1'){
	  recv_flg == '0';  
	  break;
	}
	else;
      }
      // printf("end : %d\n", z);    
    }
  }
  return NULL;
}
#endif
  /*++++++++++++++++++++++++++++++++++++++++++++++*/
  /*    Front Stepping motor thread               */
  /*++++++++++++++++++++++++++++++++++++++++++++++*/
#ifdef SPMOTOR
  static void *
    FrontSPmotorthread(void *arg)
  {
    printf("SP motor start\n");
    int i = 0, j = 0; 
    int output_angle = 0;
    int temp_angle = 0;
    const int cw[4]  = {0x2, 0x0, 0x1, 0x0}; /* using iMCs03 CN101. */
    const int ccw[4] = {0x8, 0x0, 0x4, 0x0}; /* using iMCs03 CN101. */

    for(;;){
      usleep(100); /* delay loop speed. */
      pthread_mutex_lock(&input_angle_mtx);
      temp_angle = input_angle;
      pthread_mutex_unlock(&input_angle_mtx);
      if(temp_angle >= 60){
	temp_angle = 60; /* if over 50[deg], change to 50[deg]. */
      }else if(temp_angle <= -60){
	temp_angle = -60;	/* if under -50[deg], change to -50[deg]. */
      }else{
	;
      }
      temp_angle = temp_angle * 20; /* 1 pulse is 0.05[deg]. */
      pthread_mutex_lock(&rd_cnt_mtx);
      output_angle = temp_angle - (int)RD_cnt*20; /* RD_cnt[deg] is Global. */
      pthread_mutex_unlock(&rd_cnt_mtx);
      if(output_angle < 0){			       /* left rotation */
	output_angle = (int)output_angle * (-1) * 5 / 4; /* - to + */
	for(i = 0; i < output_angle; i++){ /* Please read iMCs03 and Motor driver paper. */
	  if(i%4 == 3){			 /* check input angle. */
	    pthread_mutex_lock(&input_angle_mtx);
	    if(temp_angle != input_angle){
	      pthread_mutex_unlock(&input_angle_mtx);
	      break;
	    }
	    pthread_mutex_unlock(&input_angle_mtx);
	  }
	  for(j = 0; j < 4; j++){
	    cmd03_udr.PBDR = cw[j];
	    if (write(fd_imcs03, &cmd03_udr, sizeof(cmd03_udr)) < 0)
	      fprintf(stderr, "write error __High__\n");
	    usleep(1);
	  }
	}
      }
      else{				   /* right rotation */
	output_angle = output_angle * 5 / 4; /* 5/4 is Ratio of Gear. */
	for(i = 0; i < output_angle; i++){
	  if(i%4 == 3){
	    pthread_mutex_lock(&input_angle_mtx);
	    if(temp_angle != input_angle){
	      pthread_mutex_unlock(&input_angle_mtx);
	      break;
	    }
	    pthread_mutex_unlock(&input_angle_mtx);
	  }
	  for(j = 0; j < 4; j++){
	    cmd03_udr.PBDR = ccw[j];
	    if (write(fd_imcs03, &cmd03_udr, sizeof(cmd03_udr)) < 0)
	      fprintf(stderr, "write error __High__\n");
	    usleep(1);
	  }
	}
      }
    }
    return NULL;
  }
#endif

  /*++++++++++++++++++++++++++++++++++++++++++++++*/
  /*    Odometor thread                           */
  /*++++++++++++++++++++++++++++++++++++++++++++++*/
#ifdef ODOMETORY
  static void *
    Odometorythread(void *arg)
  {
    double rear_enc_temp = 0;
    double last_rear_enc_count = 0;
    double integral_enc = 0;
    double last_integral_enc_meter = 0;
    double time = 0;
    /* double w_k = 0; */
    double temp_rd_cnt = 0;
    double d_dis = 0;
    double last_time = 0;
    /* GPS_Data odo_state_last; */
    /* int odo_loop_num = 0; */
    int pp = 0;
    int i = 0;
    /* struct timeval s, e; */
#ifdef GPS
    while(1){
      /* sequence flag depend on gps flag. */
      pthread_mutex_lock(&pthread_flag_mtx);
      pp = flag_check(pthread_flag.sequence);
      if(pp > 0){
	pthread_mutex_unlock(&pthread_flag_mtx);
	break;
      }else{
	pthread_mutex_unlock(&pthread_flag_mtx);
      }
    }
#endif

    pthread_mutex_lock(&pthread_flag_mtx);
    pthread_flag.odometory = true;
    pthread_mutex_unlock(&pthread_flag_mtx);

    while(1){
      pthread_mutex_lock(&imcs01_mtx);
      if(read(fd_imcs01, &buf01, sizeof(buf01)) != sizeof(buf01)){ /* read from iMCs01. */
	pthread_mutex_unlock(&imcs01_mtx);   
	fprintf(stderr, "Warning: read size mismatch");
	continue;
      }else{
	pthread_mutex_unlock(&imcs01_mtx);
	pthread_mutex_lock(&rd_cnt_mtx);
	RD_cnt = (-1)*buf01.ct[1]*27/1453; /* using iMCs01 CN106, front , output is [deg]*/
	temp_rd_cnt = (-RD_cnt)*(1+(fabs(RD_cnt)/220));
	pthread_mutex_unlock(&rd_cnt_mtx);
	rear_enc_temp = buf01.ct[2];	/* using iMCs01 CN107, rear */
	if(fabs(rear_enc_temp - last_rear_enc_count) >= 30000){ /* iMCs01 counter overflow */
	  if(rear_enc_temp < 0 && last_rear_enc_count > 0){ /* forward */
	    integral_enc += (65635 +(rear_enc_temp - last_rear_enc_count));
	    last_rear_enc_count = rear_enc_temp;
	  }else if(rear_enc_temp > 0 && last_rear_enc_count < 0){ /* back */
	    integral_enc += (65635 - (rear_enc_temp - last_rear_enc_count));
	    last_rear_enc_count = rear_enc_temp;
	  }else{
	    printf("maybe arienai\n");
	    continue;
	    /* exit(1); */
	  }
	}else{			/* do not overflow */
	  integral_enc += (rear_enc_temp - last_rear_enc_count);
	  last_rear_enc_count = rear_enc_temp;
	}
	time = (buf01.time - last_time)/1000; /* [ms] to [s] */
	if(time < 0){
	  time = time + 65536;
	}
	last_time = buf01.time;
	integral_enc_meter = (integral_enc/4*26/20)*(0.06005*PI/360); /* pulse to meter. */
	/* odo_loop_num++; */
	/* if(odo_loop_num == 1){ */
	/* 	gettimeofday(&s, NULL); */
	/* } */
	/* if(odo_loop_num == 10){ */
	/* pthread_mutex_lock(&now_gps_data_mtx); */
	/* gettimeofday(&e, NULL); */
	/* time = (e.tv_sec - s.tv_sec)+(e.tv_usec - s.tv_usec)*1.0E-6;/\* 0.001*0.001; *\/ */
	d_dis = integral_enc_meter - last_integral_enc_meter;
	pthread_mutex_lock(&rear_enc_mtx);
	velocity = d_dis/time;
	/* printf("time :\t%lf,\td_dis :\t%lf,\tvelocity :\t%lf\n",time,d_dis,velocity); */
	w_k = (velocity/machineL)*tan(temp_rd_cnt/180*PI);
	odo_state.dir = odo_state.dir + w_k * time;
	odo_state.dir = calc_around_pi(odo_state.dir);
	if(w_k != 0){
	  odo_state.pt.x = odo_state.pt.x+(velocity/w_k)*(sin(odo_state.dir+w_k*time)-sin(odo_state.dir));
	  odo_state.pt.y = odo_state.pt.y+(-1)*(velocity/w_k)*(cos(odo_state.dir+w_k*time)-cos(odo_state.dir));
	}else{
	  if(velocity != 0){
	    odo_state.pt.x = odo_state.pt.x + d_dis*(cos(odo_state.dir));
	    odo_state.pt.y = odo_state.pt.y + d_dis*(sin(odo_state.dir));
	  }else{
	    /* odo_state.pt.x; */
	    /* odo_state.pt.y; */
	  }
	  /* } */
	  /* pthread_mutex_unlock(&now_gps_data_mtx); */
	  /* odo_loop_num = 0; */
	}
	last_integral_enc_meter = integral_enc_meter;
	pthread_mutex_unlock(&rear_enc_mtx);
	for(i=0;i<2;i++){
	  usleep(1000);
	}
      }
    }
    return NULL;
  }
#endif


  /*++++++++++++++++++++++++++++++++++++++++++++++*/
  /*    DisplayIndicatethread                     */
  /*++++++++++++++++++++++++++++++++++++++++++++++*/
#ifdef debug
  static void *
    DisplayIndicatethread(void *arg)
  {
    int loop = 0;
    while(1){
      loop++;
      /* pthread_mutex_lock(&now_gps_data_mtx); */
      /* pthread_mutex_lock(&rear_enc_mtx); */
      printf("\033[2J");    //画面クリア
      printf("loopnum %d\n",loop);
      printf("[main]\nTP == %d\nnextTP:\t\tLa == %f, Lo == %f\n",
	     NowGdata.nowpoint+1,TRGdata[NowGdata.nowpoint+1].pt.x,TRGdata[NowGdata.nowpoint+1].pt.y);
      printf("gps_state:\t time == %lf, La == %f, Lo == %f, QUA == %d, HDOP == %lf\n",NowGdata.time,NowGdata.la ,NowGdata.lo,NowGdata.qua,NowGdata.hdop);
      printf("odo_state:\t time == %lf, La == %f, Lo == %f, QUA == %d, HDOP == %lf\n",odo_state.time,odo_state.la ,odo_state.lo,odo_state.qua,odo_state.hdop);
      printf("desire_angle\t%lf\n",desire_angle*180/PI);
      printf("calc_pi\t%lf\n",calc_around_pi(NowGdata.dir)*180/PI);
      printf("gps_state:\t x:%f\ty:%f\tdir:%lf\n",NowGdata.pt.x ,NowGdata.pt.y, NowGdata.dir*180/PI);    
      printf("odo_state:\t x:%lf\ty:%lf\tdir:%lf\n",odo_state.pt.x,odo_state.pt.y,fmod(odo_state.dir*180/PI,360));
      //   printf("velocity == %lf\t[km/h]\n",velocity*18/5);
      /* pthread_mutex_unlock(&rear_enc_mtx); */
      /* pthread_mutex_unlock(&now_gps_data_mtx); */
      usleep(10000);
    }
    return NULL;
  }
#endif
  /*++++++++++++++++++++++++++++++++++++++++++++++*/
  /*    Main                                      */
  /*++++++++++++++++++++++++++++++++++++++++++++++*/
int main(int argc, char *argv[])
{
  int s = 0, ch = 0, j = 0, i=0;
  int back_wait_count = 0;
  int back_check_flag = 0;
  int failure_flag_front_b = 0;
  int failure_flag_rear = 0;
  double target_angle = 0;
  pthread_t GPSthread_t;
  pthread_t SplitGPSDatathread_t;
  pthread_t LRFthread_t;
  pthread_t FrontSPmotorthread_t;
  pthread_t Odometorythread_t;
  pthread_t sequence_t;
  pthread_t Serverthread_t;
  pthread_t DisplayIndicatethread_t;

  /* Set Signal */
  if(signal(SIGINT, handler) == SIG_ERR){
    fprintf(stderr, "Signal set error\n");
    exit(1);
  }
  /* Read and set GPS log data to TRGdata[]. */
  gps_log_num = set_GTargetGPS(&WORLD_0, TRGdata, GPS_LOG_DATA);

#ifdef IMCS03
  /* Open iMCs03. */
  if((fd_imcs03 = open(imcs03, O_RDWR)) == -1){
    fprintf(stderr, "%s : Open Error\n", imcs03);
    exit(1);
  }
  /* Setting of iMCs03. */
  set_imcs03(fd_imcs03, &cmd03_uddr);
#endif

#ifdef IMCS01
  /* Open iMCs01. */
  if((fd_imcs01 = open(imcs01, O_RDWR)) == -1){
    fprintf(stderr, "%s : Open Error\n", imcs01);
    exit(-1);
  }
  /* Setting of iMCs01. */
  set_imcs01(fd_imcs01, &cmd01_ccmd, &cmd01_uout);
#endif

  /* Start creating thread. */
#ifdef GPS
  /* Set First Target Point */
  if(argc == 2){
    nextTP = atoi(argv[1]);
  }
  s = pthread_create(&GPSthread_t, NULL, GPSthread, NULL);
  if(s != 0){
    printf("Error GPS thread create!\n");
    return 0;
  }
  s = pthread_create(&SplitGPSDatathread_t, NULL, SplitGPSDatathread, NULL);
  if(s != 0){
    printf("Error Split_GPS_Data thread create!\n");
    return 0;
  }
  s = pthread_create(&sequence_t, NULL, Sequencethread, NULL);
  if(s != 0){
    printf("Error Sequence thread create!\n");
    return 0;
  }
#endif

#ifdef LRF
  s = pthread_create(&LRFthread_t, NULL, LRFthread, NULL);
  if(s != 0){
    printf("Error LRF thread create!\n");
    return 0;
  }
#endif

  usleep(1000);			/* wait for lrf thread */

#ifdef SERVER
  s = pthread_create(&Serverthread_t, NULL, Serverthread, NULL);
  if(s != 0){
    printf("Error LRF thread create!\n");
    return 0;
  }
#endif

#ifdef SPMOTOR
  s = pthread_create(&FrontSPmotorthread_t, NULL, FrontSPmotorthread, NULL);
  if(s != 0){
    printf("Error Front SP motor thread create!\n");
    return 0;
  }
#endif

#ifdef ODOMETORY
  s = pthread_create(&Odometorythread_t, NULL, Odometorythread, NULL);
  if(s != 0){
    printf("Error Odometory thread create!\n");
    return 0;
  }
#endif

  printf("Please Wait ...\n");
  /* Check of pthread start and prepare. */

#ifdef GPS
  while(1){
    /* sequence flag depend on gps flag. */
    pthread_mutex_lock(&pthread_flag_mtx);
    s = flag_check(pthread_flag.sequence);
    if(s > 0){
      pthread_mutex_unlock(&pthread_flag_mtx);
      break;
    }else{
      pthread_mutex_unlock(&pthread_flag_mtx);
    }
  }
  printf("Sequence and GPS [OK]\n");
#endif

#ifdef LRF
  while(1){
    pthread_mutex_lock(&pthread_flag_mtx);
    s = flag_check(pthread_flag.lrf);
    if(s > 0){
      pthread_mutex_unlock(&pthread_flag_mtx);
      break;
    }else{
      pthread_mutex_unlock(&pthread_flag_mtx);
    }
  }
  printf("LRF[OK]\n");
#endif

#ifdef ODOMETORY
  while(1){
    pthread_mutex_lock(&pthread_flag_mtx);
    s = flag_check(pthread_flag.odometory);
    if(s > 0){
      pthread_mutex_unlock(&pthread_flag_mtx);
      break;
    }else{
      pthread_mutex_unlock(&pthread_flag_mtx);
    }
  }
  printf("ODOMETORY[OK]\n");
#endif

#ifdef IMCS01
  stop(fd_imcs01, &cmd01_ccmd);
  printf("iMCs01[OK]\n");
#endif

  printf("Check of all sensors complete. \n");

  while(1){
    /* printf("\033[2J");    //画面クリア */
    pthread_mutex_lock(&rear_enc_mtx);
    printf("odo_state.dir = %lf\n",fmod(odo_state.dir*180/PI,360));
    pthread_mutex_unlock(&rear_enc_mtx);
    pthread_mutex_lock(&now_gps_data_mtx);
    printf("gps_state:\ttime == %lf, La == %f, Lo == %f, QUA == %d, HDOP == %lf\n",
	   NowGdata.time,NowGdata.la ,NowGdata.lo,NowGdata.qua,NowGdata.hdop);
    printf("gps_state:\t x:%f\ty:%f\tdir:%lf\n",NowGdata.pt.x ,NowGdata.pt.y, NowGdata.dir*180/PI);
    pthread_mutex_unlock(&now_gps_data_mtx);
    printf("Please input 1[Enter] to start.\n");
    ch = kakunin(); /* kakunin() will retrun 1 or 2 */
    if(ch == 1){
      break;
    }else{
      usleep(10000);
      continue;
    }
  }

#ifdef IMCS01
  sleep(1);
  pthread_mutex_lock(&imcs01_mtx);
  forward(fd_imcs01, &cmd01_ccmd, 4); /* Access to imcs01. */
  pthread_mutex_unlock(&imcs01_mtx);
  pthread_mutex_lock(&motionflag_mtx);
  motionflag = FORWARD;
  pthread_mutex_unlock(&motionflag_mtx);
  sleep(16);
  pthread_mutex_lock(&motionflag_mtx);
  motionflag = STOP;
  pthread_mutex_unlock(&motionflag_mtx);
  pthread_mutex_lock(&imcs01_mtx);
  stop(fd_imcs01, &cmd01_ccmd); /* Access to imcs01. */
  pthread_mutex_unlock(&imcs01_mtx);
#endif

  odo_state = NowGdata;

#ifdef debug
  s = pthread_create(&DisplayIndicatethread_t, NULL, DisplayIndicatethread, NULL);
  if(s != 0){
    printf("Error Display Indicate thread create!\n");
    return 0;
  }
#endif

#ifdef LRF
  for(i=0;i<4;i++){
    pthread_mutex_lock(&lrf_data_mtx[LRF_REAR]);
    for(i=0; i<lrf_data_max[LRF_REAR]; i++){
      lrf_data_cpy[LRF_REAR][i]    = lrf_data[LRF_REAR][i];
      lrf_data_xy_cpy[LRF_REAR][i] = lrf_data_xy[LRF_REAR][i];
    }
    pthread_mutex_unlock(&lrf_data_mtx[LRF_REAR]);
    check_back_new(lrf_data_cpy[LRF_REAR], lrf_data_xy_cpy[LRF_REAR], lrf_data_max[LRF_REAR]);
  }
#endif

  /* Main loop start. */
  printf("main loop start\n");
  while(quit_flag){

#ifdef LRF
#ifdef ETHERLRF

    for(i=0; i<NUM_OF_LRF; i++){ /* lrf data copy. */
      pthread_mutex_lock(&lrf_data_mtx[i]);
      for(j=0; j<lrf_data_max[i]; j++){
	lrf_data_cpy[i][j]    = lrf_data[i][j];
	lrf_data_xy_cpy[i][j] = lrf_data_xy[i][j];
      }
      pthread_mutex_unlock(&lrf_data_mtx[i]);
    }
    s = check_front_obstacle2(lrf_data_cpy[LRF_FRONT_TOP], lrf_data_xy_cpy[LRF_FRONT_TOP],
			      lrf_data_max[LRF_FRONT_TOP]); /* return 1 is find obstacle, 0 is no obstacle */

    if(s != 0){
      pthread_mutex_lock(&motionflag_mtx);
      if(motionflag != STOP){
	motionflag = STOP;
	pthread_mutex_unlock(&motionflag_mtx);
	pthread_mutex_lock(&imcs01_mtx);
	stop(fd_imcs01, &cmd01_ccmd); /* Access to imcs01. */
	pthread_mutex_unlock(&imcs01_mtx);
	printf("STOP!!!@check_front_obstacle\n");
	continue;
      }else{
	pthread_mutex_unlock(&motionflag_mtx);
	if(failure_flag_front_b < 3){
	  sleep(1);
	  failure_flag_front_b++;
	  printf("failure_flag_front_b count up\n");
	  continue;
	}else{
	  printf("failure_flag_front_b is %d.\n",failure_flag_front_b);
	  failure_flag_front_b = 0;
	  pthread_mutex_lock(&input_angle_mtx);
	  if(s == -1){
	    input_angle = 0; /* Radian to degree. */
	  }else if(s == 1){
	    input_angle = 0;
	  }else if(s == 2){
	    input_angle = 0;
	  }else{
	    input_angle = 0;
	  }
	  pthread_mutex_unlock(&input_angle_mtx);
	  sleep(2);			/* Wait handle. */
	  pthread_mutex_lock(&motionflag_mtx);
	  motionflag = BACK;
	  pthread_mutex_unlock(&motionflag_mtx);
	  pthread_mutex_lock(&imcs01_mtx);
	  back(fd_imcs01, &cmd01_ccmd); /* Access to imcs01. */
	  pthread_mutex_unlock(&imcs01_mtx);
	  back_wait_count = 0;
	  printf("back loop hajimeruyo~~~\n");
	  while(back_wait_count < 500 /* && back_check_flag < 1 */){
	    pthread_mutex_lock(&lrf_data_mtx[LRF_REAR]);
	    for(j=0; j<lrf_data_max[LRF_REAR]; j++){
	      lrf_data_cpy[LRF_REAR][j]    = lrf_data[LRF_REAR][j];
	      lrf_data_xy_cpy[LRF_REAR][j] = lrf_data_xy[LRF_REAR][j];
	    }
	    pthread_mutex_unlock(&lrf_data_mtx[LRF_REAR]);
	    s = check_back_new(lrf_data_cpy[LRF_REAR], lrf_data_xy_cpy[LRF_REAR], lrf_data_max[LRF_REAR]);
	    if(s == -1){
	      break;
	    /* }else if(s == 1){ */
	    /*   printf("hito mituketa!!\n"); */
	    /*   break; */
	    /* }else if(s == 2){ */
	    /*   back_check_flag++; */
	    /*   printf("mono mituketa!!\n"); */
	    /*   break; */
	    }else{
	      usleep(10000);
	      back_wait_count++;
	    }
	  }			/* back wait */
	  printf("back loop owari ^_^v\n");
	  pthread_mutex_lock(&motionflag_mtx);
	  motionflag = STOP;	/* Stop flag */
	  pthread_mutex_unlock(&motionflag_mtx);
	  pthread_mutex_lock(&imcs01_mtx);
	  stop(fd_imcs01, &cmd01_ccmd); /* Access to imcs01. */
	  pthread_mutex_unlock(&imcs01_mtx);
	}
      }
    }else{
      pthread_mutex_lock(&desire_angle_mtx);
      pthread_mutex_lock(&rd_cnt_mtx);
      target_angle = obstacle_avoidance2(lrf_data_cpy[LRF_FRONT_BOTTOM], lrf_data_xy_cpy[LRF_FRONT_BOTTOM], 
					 lrf_data_max[LRF_FRONT_BOTTOM], desire_angle, MAXRANGE, RD_cnt);
      pthread_mutex_unlock(&rd_cnt_mtx);
      pthread_mutex_unlock(&desire_angle_mtx);
    }

    if(target_angle == -360 || 
       target_angle == -540 || 
       target_angle == -720)
      {	/* There is not area to go. */
	if(target_angle == -360)
	  printf("STOP-360\n");
	else if(target_angle == -720)
	  printf("STOP-720\n");
	else
	  printf("STOP-540\n");
	pthread_mutex_lock(&motionflag_mtx);
	motionflag = STOP;
	pthread_mutex_unlock(&motionflag_mtx);
	pthread_mutex_lock(&imcs01_mtx);
	stop(fd_imcs01, &cmd01_ccmd); /* Access to imcs01. */
	pthread_mutex_unlock(&imcs01_mtx);
	if(failure_flag_rear < 3){
	  printf("failure_flag_rear count up\n");
	  sleep(1);
	  failure_flag_rear++;
	  continue;
	}else{
	  printf("failure_flag_rear is %d.\n",failure_flag_rear);
	  failure_flag_rear = 0;
	  pthread_mutex_lock(&input_angle_mtx);
	  input_angle = 0; /* Radian to degree. */
	  pthread_mutex_unlock(&input_angle_mtx);
	  sleep(2);			/* Wait handle to 0. */
	  pthread_mutex_lock(&motionflag_mtx);
	  motionflag = BACK;
	  pthread_mutex_unlock(&motionflag_mtx);
	  pthread_mutex_lock(&imcs01_mtx);
	  back(fd_imcs01, &cmd01_ccmd); /* Access to imcs01. */
	  pthread_mutex_unlock(&imcs01_mtx);
	  back_wait_count = 0;
	  printf("back loop hajimeruyo~~~\n");
	  while(back_wait_count < 500 /* && back_check_flag < 1 */){
	    pthread_mutex_lock(&lrf_data_mtx[LRF_REAR]);
	    for(j=0; j<lrf_data_max[LRF_REAR]; j++){
	      lrf_data_cpy[LRF_REAR][j]    = lrf_data[LRF_REAR][j];
	      lrf_data_xy_cpy[LRF_REAR][j] = lrf_data_xy[LRF_REAR][j];
	    }
	    pthread_mutex_unlock(&lrf_data_mtx[LRF_REAR]);
	    s = check_back_new(lrf_data_cpy[LRF_REAR], lrf_data_xy_cpy[LRF_REAR], lrf_data_max[LRF_REAR]);
	    if(s == -1){
	      break;
	    /* }else if(s == 1){ */
	    /*   printf("hito mituketa!!\n"); */
	    /*   break; */
	    /* }else if(s == 2){ */
	    /*   back_check_flag++; */
	    /*   printf("mono mituketa!!\n"); */
	    /*   break; */
	    }else{
	      usleep(10000);
	      back_wait_count++;
	    }
	  }			/* back wait */
	  printf("back loop owari ^_^v\n");
	  pthread_mutex_lock(&motionflag_mtx);
	  motionflag = STOP;	/* Stop flag */
	  pthread_mutex_unlock(&motionflag_mtx);
	  pthread_mutex_lock(&imcs01_mtx);
	  stop(fd_imcs01, &cmd01_ccmd); /* Access to imcs01. */
	  pthread_mutex_unlock(&imcs01_mtx);
	}
      }else{
      pthread_mutex_lock(&input_angle_mtx);
      if(target_angle != desire_angle){
	input_angle = (-1)*target_angle*180/PI; /* There are obstacles. */
      }else{
	input_angle = (-1)*target_angle*180/PI; /* Noting*/
      }
      pthread_mutex_unlock(&input_angle_mtx);
      pthread_mutex_lock(&motionflag_mtx);
      if(motionflag != FORWARD){
	motionflag = FORWARD;
	pthread_mutex_unlock(&motionflag_mtx);
	pthread_mutex_lock(&imcs01_mtx);
	stop(fd_imcs01, &cmd01_ccmd); /* Access to imcs01. */
	pthread_mutex_unlock(&imcs01_mtx);
	sleep(1);
	pthread_mutex_lock(&imcs01_mtx);
	forward(fd_imcs01, &cmd01_ccmd, 4); /* Access to imcs01. */
	pthread_mutex_unlock(&imcs01_mtx);
	back_check_flag = 0;   /* machine will go ahead, reset back check flag. */
      }else{
	pthread_mutex_unlock(&motionflag_mtx);
      }
    }
#endif
#endif
  }
#ifdef IMCS03
  close(fd_imcs03);
#endif
#ifdef IMCS01
  close(fd_imcs01);
#endif
  return 0;
}
