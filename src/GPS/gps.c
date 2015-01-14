#include "_gps.h"
#include "gps.h"
///////////////////////////////////////////////////
// 緯度経度を距離に変換
///////////////////////////////////////////////////
CvPoint2D32f GPS2m(double lad, double lod, CvPoint2D32f* pt_0)
{
  static const double d2r=PI/180.0;
  //static const double r2d=180.0/PI;
  static const double lo_0=LO_0*d2r;
  static const double la_0=LA_0*d2r;
  static const double m0=0.9999;
  static const double a=6378137;
  static const double f=1/298.257222101;
  //static const double b=a*(1-f);
  static const double e=sqrt(2*f-f*f);
  static const double e2=sqrt(2/f-1)/(1/f-1);
  static const double A
    =1+3/4*pow(e, 2)+45/64*pow(e, 4)+175/256*pow(e, 6)+11025/16384*pow(e, 8)+43659/65536*pow(e,10)
    +693693/1048576*pow(e,12)+19324305/29360128*pow(e,14);//+4927697775/7516192768*pow(e,16);
  static const double B
    =3/4*pow(e, 2)+15/16*pow(e, 4)+525/512*pow(e, 6)+2205/2048*pow(e, 8)+72765/65536*pow(e,10)
    +297297/262144*pow(e,12)+135270135/117440512*pow(e,14)+547521975/469762048*pow(e,16);
  static const double C
    =15/64*pow(e, 4)+105/256*pow(e, 6)+2205/4096*pow(e, 8)+10395/16384*pow(e,10)
    +1486485/2097152*pow(e,12)+45090045/58720256*pow(e,14)+766530765/939524096*pow(e,16);
  static const double D
    =35*pow(e, 6)/512+315*pow(e, 8)/2048+31185*pow(e,10)/131072
    +165165*pow(e,12)/524288+45090045*pow(e,14)/117440512+209053845*pow(e,16)/469762048;
  static const double E
    =315*pow(e, 8)/16384+3465*pow(e,10)/65536+99099*pow(e,12)/1048576
    +4099095*pow(e,14)/29360128+348423075*pow(e,16)/1879048192;
  static const double F
    =693*pow(e,10)/131072+9009*pow(e,12)/524288
    +4099095*pow(e,14)/117440512+26801775*pow(e,16)/469762048;
  static const double G
    =3003/2097152*pow(e,12)+315315/58720256*pow(e,14)+11486475/939524096*pow(e,16);
  static const double H=45045/117440512*pow(e,14)+765765/469762048*pow(e,16);
  static const double I=0;//765765/7516192768*pow(e,16);
  static const double B1=a*(1-pow(e,2))*A;
  static const double B2=a*(1-pow(e,2))*(-B/2);
  static const double B3=a*(1-pow(e,2))*(C/4);
  static const double B4=a*(1-pow(e,2))*(-D/6);
  static const double B5=a*(1-pow(e,2))*(E/8);
  static const double B6=a*(1-pow(e,2))*(-F/10);
  static const double B7=a*(1-pow(e,2))*(G/12);
  static const double B8=a*(1-pow(e,2))*(-H/14);
  static const double B9=a*(1-pow(e,2))*(I*16);
  
  double dr,t,n,la,lo,S1;
  double N,W;
  static int i=0;
  static double S0;
  static CvPoint2D32f dst;

  la = lad*d2r;   lo = lod*d2r;
  W  = sqrt(1-pow(e,2)*pow(sin(la),2));
  N  = a/W;
  t  = tan(la);
  n  = sqrt(pow(e2,2)*pow(cos(la),2));
  dr = lo-lo_0;

  //子午線弧長を求める
  if(i==0){
    S0=B1*la_0
      +B2*sin( 2*la_0)+B3*sin( 4*la_0)+B4*sin( 6*la_0)+B5*sin( 8*la_0)
      +B6*sin(10*la_0)+B7*sin(12*la_0)+B8*sin(14*la_0)+B9*sin(16*la_0);
    i++;
  }
  S1=B1*la
    +B2*sin( 2*la)+B3*sin( 4*la)+B4*sin( 6*la)+B5*sin( 8*la)
    +B6*sin(10*la)+B7*sin(12*la)+B8*sin(14*la)+B9*sin(16*la);

  //y座標
  dst.y=((S1-S0)
	 +N*pow(cos(la),2)*t*pow(dr,2)/2
	 +N*pow(cos(la),4)*t*(5-pow(t,2)+9*pow(n,2)+4*pow(n,4))*pow(dr,4)/24
	 -N*pow(cos(la),6)*t*(-61+58*pow(t,2)-pow(t,4)-270*pow(n,2)+330*pow(t,2)*pow(n,2))*pow(dr,6)/720
	 -N*pow(cos(la),8)*t*(-1385+3111*pow(t,2)-543*pow(t,4)+pow(t,6))*pow(dr,8)/40320)*m0;
  //x座標
  dst.x=(N*cos(la)*dr
	 -N*pow(cos(la),3)*(-1+pow(t,2)-pow(n,2))*pow(dr,3)/6
	 -N*pow(cos(la),5)*(-5+18*pow(t,2)-pow(t,4)-14*pow(n,2)+58*pow(t,2)*pow(n,2))*pow(dr,5)/120
	 -N*pow(cos(la),7)*(-61+479*pow(t,2)-179*pow(t,4)+pow(t,6))*pow(dr,7)/5040)*m0;

  dst.y-=pt_0->y;
  dst.x-=pt_0->x;

  return dst;
}

///////////////////////////////////////////////////////////////////////////////////////
///* -- GPS Data Convert double type(latitude, longitude, argument) ---------- */
///////////////////////////////////////////////////////////////////////////////////////
double vlum_convertK(char* row)
{
  double data,/*data1*/ deg,min;//sec
  
  data = atof(row);
  deg  = floor(data/100);
  min  = (data-deg*100)/60;
  //sec  = ((data/100-deg)*100-min*60)*5/3;
  data = deg+min;  
  // printf("%lf",data); 
  return data;
}

////////////////////////////////////////////
//
////////////////////////////////////////////
int set_GTargetGPS(CvPoint2D32f *zero, GPS_Data *trgdata, const char *gps_log_data)
{
  FILE *gpsdat;
  int i,t = 0;
  double la,lo,hdop;
  CvPoint2D32f tmp;

  if((gpsdat = fopen(GPS_LOG_DATA, "r")) == 0){
    printf("Can't open GPS data file\n");
    exit(0);
  }
  for(i = 0; ; i++){
    if(fscanf(gpsdat, "%d,%lf,%lf,%lf", &t, &la, &lo,&hdop) == EOF)
      break;
    //d  = floor(la);
    //m  = floor(la-d)/60;
    //s  = (la-floor(la))*60/3600;
    //la = d+m+s;
    //d  = floor(lo);
    //m  = floor(lo-d)/60;
    //s  = (lo-floor(lo))*60/3600;
    //lo = d+m+s;
    if(i == 0){
      *zero = GPS2m(la, lo, zero);
    }
    tmp = GPS2m(la, lo, zero);
    trgdata->pt = tmp;
    trgdata->r = sqrt(pow(trgdata->pt.x - (trgdata -1)->pt.x,2)
		      +pow(trgdata->pt.y - (trgdata -1)->pt.y,2));
    //    printf("trgdata -> r = %lf\n", trgdata->r);
    trgdata++;
  }
  fclose(gpsdat);
  return i;
}

/*++++++++++++++++++++++++++++++++++++++++++++++*/
/*    SplitString...文字列を指定文字で分割する  */
/*    引数 :                                    */
/*      char *buf[] : 代入先                    */
/*      const char *str : 分割元の文字列        */
/*      const char *delimiter : 分割する文字    */
/*    返り値 : 分割した個数                     */
/*++++++++++++++++++++++++++++++++++++++++++++++*/
int SplitString(char *buf[], const char *str, const char *delimiter)
{
  int i = 0;
  int len = strlen(str);
  char *before, *temp;
  before = (char*)malloc(len + 1);
  strcpy(before, str);
  temp = strtok(before, delimiter);
  if(temp == NULL){
    free(before);
    return 0;
  }else{
    strcpy(buf[0], temp);
    for(i = 1; i < len; i++){
      temp = strtok(NULL, delimiter);
      if(temp != NULL){
	strcpy(buf[i], temp);
      }else{
	break;
      }
    }
  }
  free(before);
  return i;
}

double calc_around_pi(double radian)
{
  if(radian > PI){
    return radian - 2*PI;
  }else if(radian < -PI){
    return radian + 2*PI;
  }else{
    return radian;
  }
}

double kit_map(int target_point, const GPS_Data *target_data, GPS_Data now_data){
  static double x, y = 0;
  static double temp = 0;
  switch(target_point){
  default:
    x = target_data[target_point].pt.x - now_data.pt.x;
    y = target_data[target_point].pt.y - now_data.pt.y;
    temp = atan2(y, x) - now_data.dir;
    temp = calc_around_pi(temp);
    break;
  }
  return temp;
}

int ChangeTargetPoint(const GPS_Data *target_point, GPS_Data now, int nextTP, int NUMofTP)
{
  static int next = 0;
  static int i = 0, j=0;
  static double now_dist = 0;
  static double future_dist = 0;
  static int temp = 0;
  j=0;
  if((NUMofTP - nextTP) > 5){
    for(i = 1; i < 5; i++){
      now_dist = sqrt(pow(target_point[nextTP].pt.x - now.pt.x, 2)
		      + pow(target_point[nextTP].pt.y - now.pt.y,2));
      future_dist = sqrt(pow(target_point[nextTP+i].pt.x - now.pt.x, 2)
			 + pow(target_point[nextTP+i].pt.y - now.pt.y,2));
      if(future_dist < now_dist){
	now_dist = future_dist;
	j=i;
	break;
      }
    }
  }else{
    for(i = 1; i < 5; i++){
      now_dist = sqrt(pow(target_point[nextTP].pt.x - now.pt.x, 2)
		      + pow(target_point[nextTP].pt.y - now.pt.y,2));
      future_dist = sqrt(pow(target_point[nextTP+i].pt.x - now.pt.x, 2)
			 + pow(target_point[nextTP+i].pt.y - now.pt.y,2));
      if(future_dist < now_dist){
	now_dist = future_dist;
	j=i;
	break;
      }
    }
  }
  if(now_dist <= RESET_DIST){
    next = nextTP + j + 1;
  }else{
    next = nextTP + j;
  }
  return next;
}

int ChangeTargetPoint2(const GPS_Data *target_point, GPS_Data now, int nextTP, int NUMofTP)
{
  static int next = 0;
  static int i = 0;
  static double now_dist = 0;
  static double future_dist = 0;
  static int temp = 0;
 
  now_dist = sqrt(pow(target_point[nextTP].pt.x - now.pt.x, 2)
		  + pow(target_point[nextTP].pt.y - now.pt.y,2));

  if(now_dist <= RESET_DIST){	/* go to next TP(TargetPoint). */
    return nextTP+1;
  }else{
    if(nextTP > (NUMofTP - 10)){
      temp = NUMofTP - nextTP;
    }else{
      temp = 10;
    }
    for(i=nextTP+1;i<temp;i++){
      future_dist = sqrt(pow(target_point[nextTP+i].pt.x - now.pt.x, 2)
			 + pow(target_point[nextTP+i].pt.y - now.pt.y,2));
      if(now_dist < future_dist){
	return i-1;		/* hold TP. */
      }
    }
  }
  return nextTP+1;		/* All next 10 TargetPonints are short than now_dist. */
}

double MovingDirectionAverage(GPS_Data now_data, double* value, int loop_num)
{
  static double cache[MOVINGAVERAGENUM];
  static double value_cpy[MOVINGAVERAGENUM];
  static double distance = 0;
  static double distanceABS = 0;
  static double angle1 = 0;
  static double angle2 = 0;
  static double sum = 0;
  static double ave = 0;
  static double tmp = 0;
  static int i = 0;

  for(i=1; i<MOVINGAVERAGENUM; i++){
    value[i-1] = value[i];
  }
  value[MOVINGAVERAGENUM-1] = now_data.dir*180/PI;
  for(i=0; i<MOVINGAVERAGENUM; i++){
    value_cpy[i] = value[i];
  }  
  for(i=0; i<MOVINGAVERAGENUM; i++){
    angle1 = value_cpy[i];
    angle2 = value_cpy[i+1];
    distance = angle2 - angle1;
    distanceABS = fabs(distance);
    if(distanceABS < 180){
      distanceABS = distanceABS;
    }else{
      if(distance < 0){
	distance = distance + 360;
      }else{
	distance = distance - 360;
      }
    }
    cache[i+1] = distance;
  }
  for(i=1; i<MOVINGAVERAGENUM+1; i++){
    value_cpy[i] = value_cpy[i-1] + cache[i];
  }
  for(i=0;i<MOVINGAVERAGENUM; i++){
    sum = sum + value_cpy[i];
  }
  ave = sum / MOVINGAVERAGENUM;
  sum = 0;
  for(i=0; i<MOVINGAVERAGENUM; i++){
    cache[i] = 0;
  }
  return ave*PI/180;
}

double average_angle(double data[],int size)
{
  int i;
  double sum_x = 0.0;
  double sum_y = 0.0;

  for(i=0;i<size;i++){
    sum_x = cos(data[i]);
    sum_y = sin(data[i]);
  }
  return atan2(sum_y,sum_x);
}



/* double KalmanFilterBeta( */


