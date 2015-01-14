#include "./_server.h"

void tcp_retry(const char* message)
{
  perror(message);
  puts("retry");
}

/*==========================================================================================
  Name     : tcp_error
  Argument : const char* message  : エラー表示する文字
  Return   : -1 (FAILD)
  About    : TCP/IP通信において、エラーが発生した時に呼び出される関数
  Author   : Ryodo Tanaka
  Date     : 2013/09/09
==========================================================================================*/ 
void tcp_error(const char* message)
{
  perror(message);
  exit(-1);
}

/*==========================================================================================
  Name     : server_setup
  Argument : void
  Return   : 1 (SUCCEED)
  About    : サーバーのセットアップを行う
             この関数を一度呼びだせば、serv_sock が listen 状態となる
  Author   : RyodoTanaka
  Date     : 2013/09/09
==========================================================================================*/
int server_setup(void)
{

  int i;

  /* 着信接続用のソケットを作成 */
  for(i=0;i<NUM_OF_TCP;i++){
    if( (serv_sock[i] = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0 )
      tcp_error("socket()");

    memset(&ServAddr[i], 0, sizeof(ServAddr[i])); /* 構造体をゼロで埋める */
    ServAddr[i].sin_family = AF_INET;  /* インターネットアドレスファミリ */
    ServAddr[i].sin_addr.s_addr = htonl(INADDR_ANY);
   

    // ServAddr[i].sin_addr.s_addr = inet_addr(ServIP[i]);
    ServAddr[i].sin_port = htons(ServPort[i]);  /* ローカルポート */

    /* ローカルアドレスへバインド */
    if (bind(serv_sock[i], (struct sockaddr *) &ServAddr[i], sizeof(ServAddr)) < 0)
      tcp_error("bind()");

    /* 着信接続要求のリスン状態というマークをソケットに付ける */
    if (listen(serv_sock[i], MAXPENDING) < 0)
      tcp_error("listen()");
  }

  puts("Server setup compleated");
  return 1;
}

int lrf_cmd2char(char* lrf_cmd_str, const lrf_cmd lrf_cmd_data)
{
  int i=0, j=0, k=0;
  int buf_len = 0;
  int result = 0;
  char* str_buf;

  buf_len = strlen("$LRF");
  str_buf = (char*)calloc(buf_len, sizeof(char));
  if(str_buf == NULL){
    perror("calloc() : str_buf");
    exit(-1);
  }
  sprintf(str_buf,"$LRF");
  for(j=0; j<buf_len; j++){
    lrf_cmd_str[i] = str_buf[j];
    i++;
  }
  lrf_cmd_str[i++] = ',';
  free(str_buf);

  result = lrf_cmd_data.lrf_data_max;
  for(j=0;;j++){
    result /= 10;
    if(result == 0) break;
  }
  buf_len = j+1;
  str_buf = (char*)calloc(buf_len, sizeof(char));
  if(str_buf == NULL){
    perror("calloc() : str_buf");
    exit(-1);
  }
  sprintf(str_buf,"%ld",lrf_cmd_data.lrf_data_max);
  for(j=0; j<buf_len; j++){
    lrf_cmd_str[i] = str_buf[j];
    i++;
  }
  lrf_cmd_str[i++] = ',';
  free(str_buf);

  for(k=0; k<1081; k++){
    result = lrf_cmd_data.lrf_data[k];
    for(j=0;;j++){
      result /= 10;
      if(result == 0) break;
    }
    buf_len = j+1;
    str_buf = (char*)calloc(buf_len, sizeof(char));
    if(str_buf == NULL){
      perror("calloc() : str_buf");
      exit(-1);
    }
    sprintf(str_buf,"%ld",lrf_cmd_data.lrf_data[k]);
    for(j=0; j<buf_len; j++){
      lrf_cmd_str[i] = str_buf[j];
      i++;
    }
    lrf_cmd_str[i++] = ',';
    free(str_buf);
  }
  lrf_cmd_str[i] = '\0';

  return 1;
} 

/* int thf_server_communication(int* clnt_sock,int* serv_sock, thf_cmd* cmd, int send_flg, int recv_flg) */
/* { */

/*   unsigned int Client_Bytes = 0; */
/*   unsigned int thf_Send_Bytes = 0; */
/*   unsigned int thf_Recv_Bytes = 0; */
/*   int Rcvd_Bytes = 0; */
/*   int Total_Rcvd_Bytes = 0; */

/*   Client_Bytes = sizeof(ClntAddr); */

/*   if( (*clnt_sock = accept(*serv_sock, (struct sockaddr*)&ClntAddr, &Client_Bytes)) < 0 ) */
/*     tcp_error("accept()"); */

/*   if(send_flg){ */
/*     thf_Send_Bytes = sizeof(*cmd); */
/*     if(send(*clnt_sock, cmd, sizeof(*cmd), 0) != thf_Send_Bytes) */
/*       tcp_error("send()"); */
/*   } */

/*   if(recv_flg){ */
/*     thf_Recv_Bytes = sizeof(*cmd); */
/*     while(Total_Rcvd_Bytes < (int)thf_Recv_Bytes){ */
/*       if( (Rcvd_Bytes = recv(*clnt_sock, cmd, thf_Recv_Bytes, 0)) <= 0 ) */
/* 	tcp_error("recv()"); */
/*       Total_Rcvd_Bytes += Rcvd_Bytes; */
/*     } */
/*   } */

/*    close(*clnt_sock);  /\* クライアントソケットをクローズする *\/ */

/*   return 1; */
/* } */
