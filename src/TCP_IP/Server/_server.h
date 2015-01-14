#include <stdio.h>  /* printf()、fprintf()に必要 */
#include <sys/socket.h> /* socket()、bind()、connect()に必要 */
#include <arpa/inet.h> /* sockaddr_in、inet_ntoa()に必要 */
#include <stdlib.h> /* atoi()に必要 */
#include <string.h> /* memset()に必要 */
#include <unistd.h> /* close()に必要 */
#include "./server.h"

#define MAXPENDING NUM_OF_TCP /* 同時にキュー可能な接続要求の最大数 */

//const char* ServIP[NUM_OF_TCP] = {"192.168.0.20"}; 
const unsigned short ServPort[NUM_OF_TCP] = {5000,5100,5200};//,6000};
struct sockaddr_in ServAddr[NUM_OF_TCP];
