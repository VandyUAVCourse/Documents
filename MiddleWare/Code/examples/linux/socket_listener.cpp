#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <mavlink.h>
 
#define MAXRCVLEN 2047
#define PORTNUM 14550
 
int main(int argc, char *argv[])
{
  uint8_t buffer[MAXRCVLEN + 1]; 
  int recvlen, mysocket;
  socklen_t fromlen;
  struct sockaddr_in dest;
  struct sockaddr_in serv;
  socklen_t socksize = sizeof(struct sockaddr_in);
 
  mysocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
 
  memset(&serv, 0, sizeof(serv));   
  serv.sin_family = AF_INET;
  serv.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  serv.sin_port = htons(PORTNUM);               
 
  //  connect(mysocket, (struct sockaddr *)&dest, sizeof(struct sockaddr));
  bind(mysocket, (struct sockaddr*)&serv, sizeof(struct sockaddr));
  listen(mysocket, 1);
  int consocket = accept(mysocket, (struct sockaddr *)&dest, &socksize);

  while(consocket) {
    //len = recv(mysocket, buffer, MAXRCVLEN, 0);
    recvlen = recvfrom(mysocket, (void*)buffer, MAXRCVLEN, 0, 
		       (struct sockaddr*)&dest, &fromlen);
    
    printf("RECVlen: %d", recvlen);
    if(recvlen > 0) {

      mavlink_message_t msg;
      mavlink_status_t status;
      unsigned int temp = 0;
      
      for(int i = 0; i < recvlen; ++i) {
	temp = buffer[i]; 
	if(mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
	  printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSGID: %d\n", 
		 msg.sysid, msg.compid, msg.len, msg.msgid);
	}
      }
      printf("\n");
    }
    sleep(1);
  }  

  close(mysocket);
  return EXIT_SUCCESS;
}
