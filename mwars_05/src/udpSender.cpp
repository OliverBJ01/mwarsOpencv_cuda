/*
 * udpSender.cpp
 *
 * sends to pointing stream to PC at 10.1.1.31:9988
 *
 *  QT Project scannerconsole, listener.cpp
 * receives signal sole project on
 *
 *  Created on: 24/08/2014
 *      Author: bernard
 */


#include<arpa/inet.h>
#include<sys/socket.h>
#include <errno.h>
#include<string.h> 	// memset
#include<stdio.h> 		// printf
#include<stdlib.h> 		// exit(0);

#include "mwars.hh"

using namespace std;

#define SERV_PORT 9988						// address of Scanner Console
#define SERV_ADDR "10.1.1.31"

// message structure to send data to scanner
struct opencvMssg {     	// message recvd. from opencv apps
    ushort XAngle;  			//scanner X value
    ushort YAngle;
    bool laserOn;      			// turn laser on
}  ;

struct opencvMssg mssg;
struct sockaddr_in servaddr;
int sockfd,  slen=sizeof(servaddr);


//=============================================
//
// void sendMssg(uint x, uint y, bool laserOn)
//
//  sends UDP message to Scanner Console
//=============================================
void sendMssg(uint x, uint y, bool laserOn)  {

	//printf("sendMssg() %d %d l:%d \n", x, y, laserOn);

	mssg.XAngle = htons(x);
	mssg.YAngle = htons(y);
	mssg.laserOn = laserOn;

	//printf(" l:%d \n", mssg.laserOn);
	//printf("sendMssg()  %d %d l:%d \n", mssg.XAngle, mssg.YAngle, mssg.laserOn);
	// use sendto() for UDP because unconnected and need specify destination each transmission
	if (sendto (sockfd,  & mssg, sizeof(struct opencvMssg) , 0 , (struct sockaddr *) &servaddr, slen)==-1)      {
			cerr << "sendMssg failed: " << strerror(errno) << endl;
			return ;
	}
}


//================== ===============================
//	int initSocket(void )
//
//  creates UDP socket to the Scanner Console
//
//==================  ===============================
int initSocket(void )   {

	// create socket
	if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		cerr << "socket error: " << strerror(errno) << endl;
		return 1 ;
	}

    memset((char *) &servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(SERV_PORT);

    //if (inet_aton(SERV_ADDR , &servaddr.sin_addr) == 0)      {
    if ((inet_pton(AF_INET, SERV_ADDR , &servaddr.sin_addr) == 0))      {
			cerr << "inet_aton() failed: " << strerror(errno) << endl;
			return 1;
    }
    cout << "UDP socket to scanner server created" << endl;
    return 0;
}
