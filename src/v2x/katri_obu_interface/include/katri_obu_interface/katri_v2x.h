#ifndef KATRI_V2X_
#define KATRI_V2X_

// #include <iostream>
// #include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>
#include <string.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <ros/ros.h>
#include <v2x_msgs/intersection_array_msg.h>

#define LOCAL_PORT  50000
#define PACKET_SIZE 1024

typedef struct{
	int Intersection_id;
	unsigned char movementName[5];
	int signalGroup;
	int eventState;
	unsigned int minEndTime;
} SIG_SPAT;

static int sock_fd;

class KATRI_V2X{

    public:

        KATRI_V2X();
        ~KATRI_V2X();

        ros::Publisher pub;

        SIG_SPAT sig_SPaT[10];

        struct sockaddr_in local_addr;
        struct sockaddr_in from_local_addr;

        socklen_t local_rx_len;

        // int sock_fd;

        unsigned char buffer[PACKET_SIZE] = {0,};
        int recv_size;

        int optVal = 10000;
        int optLen = sizeof(optVal);

        void loop(void);

        static void end(int sig);

};
#endif