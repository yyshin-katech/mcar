#include <katri_obu_interface/katri_v2x.h>

#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include <stdlib.h>

#include <v2x_msgs/intersection_msg.h>
#include <v2x_msgs/intersection_array_msg.h>
#include <std_msgs/String.h>

using namespace std;

KATRI_V2X::KATRI_V2X()
{   
    memset(&local_addr, 0, sizeof(local_addr));
    memset(&from_local_addr, 0, sizeof(from_local_addr));
    memset(buffer, 0, PACKET_SIZE);

    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        perror("socket ");
        exit(0);
    }

    local_addr.sin_family = AF_INET;
    // local_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(LOCAL_PORT);

    setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&optVal, optLen);
    if(sock_fd < 0)
    {
        perror("socket init error ");
        exit(0);
    }

    if(bind(sock_fd, (struct sockaddr *) &local_addr, sizeof(local_addr)) < 0)
    {
        perror("bind error ");
        exit(0);
    }

    memset(&sig_SPaT, 0, sizeof(SIG_SPAT) * 10);
    
}
KATRI_V2X::~KATRI_V2X()
{

}

void KATRI_V2X::end(int sig)
{
    if(sig == 2)
    {
        int test = shutdown(sock_fd, SHUT_RDWR);
		test = close(sock_fd);
		std::cout << "socket_end : " << test << std::endl;
    }
}

void KATRI_V2X::loop(void)
{
    int i = 0;
    v2x_msgs::intersection_array_msg msg1;

    signal(SIGINT, end);

    while(ros::ok())
    {
        ros::spinOnce();

        local_rx_len = sizeof(from_local_addr);

        recv_size = recvfrom(sock_fd, buffer, PACKET_SIZE, 0, (struct sockaddr *) &from_local_addr, (socklen_t*) &local_rx_len);
        if(recv_size < 0)
        {
            perror("recvfrom error ");
            exit(0);
        }
        else
        {
            ROS_INFO("RECV SIZE: %d", recv_size);
        }
        // for(i=0;i<240;i++)
        // {
        //     printf("%2x ", buffer[i]);
        // }
        //     printf("\r\n");
            // ROS_INFO("%d",((SIG_SPAT *)buffer + 8)->Intersection_id);
            // ROS_INFO("%d",((SIG_SPAT *)buffer + 8)->signalGroup);
            // ROS_INFO("%s",((SIG_SPAT *)buffer + 8)->movementName);
            // ROS_INFO("%d",((SIG_SPAT *)buffer + 8)->eventState);
            // ROS_INFO("%d",((SIG_SPAT *)buffer + 8)->minEndTime);

        {
            void *p;

            p = malloc(sizeof(SIG_SPAT) * 20);
            memcpy(p, buffer, sizeof(SIG_SPAT) * 11);
            // memcpy(sig_SPaT, p, sizeof(SIG_SPAT) * 10);
            memcpy(sig_SPaT, (SIG_SPAT *)buffer, sizeof(SIG_SPAT) * 11);

            // ROS_INFO("%d",((SIG_SPAT *)p + 8)->Intersection_id);
            // ROS_INFO("%d",((SIG_SPAT *)p + 8)->signalGroup);
            // ROS_INFO("%s",((SIG_SPAT *)p + 8)->movementName);
            // ROS_INFO("%d",((SIG_SPAT *)p + 8)->eventState);
            // ROS_INFO("%d",((SIG_SPAT *)p + 8)->minEndTime);

            free(p);
        }

        for(i=0;i<11;i++)
        {
            	ROS_INFO("Intersection ID : %d", sig_SPaT[i].Intersection_id);
				ROS_INFO("signalGroup : %d", sig_SPaT[i].signalGroup);
				ROS_INFO("movementName : %s", sig_SPaT[i].movementName);
				ROS_INFO("eventState : %d", sig_SPaT[i].eventState);
				ROS_INFO("minEndTime : %d", sig_SPaT[i].minEndTime);
                ROS_INFO(" ");
        }
        
        // ROS_INFO("%s", &p);
        v2x_msgs::intersection_msg intersection_data;
        std_msgs::String str_msg;

        msg1.time = ros::Time::now();
        for(i=0;i<11;i++)
        {
            intersection_data.IntersectionID = sig_SPaT[i].Intersection_id;
            str_msg.data = sig_SPaT[i].movementName[0];
            intersection_data.Movements.MovementStateName = str_msg.data;
            
            intersection_data.Movements.SignalGroupID = sig_SPaT[i].signalGroup;
            intersection_data.Movements.MovementPhaseStatus = sig_SPaT[i].eventState;
            intersection_data.Movements.TimeChangeDetails = sig_SPaT[i].minEndTime;
            
            msg1.data.push_back(intersection_data);

            memset(&intersection_data, 0, sizeof(intersection_data));
        }
        
        
        pub.publish(msg1);
        msg1.data.clear();

        ROS_INFO("Katri SPaT Mesage Publish");
    }
}