
/* This program extract the Lat lon Height Roll Pitch Yaw from the Spatial and pucblish to topic /array with Rate 10 Hz */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string>
#include <iostream>
#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"

#include </home/vm/catkin_ws/src/geo_referencing/include/geo_referencing/an_packet_protocol.h>
#include </home/vm/catkin_ws/src/geo_referencing/include/geo_referencing/spatial_packets.h>
#include </home/vm/catkin_ws/src/geo_referencing/include/geo_referencing/rs232.h>

#define RADIANS_TO_DEGREES (180.0/M_PI)

int main(int argc, char **argv)
{
    
    // ------------------------------------------------------------------
    an_decoder_t an_decoder;
	an_packet_t *an_packet;
	
	
	system_state_packet_t system_state_packet;
	raw_sensors_packet_t raw_sensors_packet;
	
	int bytes_received;

    char* port="/dev/ttyUSB0";
    int baud_rate=115200;

	if (OpenComport(port, baud_rate))
	{
		printf("Here");
		printf("Could not open serial port\n");
		exit(EXIT_FAILURE);
	}
	
    float lat=0,lon=0,height=0,roll=0,pitch=0, yaw=0;
    //std::cout << lat << " " << lon <<std::endl;

	an_decoder_initialise(&an_decoder);
    //------------------------------------------------------------------


	ros::init(argc, argv, "arrayPublisher");

	ros::NodeHandle n;

	ros::Publisher pub_imu=n.advertise<sensor_msgs::Imu>("/imu_topic",1);
	ros::Publisher pub_gps=n.advertise<sensor_msgs::NavSatFix>("/gps_topic",1);

    //-----------------------------------------------------------------------
   // ros::Time time1= ros::Time::now();
	//ros::Time time2 ;
	while (ros::ok())
	{
      
        if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0)
		{
            /* increment the decode buffer length by the number of bytes received */
			an_decoder_increment(&an_decoder, bytes_received);
            while ((an_packet = an_packet_decode(&an_decoder)) != NULL)
			{
				//std::cout << "The second count is : " << count<< std::endl;
                if (an_packet->id == packet_id_system_state) /* system state packet */
				{
					/* copy all the binary data into the typedef struct for the packet */
					/* this allows easy access to all the different values             */
					if(decode_system_state_packet(&system_state_packet, an_packet) == 0)
					{
						//time2=ros::Time::now();
						//std::cout << "Packet time is " << time2-time1 << std::endl;
						//time1=time2;
						
					//	printf("System State Packet:\n");
					//	printf("\tLatitude = %f, Longitude = %f, Height = %f\n", system_state_packet.latitude * RADIANS_TO_DEGREES, system_state_packet.longitude * RADIANS_TO_DEGREES, system_state_packet.height);
					//	printf("\tRoll = %f, Pitch = %f, Heading = %f\n", system_state_packet.orientation[0] * RADIANS_TO_DEGREES, system_state_packet.orientation[1] * RADIANS_TO_DEGREES, system_state_packet.orientation[2] * RADIANS_TO_DEGREES);
						
                       
						sensor_msgs::Imu imu_msg;
						sensor_msgs::NavSatFix gps_msg;

						imu_msg.header.stamp = ros::Time::now();
						gps_msg.header.stamp=ros::Time::now();
						
						imu_msg.header.frame_id = "/imu_link";
						gps_msg.header.frame_id="/gps_link";	

                        lat=system_state_packet.latitude * RADIANS_TO_DEGREES;
                        
                        lon=system_state_packet.longitude * RADIANS_TO_DEGREES;
                       
                        height=system_state_packet.height;
                        

						gps_msg.latitude=lat;
						gps_msg.longitude=lon;
						gps_msg.altitude=height;

                        roll=system_state_packet.orientation[0] * RADIANS_TO_DEGREES;
                       
                        pitch=system_state_packet.orientation[1] * RADIANS_TO_DEGREES;
                       
                        yaw=system_state_packet.orientation[2] * RADIANS_TO_DEGREES;
                      
						imu_msg.orientation.x=roll;
						imu_msg.orientation.y=pitch;
						imu_msg.orientation.z=yaw;

                       
						pub_imu.publish(imu_msg);
						pub_gps.publish(gps_msg);

						std::cout << " GPS and IMU data has been published "<<std::endl;

                        ros::spinOnce();
                        ros::Duration(0.09968903601).sleep();
						
					}
				}
				
				/* Ensure that you free the an_packet when your done with it or you will leak memory */
				an_packet_free(&an_packet);
                //break;
            }
        }
	}

}