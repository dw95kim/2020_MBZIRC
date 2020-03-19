
#ifndef DefineList_H
#define DefineList_H

// essential header for ROS
#include <ros/ros.h>

// for using serial communication
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>

// for topic message
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include "std_msgs/Int32MultiArray.h"

std_msgs::Int32MultiArray ROS_RX_Status;
std_msgs::Int32 Water_detected;

// setup the initial name
using namespace ros;
using namespace std;

#define DEBUG_MODE
#define DEBUG_MODE_Agent_List
#define ROS_FREQ  5.0    // while loop frequency [Hz]

#define PORT1 		"/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"//"/dev/ttyUSB0"
#define BAUDRATE 	115200

#define D2R	        3.1415926535/180.0
#define eps             0.000001


int   FdPort1;
int   count_ros = 0;
float t_cur     = 0.0;

int   Servo_angle = 0;
int   Water_detected_value = 0;


// ROS topic define
ros::Publisher ROS_RX_Angle_Data_pub;
ros::Subscriber ROS_TX_CMD_Data_sub;

float satmax(float, float);
float satmin(float, float);
void  CalChecksumAB(void *, int32_t, uint8_t *, uint8_t *);

int   OpenSerial(char* device_name);
void  SerialSend(int fd);
void  SerialReceive(int FdPort1);
void* receive_p_thread(void *fd);

void  UpdateCommand(void);
void  STM_State_Publish(void);
void  ROS_CMD_Data_Callback(const std_msgs::Int32MultiArray& msg_input);

#pragma pack(1)
struct struct_ROS_TX_CMD_Data
{
    // -------------
    // Packet Header
    // -------------
    uint8_t  Header[2];           // 0x12, 0x34 (ROS to STM32Duino
    uint8_t  IDs[2];              // 0x02 (Mission 2), 0x03 (Mission 3), 
          	                  // 0x01 (UAV), 0x02 (UGV)
    uint8_t  Flag;                // [0/1, 0/1], [Servo Bottom/Front, Pump Off/On]

};
#pragma pack()

#pragma pack(1)
struct struct_ROS_RX_STM_Data
{
    // -------------
    // Packet Header
    // -------------
    uint8_t  Header[2];      	  // 0x43, 0x21 (STM32Duino to ROS)
    uint8_t  angle          = 0;  // servo angle
    uint8_t  Water_detected = 0;  // [0] NOT detected
                                  // [1] detected 
};
#pragma pack()


struct struct_ROS_TX_CMD_Data  StrROS_TX_CMD_Data;
struct struct_ROS_RX_STM_Data  StrROS_RX_Angle_Data;


float satmax(float data, float max)
{
    float res;

    if(fabs(data) > max)
        res = (data + eps)/fabs(data + eps)*max;
    else
        res = data;

    return res;
}


float satmin(float data, float min)
{
    float res;

    if(fabs(data) < min)
        res = (data + eps)/fabs(data + eps)*min;
    else
        res = data;

    return res;
}

void CalChecksumAB(void * PtrStartAddr, int32_t Size, uint8_t * PtrChecksumA, uint8_t * PtrChecksumB)
{
	int32_t Ind;
	uint8_t * Ptr = (uint8_t *)PtrStartAddr;

	uint8_t ChecksumA = 0;
	uint8_t ChecksumB = 0;

	for(Ind = 0; Ind<Size; Ind++)
	{
		ChecksumA += Ptr[Ind];
		ChecksumB += ChecksumA;
	}

	*PtrChecksumA = ChecksumA;
	*PtrChecksumB = ChecksumB;
}



#endif
