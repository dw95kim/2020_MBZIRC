#include "DefineList.h"
#include "CommModule.h"

int flag_command[2] = {0, 0};
int send_interlock = 0;
int call_back_flag = 0;
// node main loop, for ROS
int main(int argc, char** argv)
{
	init(argc, argv, "pump_serial_node");      // node name initialization
	NodeHandle nh;                           // assign node handler

	printf("Initiate: pump_serial_node\n");    // for debugging

// Publish Topic
	ROS_RX_Angle_Data_pub = nh.advertise<std_msgs::Int32MultiArray>("/ROS_RX_Status", 10);
	printf("Initiate: publish rostopic </ROS_RX_Status>\n");   // for debugging

// Subscribe Topic
	ROS_TX_CMD_Data_sub = nh.subscribe("/ROS_CMD", 10, ROS_CMD_Data_Callback);
	printf("Initiate: Subscribe rostopic </ROS_CMD>\n");   // for debugging

	FdPort1 = OpenSerial(PORT1);             // Open Serial
	SerialReceive(FdPort1);                  // Serial Receive (pthread)
	Rate loop_rate(ROS_FREQ);                // setup the loop speed, [Hz]

// node loop, for ROS, check ros status, ros::ok()
	while( ok() )
	{
		UpdateCommand();

		if(call_back_flag == 0)
		{
			StrROS_TX_CMD_Data.Flag = 2;
			SerialSend(FdPort1);
		}
		else
		{
			StrROS_TX_CMD_Data.Flag = flag_command[send_interlock];
			SerialSend(FdPort1);
			STM_State_Publish();

			send_interlock = (send_interlock == 0) ? 1 : 0; // send_interlock = ~send_interlock
		}
		
		count_ros++;
		t_cur = count_ros/ROS_FREQ;

#ifdef DEBUG_MODE
		system("clear");
// status
		printf("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
		printf("\t[USRG] Serial data from STM32Duino\n");
		printf("[Header(0)] \t 0x%X \t[Header(1)] \t 0x%X\n", StrROS_RX_Angle_Data.Header[0], StrROS_RX_Angle_Data.Header[1]);
		printf("[Angle] \t %d\n", StrROS_RX_Angle_Data.angle);
		printf("[Water_detected] \t %d\n", StrROS_RX_Angle_Data.Water_detected);
		printf("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
		printf("\n");
// command
		printf("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
		printf("\t[USRG] Serial data to STM32Duino\n");
		printf("[Header(0)] \t 0x%X \t[Header(1)] \t 0x%X\n", StrROS_TX_CMD_Data.Header[0], StrROS_TX_CMD_Data.Header[1]);
		printf("[ID(0)] \t 0x%X \t[ID(1)] \t 0x%X\n", StrROS_TX_CMD_Data.IDs[0], StrROS_TX_CMD_Data.IDs[1]);
		printf("[Flag] \t %d\n", StrROS_TX_CMD_Data.Flag);
		printf("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
		printf("\n");
#endif

// loop rate [Hz]
		loop_rate.sleep();

// loop sampling, ros
		spinOnce();
	}

// for debugging
	printf("Terminate: pump_serial_node\n");

	return 0;
}


void UpdateCommand(void)
{

}

void STM_State_Publish(void)
{
    ROS_RX_Status.data.push_back(Servo_angle);
    ROS_RX_Status.data.push_back(Water_detected_value);
    ROS_RX_Angle_Data_pub.publish(ROS_RX_Status);
}

void ROS_CMD_Data_Callback(const std_msgs::Int32MultiArray& msg_input)
{
	call_back_flag = 1;
	flag_command[0] = (msg_input.data[0] == 0) ? 2 : 1;
	flag_command[1] = (msg_input.data[1] == 0) ? 3 : 4;
}
