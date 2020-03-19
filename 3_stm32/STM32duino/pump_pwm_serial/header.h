
//#define DEBUG_MODE
//#define DEBUG_MODE_ROS_RX
//#define DEBUG_MODE_ROS_TX

// --------------------
// Variable Declaration
// --------------------

#include <stdint.h>

#pragma pack(1)
struct struct_ROS_TX_CMD_Data
{
    // -------------
    // Packet Header
    // -------------
    uint8_t  Header[2];           // 0x12, 0x34 (ROS to STM32Duino
    uint8_t  IDs[2];              // 0x02 (Mission 2), 0x03 (Mission 3), 
          	                      // 0x01 (UAV), 0x02 (UGV)
	
    uint8_t  Flag;                // [1] hose stare bottom
                                  // [2] hose stare front
                                  // [3] pump is not wroking
                                  // [4] pump is working
};
#pragma pack()



#pragma pack(1)
struct struct_ROS_RX_STM_Data
{
    // -------------
    // Packet Header
    // -------------
    uint8_t  Header[2];      	  // 0x43, 0x21 (STM32Duino to ROS)
    uint8_t  angle;             // servo angle 
    uint8_t  Water_detected;    // [0] NOT detected
                                // [1] detected
};
#pragma pack()
