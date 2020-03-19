
#include "header.h"
#import <Arduino.h>
#include <Servo.h>

Servo Pump_Servo;
// -------------------
// Variable Declaration
// -------------------

//
// structs
struct struct_ROS_TX_CMD_Data     StrTXCommand;
struct struct_ROS_RX_STM_Data      StrRXStatus;

//
// Variable
uint32_t cur_time = 0;
uint32_t prev_time = 0;
int count_loop = 0;

int flag_TX_Command = 0;
int flag_ROS_TX_Status = 0;

int time_MAG = 0;

int cur_Flag = 0;
int cur_angle = 0;

int initial_angle = 0;

// --------------------
// Function Declaration
// --------------------

void Toggle_OnOff_LED(int LED_Number, int Status) {
  static int LED_HEARTBEAT = 0; // [0] LOW,   [1] HIGH
  static int LED_RX = 0;
  static int LED_check = 0;
  switch(LED_Number)
  {
    case 2: // heartbeat
      LED_HEARTBEAT = ~LED_HEARTBEAT;
      digitalWrite(PC13, LED_HEARTBEAT);
      break;
    case 14:
      LED_RX = ~LED_RX;
      digitalWrite(PA4, LED_RX);
      break;
    case 15:
      LED_check = ~LED_check;
      digitalWrite(PA5, LED_check);
      break;
    case 16:
      LED_check = ~LED_check;
      digitalWrite(PA6, LED_check);
      break;
  }
}


void ROS_TX(int TX_MODE) {
  int i;
  
  switch(TX_MODE)
  {
    case 1: // Magnet Status
#ifdef DEBUG_MODE_ROS_TX
      Serial.print("[TX_MODE]\t");
#endif
      if (flag_ROS_TX_Status == 1)
      {
          StrRXStatus.Header[0] = 0x43;
          StrRXStatus.Header[1] = 0x21;
          StrRXStatus.angle = Pump_Servo.read();
          StrRXStatus.Water_detected = digitalRead(PA7);
//          StrRXStatus.angle = 0x0;
//          StrRXStatus.Water_detected = 0x0;
          Serial.write(&StrRXStatus, sizeof(struct_ROS_RX_STM_Data));
          flag_ROS_TX_Status = 0;

          cur_angle = StrRXStatus.angle;
      
#ifdef DEBUG_MODE_ROS_TX
          Serial.println("[ROS (Agents' Status) TX Data]");
    
          Serial.print("[Packet_Size]\t"); Serial.println(sizeof(struct_ROS_RX_STM_Data));
          
          Serial.print("[Header(0)]\t0x"); Serial.print(StrRXStatus.Header[0], HEX);
          Serial.print("\t[Header(1)]\t0x"); Serial.println(StrRXStatus.Header[1], HEX);
          
          Serial.print("[Angle]\t\t0x"); Serial.println(StrRXStatus.angle, HEX);
          Serial.print("[Water_Dectected]\t0x"); Serial.println(StrRXStatus.Water_detected, HEX);
          Serial.println(" ");
#endif
      }
      break;
    
    default:
#ifdef DEBUG_MODE_ROS_TX
      Serial.println("\t[TX_MODE]\tNaN");
#endif
      break;
  }
}





void ROS_RX(void) {
  static int ParsingMode = 1;
  uint8_t    TempData[sizeof(struct_ROS_TX_CMD_Data)];
  uint8_t    HeaderA;
  uint8_t    HeaderB;
#ifdef DEBUG_MODE_ROS_RX
  int        i;
#endif

  switch(ParsingMode)
  {
    case 1:
      if(Serial.available() >= 1)
      {
//        Serial.println("line 123");
        Serial.readBytes(&TempData[0], 1);
        if(TempData[0] == 0x12)
        {
//          Serial.println("line 127");
          HeaderA = TempData[0];
          ParsingMode = 2;
        }
        else
        {
//          Serial.println("line 132");
          ParsingMode = 1;
        }
      }
      break;
    case 2:
      if(Serial.available() >= 1)
      {
//        Serial.println("line 139");
        Serial.readBytes(&TempData[1], 1);
        if(TempData[1] == 0x34)
        {
        HeaderB = TempData[1];
          ParsingMode = 3;
        }
        else
        {
          ParsingMode = 1;
        }
      }
      break;
    case 3:
      if(Serial.available() >= 2)
      {
//        Serial.println("line 155");
        Serial.readBytes(&TempData[2], 2);
        if((TempData[2] == 0x03) && (TempData[3] == 0x01))
        {
          StrTXCommand.Header[0] = HeaderA;
          StrTXCommand.Header[1] = HeaderB;
          StrTXCommand.IDs[0] = TempData[2];
          StrTXCommand.IDs[1] = TempData[3];
          ParsingMode = 4;
        }
        else
        {
          ParsingMode = 1;
        }
      }
      break;
    case 4:
      if(Serial.available() >= (sizeof(struct_ROS_TX_CMD_Data)-4))
      {
//        Serial.println("line 174");
        Serial.readBytes(&TempData[4], (sizeof(struct_ROS_TX_CMD_Data)-4));
        memcpy((void *)(&StrTXCommand.Flag), (void *)(&TempData[4]), (sizeof(struct_ROS_TX_CMD_Data)-4));
        cur_Flag = StrTXCommand.Flag;
//        Toggle_LED(1);
        Toggle_OnOff_LED(14, 0);
        flag_TX_Command = 1;
        ParsingMode = 1;
    
#ifdef DEBUG_MODE_ROS_RX
        Serial.println("[ROS (Command Message) RX Data]");
    
        Serial.print("[ParsingMode]\t"); Serial.print(ParsingMode);
        Serial.print("\t[Packet_Size]\t"); Serial.println(sizeof(struct_ROS_TX_CMD_Data));
         
        Serial.print("[Header(0)]\t0x"); Serial.print(StrTXCommand.Header[0], HEX);
        Serial.print("\t[Header(1)]\t0x"); Serial.print(StrTXCommand.Header[1], HEX);
        Serial.print("\t[IDs(0)]\t0x"); Serial.print(StrTXCommand.IDs[0], HEX);
        Serial.print("\t[IDs(1)]\t0x"); Serial.println(StrTXCommand.IDs[1], HEX);
          
        Serial.print("[Flag]\t\t0x"); Serial.println(StrTXCommand.Flag, HEX);
        Serial.println(" ");
#endif
      }
      break;
  }
}
