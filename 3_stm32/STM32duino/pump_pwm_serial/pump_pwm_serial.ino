/*
  Controlling a servo position using a potentiometer (variable resistor)
  by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

  modified on 8 Nov 2013
  by Scott Fitzgerald
  http://www.arduino.cc/en/Tutorial/Knob
*/
//#include "header.h"
#include "structs_functions.h"

int temp_angle = 70;

void setup() {
  Serial.begin(115200); // USB, ROS Interface //921600 //115200
#ifdef DEBUG_MODE
  Serial.println("Setup Start");
  Serial.println("");
#endif

  pinMode(PA0, OUTPUT);
  pinMode(PC13, OUTPUT);
  pinMode(PA1, OUTPUT);
  
  pinMode(PA4, OUTPUT);
  pinMode(PA5, OUTPUT);

  pinMode(PA7, INPUT);
  
  digitalWrite(PA1, LOW);
  digitalWrite(PA4, LOW);

  Pump_Servo.attach(PA0); // MAG_LEFT
  
  delay(1000);

  while(temp_angle > 22)
  {
    Pump_Servo.write(temp_angle);
    delay(50);
    temp_angle -= 1;
  }
  
  cur_time = millis();
  prev_time = cur_time;
}



void loop() {
  // put your main code here, to run repeatedly:
  cur_time = millis();
  if (cur_time - prev_time > 500)
  {
    Toggle_OnOff_LED(2, 0);
    prev_time = cur_time;
    flag_ROS_TX_Status = 1;
  }


  //
  // ROS Interface
  //
  if (flag_ROS_TX_Status == 1)
    ROS_TX(1); // [1] TX_Status

  ROS_RX();

  switch (cur_Flag)
  {
    case 1:
      while (temp_angle < 115) //105
      {
        Pump_Servo.write(temp_angle);
        delay(10);
        temp_angle += 1;
      }
      break;
    case 2:
      while (temp_angle > 22) //0
      {
        Pump_Servo.write(temp_angle);
        delay(10);
        temp_angle -= 1;
      }
      break;
    case 3:
      Toggle_OnOff_LED(15, 0);
      digitalWrite(PA1, LOW);
      break;
    case 4:
      Toggle_OnOff_LED(16, 0);
      digitalWrite(PA1, HIGH);
      break;
  }
}
