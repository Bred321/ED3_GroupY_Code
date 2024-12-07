#include <Arduino.h>
#include <esp_now.h>
#include "WIFI.h"
#include "motor_control.hpp"
#include "encoder_reading.hpp"
#include "agv_kinematics.hpp"
#include "esp_now_sender.hpp"
#include "pid.hpp"
#include "commands.h"
#define PID_rate 30
#define PID_interval 1000/PID_rate
PID_CLASS motor1(2, 0.01, 1.1, MOTOR1);
PID_CLASS motor2(2, 0.01, 1.1, MOTOR2);
PID_CLASS motor3(2, 0.01, 1.1, MOTOR3);
PID_CLASS motor4(2, 0.01, 1.1, MOTOR4);
// A pair of varibles to help parse serial commands 
int arg = 0;
int index2 = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

// Variables for encoder value reading
double angle_reading = 0;
double deltaT = 0;
long t = 0;
long t_prev = 0;

long next_PID = 0;

void resetCommand() {
  cmd = '\0';
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index2 = 0;
}

/* Run a command.  Commands are defined in commands.h */
void runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  // Read encoder terminal command
  case READ_PID:
    Serial.print("SP: ");
    Serial.println(motor3.inputSpeed);
    Serial.print("ER: ");
    Serial.println(motor3.error);
    Serial.print("O: ");
    Serial.println(motor3.get_output());
    Serial.print("i: ");
    Serial.println(motor3.integration);
    Serial.print("d: ");
    Serial.println(motor3.derivative);
    Serial.print("ER: ");
    Serial.println(motor3.error);
    Serial.print("Kp: ");
    Serial.println(motor3.Kp);
    Serial.print("Kd: ");
    Serial.println(motor3.Kd);
    Serial.print("Ki: ");
    Serial.println(motor3.Ki);
    break;
  case READ_ENCODERS:
    Serial.print("Motor1: ");
    Serial.println(actual_speed1);
    Serial.print("Motor2: ");
    Serial.println(actual_speed2);
    Serial.print("Motor3: ");
    Serial.println(actual_speed3);
    Serial.print("Motor4: ");
    Serial.println(actual_speed4);
    break;
  // Set motor speeds terminal command
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    
    if (arg1 == 0 && arg2 == 0) {
       motor3.set_input(0);
    }
    else 
    motor3.set_input(arg1);
    motor4.set_input(arg2);
    Serial.println("OK"); 
    break;

  case MOTOR_RAW_PWM:
    /* Reset the auto stop timer */
    drive_motor(0, 0, arg1, 0);
    Serial.println("OK"); 
    break;

  case UPDATE_PID:
     while ((str = strtok_r(p, ":", &p)) != NULL) {
       pid_args[i] = atoi(str);
       i++;
    }
    motor3.set_PID(pid_args[0], pid_args[1], pid_args[2]);
    Serial.println("OK");
    break;
  default:
    Serial.println("Invalid Command");
    break;
  }
}

void setup(){
   Init_Encoder();
   Init_Motor();
   esp_now_setup();
   Serial.begin(115200);
   delay(1000);
}


void loop(){
    t = millis();
    while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == '/') {
      if (arg == 1) argv1[index2] = '\0';
      else if (arg == 2) argv2[index2] = '\0';
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index2] = '\0';
        arg = 2;
        index2 = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index2] = chr;
        index2++;
      }
      else if (arg == 2) {
        argv2[index2] = chr;
        index2++;
      }
    }
  }
    
    if (millis() > next_PID) {
    deltaT = ((double)(t - t_prev))/1.0e3;
    t_prev = t;
    Get_Speed(deltaT);
    // Serial.print(">Setpoint3: ");
    // Serial.println(motor3.inputSpeed);
    // Serial.print(">Speed result3: ");
    // Serial.println(actual_speed3);
    motor3.calculate();
    // motor4.calculate();
    // Serial.print(">deltaT: ");
    // Serial.println(deltaT);
    // drive_motor(0,0,motor3.get_output(),0);
    next_PID += PID_interval;
  }
   
    
    // Encoder reading
    // angle_reading = Get_Angle();
    // Run_Max_Speed();
    // esp_now_send_message();

}