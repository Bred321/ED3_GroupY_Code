#include <Arduino.h>
#include <iostream>
#include <string>
#include "WIFI.h"
#include "motor_control.hpp"
#include "encoder_reading.hpp"
#include "agv_kinematics.hpp"
#include "pid.hpp"
#include "commands.h"
// Set PID rate as 30 times per loop
#define PID_rate 30
#define PID_interval 1000/PID_rate
Inverse_Output kinematic;
// Declare 4 motors using PID
PID_CLASS motor1(0.5, 1.5, 0.09, MOTOR1); // Kp:2/Kd: 3.95/Ki: 0.12
PID_CLASS motor2(0.5, 1.5, 0.09, MOTOR2);
PID_CLASS motor3(0.5, 1.5, 0.09, MOTOR3);
PID_CLASS motor4(0.5, 1.5, 0.09, MOTOR4);
/* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
#define AUTO_STOP_INTERVAL 160000 //2000
long lastMotorCommand = AUTO_STOP_INTERVAL;
// A pair of varibles to help parse serial commands 
int arg = 0;
int index2 = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the arguments
char argv1[16];
char argv2[16];
char argv3[16];
char argv4[16];

// The arguments converted to integers
long arg1;
long arg2;
long arg3;
long arg4;

// Variables for encoder value reading
double angle_reading = 0;
double deltaT = 0;
long t = 0;
long t_prev = 0;
long next_PID = 0;
// Reset all command and ready to store new arguments
void resetCommand() {
  cmd = '\0';
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  memset(argv4, 0, sizeof(argv4));
  arg1 = 0;
  arg2 = 0;
  arg3 = 0;
  arg4 = 0;
  arg = 0;
  index2 = 0;
}
// Execute the command from Serial
void runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  double pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  arg3 = atoi(argv3);
  arg4 = atoi(argv4);
  
  switch(cmd) {
  // Set speeds in RPM to all motors
  case MOTOR_SPEEDS_ALL:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0) {
      drive_motor(0, 0, 0, 0);
      motor1.reset_PID();
      motor2.reset_PID();
      motor3.reset_PID();
      motor4.reset_PID();
      moving = 0;
    }
    else moving = 1;
    motor1.set_input(arg1);
    motor2.set_input(arg1);
    motor3.set_input(arg1);
    motor4.set_input(arg1);
    Serial.println("OK"); 
  break;
  // Print out all the PID parameters
  case READ_PID:
  // Value of motor 1
  if(arg1 == 1)
  {
    Serial.println("--------MOTOR1 PID--------");
    Serial.print("SP: ");
    Serial.println(motor1.inputSpeed);
    Serial.print("ER: ");
    Serial.println(motor1.error);
    Serial.print("O: ");
    Serial.println(motor1.get_output());
    Serial.print("i: ");
    Serial.println(motor1.integration);
    Serial.print("d: ");
    Serial.println(motor1.derivative);
    Serial.print("Kp: ");
    Serial.println(motor1.Kp,3);
    Serial.print("Kd: ");
    Serial.println(motor1.Kd,3);
    Serial.print("Ki: ");
    Serial.println(motor1.Ki,3);
  }
   // Value of motor 2
  else if(arg1 == 2)
   {
    Serial.println("--------MOTOR2 PID--------");
    Serial.print("SP: ");
    Serial.println(motor2.inputSpeed);
    Serial.print("ER: ");
    Serial.println(motor2.error);
    Serial.print("O: ");
    Serial.println(motor2.get_output());
    Serial.print("i: ");
    Serial.println(motor2.integration);
    Serial.print("d: ");
    Serial.println(motor2.derivative);
    Serial.print("Kp: ");
    Serial.println(motor2.Kp,3);
    Serial.print("Kd: ");
    Serial.println(motor2.Kd,3);
    Serial.print("Ki: ");
    Serial.println(motor2.Ki,3);
  }
   // Value of motor 3
  else if(arg1 == 3)
   {
    Serial.println("--------MOTOR3 PID--------");
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
    Serial.print("Kp: ");
    Serial.println(motor3.Kp,3);
    Serial.print("Kd: ");
    Serial.println(motor3.Kd,3);
    Serial.print("Ki: ");
    Serial.println(motor3.Ki,3);
  }
   // Value of motor 4
  else if(arg1 == 4)
   {
    Serial.println("--------MOTOR4 PID--------");
    Serial.print("SP: ");
    Serial.println(motor4.inputSpeed);
    Serial.print("ER: ");
    Serial.println(motor4.error);
    Serial.print("O: ");
    Serial.println(motor4.get_output());
    Serial.print("i: ");
    Serial.println(motor4.integration);
    Serial.print("d: ");
    Serial.println(motor4.derivative);
    Serial.print("Kp: ");
    Serial.println(motor4.Kp,3);
    Serial.print("Kd: ");
    Serial.println(motor4.Kd,3);
    Serial.print("Ki: ");
    Serial.println(motor4.Ki,3);
  }
    break;
  case KINEMATIC:
  /* Reset the auto stop timer */
    lastMotorCommand = millis();
  if (arg1 == 0 && arg2 == 0 && arg3 == 0 && arg4 == 0) {
      drive_motor(0, 0, 0, 0);
      motor1.reset_PID();
      motor2.reset_PID();
      motor3.reset_PID();
      motor4.reset_PID();
      moving = 0;
    }
    else moving = 1;
    calculate_inverse_kinematics(arg1, arg2, arg3);
    motor1.set_input(kinematic.v1);
    motor2.set_input(kinematic.v2);
    motor3.set_input(kinematic.v3);
    motor4.set_input(kinematic.v4);
    Serial.println("OK");

  break;
  // Read speeds in RPM of each motor
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
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0 && arg3 == 0 && arg4 == 0) {
      drive_motor(0, 0, 0, 0);
      motor1.reset_PID();
      motor2.reset_PID();
      motor3.reset_PID();
      motor4.reset_PID();
      moving = 0;
    }
    else moving = 1;
    motor1.set_input(arg1);
    motor2.set_input(arg2);
    motor3.set_input(arg3);
    motor4.set_input(arg4);
    Serial.println("OK"); 
    break;
  // Drive all motors using PWM signal from 0 - 255
  case MOTOR_RAW_PWM:
    lastMotorCommand = millis();
    motor1.reset_PID();
    motor2.reset_PID();
    motor3.reset_PID();
    motor4.reset_PID();
    moving = 0;
    /* Reset the auto stop timer */
    drive_motor(arg1, arg2, arg3, arg4);
    Serial.println("OK"); 
    break;
  // Change the PID parameters of each motor - Kd Ki Kp
  case UPDATE_PID:
    while ((str = strtok_r(p, "/", &p)) != NULL) {
      if(i < 3) {
        pid_args[i] = std::atof(str);
        switch(i) {
          case 0:
            Serial.print("Kp: ");
            Serial.println(pid_args[i]);
            break;
          case 1:
            Serial.print("Kd: ");
            Serial.println(pid_args[i]);
            break;
          case 2:
            Serial.print("Ki: ");
            Serial.println(pid_args[i]);
        }
        i++;
      }
    }
    motor1.set_PID(pid_args[0], pid_args[1], pid_args[2]);
    motor2.set_PID(pid_args[0], pid_args[1], pid_args[2]);
    motor3.set_PID(pid_args[0], pid_args[1], pid_args[2]);
    motor4.set_PID(pid_args[0], pid_args[1], pid_args[2]);
    Serial.println("OK");
    break;

  default:
    Serial.println("Invalid Command");
    break;
  }
}
void setup(){
  //Set up drivers and motors
   Init_Encoder();
   Init_Motor();
   //Set up Serial communication
   Serial.begin(115200);
   Serial2.begin(115200);
   // Delay for Serial initialization
   delay(2000);
}

void loop() {
  t = millis();
  while (Serial.available() > 0) {
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == '\'') {
      if (arg == 1) argv1[index2] = '\0';
      else if (arg == 2) argv2[index2] = '\0';
      else if (arg == 3) argv3[index2] = '\0';
      else if (arg == 4) argv4[index2] = '\0';
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1) {
        argv1[index2] = '\0';
        arg = 2;
        index2 = 0;
      }
      else if (arg == 2) {
        argv2[index2] = '\0';
        arg = 3;
        index2 = 0;
      }
      else if (arg == 3) {
        argv3[index2] = '\0';
        arg = 4;
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
        argv1[index2] = chr;
        index2++;
      }
      else if (arg == 2) {
        argv2[index2] = chr;
        index2++;
      }
      else if (arg == 3) {
        argv3[index2] = chr;
        index2++;
      }
      else if (arg == 4) {
        argv4[index2] = chr;
        index2++;
      }
    }
  }
  // Do PID for all motors with fixed interval
  if (millis() > next_PID) {
    deltaT = ((double)(t - t_prev)) / 1.0e3;
    t_prev = t;
    Get_Speed(deltaT);
    motor1.do_PID();
    motor2.do_PID();
    motor3.do_PID();
    motor4.do_PID();
    next_PID += PID_interval;
  // Send the speed values of motors to web
    Serial2.print(actual_speed1);
    Serial2.print(" ");
    Serial2.print(actual_speed2);
    Serial2.print(" ");
    Serial2.print(actual_speed3);
    Serial2.print(" ");
    Serial2.println(actual_speed4);
  }
  // Auto stop the robot after certain interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    drive_motor(0, 0, 0, 0);
    moving = 0;
    delay(1);
  }
}
