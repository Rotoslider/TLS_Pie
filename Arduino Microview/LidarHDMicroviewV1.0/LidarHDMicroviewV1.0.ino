/*
 * Title       LidarHDMicroview
 * by          Donny Mott
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Description:
 *   Single axis stepper motor control with OLED dsiplay using a Microview controller. This software is for use with a DIY Pie Powered Terrestrial Laser Scanner 
 *   Button inputs to select various scan settings and to stop and reset
 *   180 degree scan at 1 deg/sec
 *   360 degree scan at 2 deg/sec
 *   360 degree scan at 1 deg/sec
 *   
 *   When scan buttons are pressed an output is fired to tell a Raspberry Pie to start recording the Lidar data in a PCAP file.
 *   The code below is designed to work with a 0.9deg 400 step per rev stepper motor and a 50:1 gear reducer. The code is currenlty set to 32 microsteps.
 *   The code is easily changed to support whatever stepper motor and gearbox you choose to use.
 *
 * Author: Donny Mott
 * 
 *   3dmapmaker@gmail.com
 *
 * 
 */
#include <stdio.h>
#include <Arduino.h>
#include <SpeedyStepper.h>
#include <MicroView.h>

const int pan360s = A0; //button
const int pan360f = A1; //button
const int pan180s = A2; //button
const int KILL = 2; //button
const int MOTOR_EN_PIN = 6; //pan motor enable
const int MOTOR_STEP_PIN = 5; //pan motor step
const int MOTOR_DIRECTION_PIN = 3; //pan motor direction
const int RECORDSTART = A4; //output to start bag record on Pie
const int RECORDSTOP = A3; // output to stop bag record on Pie

bool buttonS360pressed = false;
bool buttonF360pressed = false;
bool buttonS180pressed = false;

SpeedyStepper stepper;

// 'lidar logo, 64x48px
uint8_t logo [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x80, 0xC0, 0xE0, 0x70, 0x30, 0x18, 0x18, 0x0C, 0x0C, 0x0C, 0x06, 0x06, 0x86, 0x86,
0x86, 0x86, 0x06, 0x06, 0x0C, 0x0C, 0x0C, 0x18, 0x18, 0x30, 0x70, 0xE0, 0xC0, 0x80, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0,
0xFC, 0x1E, 0x07, 0x01, 0x00, 0x00, 0xE0, 0x78, 0x1C, 0x0E, 0x06, 0x03, 0x83, 0xC3, 0xC1, 0xC1,
0xC1, 0xC1, 0xC3, 0x83, 0x03, 0x06, 0x0E, 0x1C, 0x78, 0xE0, 0x00, 0x00, 0x01, 0x07, 0x1E, 0xFC,
0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F,
0xFF, 0x80, 0x00, 0x00, 0x00, 0x07, 0x3F, 0x70, 0x40, 0x00, 0x00, 0x00, 0x0F, 0x1F, 0x3F, 0x3F,
0x3F, 0x3F, 0x1F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x78, 0x3F, 0x07, 0x00, 0x00, 0x00, 0x80, 0xFF,
0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x01, 0x07, 0x0E, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xF0, 0xF8, 0x78, 0x78, 0x78,
0x78, 0x78, 0x78, 0xF8, 0xF0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0E, 0x07, 0x01,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0xE3, 0x39, 0x0C, 0x06, 0x02, 0x02, 0x03,
0x03, 0x02, 0x02, 0x06, 0x0C, 0x39, 0xE3, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7C, 0x71, 0x67, 0x4C, 0x58, 0x10, 0x30, 0x30,
0x30, 0x30, 0x10, 0x58, 0x4C, 0x67, 0x71, 0x7C, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
void setup() {
    Serial.begin(9600);
    delay(500);  // needed to be able to flash a new sketch more easily
     
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
  
  pinMode(pan360s, INPUT); //Start Motor @ 1 degree per second for 360 degrees
  pinMode(pan360f, INPUT); //Start Motor @ 2 degree per second for 360 degrees
  pinMode(pan180s, INPUT); //Start Motor @ 1 degree per second for 180 degrees
  pinMode(KILL, INPUT); //Stop Motor
  pinMode(MOTOR_EN_PIN, OUTPUT);
  pinMode(RECORDSTART, OUTPUT);
  pinMode(RECORDSTOP, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(KILL), KILL_SCAN, RISING);
  
  digitalWrite(MOTOR_EN_PIN, HIGH); // disable motor driver
  digitalWrite(RECORDSTART, HIGH); // makes sure not to trigger pie
  digitalWrite(RECORDSTOP, HIGH); // makes sure not to trigger pie

    uView.begin();              // start MicroView
    uView.clear(PAGE);          // erase the memory buffer, when next uView.display() is called, the OLED will be cleared.
    uView.display();            // display the content in the buffer memory
        

  // Display start
    uView.clear(PAGE);
    uView.drawBitmap(logo);
    uView.display();
    delay(2000);
    uView.clear(PAGE);
    uView.setFontType(0); // set font type 0: Numbers and letters. 10 characters per line (6 lines)
    uView.setCursor(6,5);
    uView.print("STARTING"); // display string
    uView.display();
    delay(500);
    uView.setCursor(14,20);
    uView.print("LIDAR");
    uView.display();
    delay(500);
    uView.setCursor(10,36);
    uView.print("SCANNER");
    uView.display();
    
    delay(1500);
    uView.clear(PAGE);
    uView.display();
    uView.setFontType(0); // set font type 0: Numbers and letters. 10 characters per line (6 lines)
    uView.setCursor(14,5);
    uView.print("SELECT"); // display string
    uView.display();
    delay(500);
    uView.setCursor(20,20);
    uView.print("SCAN");
    uView.display();
    uView.setCursor(20,36);
    uView.print("TYPE");
    uView.display();
    delay(750);
}

void loop() {
  readButtons();
  actOnButtons();
}

void readButtons() {
    buttonS360pressed = false;
    buttonF360pressed = false;
    buttonS180pressed = false;

    if (digitalRead(pan360s) == LOW) {
        buttonS360pressed = true;        
    }
    if (digitalRead(pan360f) == LOW) {
        buttonF360pressed = true;
    }
    if (digitalRead(pan180s) == LOW) {
        buttonS180pressed = true;
    }    
}

void actOnButtons() {
    if (buttonS360pressed == true) {
        PANO360S();
    }
    if (buttonF360pressed == true) {
        PANO360F();
    }
    if (buttonS180pressed == true) {
        PANO180S();
    }
}   
 

void PANO360S() {
    digitalWrite(MOTOR_EN_PIN, LOW); // enable drive
    stepper.setStepsPerRevolution(640000); //CSF-14-50 harmonic drive 50:1, 32 microsteps, 0.9 deg motor
    stepper.setSpeedInRevolutionsPerSecond(0.0027778); // 1 degree per second, 360 degrees in 6 minutes
    stepper.setAccelerationInRevolutionsPerSecondPerSecond(1.0);
    stepper.setCurrentPositionInSteps(0);
    uView.clear(PAGE);
    uView.display();
    delay(100);
    //uView.setFontType(0);
    //uView.setCursor(7,4);
    //uView.print("SCANNING"); 
    //uView.setFontType(1);
    //uView.setCursor(17,18);
    //uView.print("360");
    uView.setFontType(0);
    uView.setCursor(5,36);
    uView.print("1 DEG/SEC");
    uView.display();
    delay(2000);
    uView.clear(PAGE);
    uView.display();
    delay(100);
    uView.setFontType(0);
    uView.setCursor(7,4);
    uView.print("SCANNING"); 
    uView.setFontType(1);
    uView.setCursor(17,18);
    uView.print("360");
    uView.setFontType(0);
    uView.setCursor(5,36);
    uView.print("6 Minutes");
    uView.display();
    digitalWrite(RECORDSTART, LOW); // starts bag record
    delay(1);
    digitalWrite(RECORDSTART, HIGH);
    //delay(200);
    stepper.moveToPositionInRevolutions(1.05); // rotates 378 degrees. Add some rotation to be sure and record a full 360 after TCPDUMP starts
    digitalWrite(RECORDSTOP, LOW); // stops bag record
    delay(10);
    digitalWrite(RECORDSTOP, HIGH);
    delay(1000);
    stepper.setSpeedInRevolutionsPerSecond(0.1);
    stepper.moveToPositionInRevolutions(1); // back to start
    delay(100);
    digitalWrite(MOTOR_EN_PIN, HIGH); // disable driver
    uView.clear(PAGE);
    uView.display();
    delay(100);
    uView.clear(PAGE);
    uView.setFontType(1); // set font type 0: Numbers and letters. 10 characters per line (6 lines)
    uView.setCursor(12,7);
    uView.print("DONE"); // display string
    uView.display();
    delay(2000);
    uView.clear(PAGE);
    uView.display();
    delay(100);
    uView.setCursor(1,32);
    uView.print("RESTART");
    uView.display();
    }

void PANO360F() {
    digitalWrite(MOTOR_EN_PIN, LOW); // enable drive
    stepper.setStepsPerRevolution(640000); 
    stepper.setSpeedInRevolutionsPerSecond(0.00556); // 2 degree per second, 360 degrees in 3 minutes
    stepper.setAccelerationInRevolutionsPerSecondPerSecond(1.0);
    stepper.setCurrentPositionInSteps(0);
    uView.clear(PAGE);
    uView.display();
    delay(100);
    //uView.setFontType(0);
    //uView.setCursor(7,4);
    //uView.print("SCANNING"); 
    //uView.setFontType(1);
    //uView.setCursor(17,18);
    //uView.print("360");
    uView.setFontType(0);
    uView.setCursor(5,36);
    uView.print("2 DEG/SEC");
    uView.display();
    delay(1000);
    uView.clear(PAGE);
    uView.display();
    delay(100);
    uView.setFontType(0);
    uView.setCursor(7,4);
    uView.print("SCANNING"); 
    uView.setFontType(1);
    uView.setCursor(17,18);
    uView.print("360");
    uView.setFontType(0);
    uView.setCursor(5,36);
    uView.print("3 Minutes");
    uView.display();
    digitalWrite(RECORDSTART, LOW); // starts bag record
    delay(1);
    digitalWrite(RECORDSTART, HIGH);
    //delay(200);  
    stepper.moveToPositionInRevolutions(1.05); // rotates 378 degrees
    digitalWrite(RECORDSTOP, LOW); // stops bag record
    delay(10);
    digitalWrite(RECORDSTOP, HIGH);
    delay(1000);
    stepper.setSpeedInRevolutionsPerSecond(0.1);
    stepper.moveToPositionInRevolutions(1); // back to start
    delay(100);
    digitalWrite(MOTOR_EN_PIN, HIGH); // disable driver
    uView.clear(PAGE);
    uView.display();
    delay(100);
    uView.clear(PAGE);
    uView.setFontType(1); // set font type 0: Numbers and letters. 10 characters per line (6 lines)
    uView.setCursor(12,7);
    uView.print("DONE"); // display string
    uView.display();
    delay(2000);
    uView.clear(PAGE);
    uView.display();
    delay(100);
    uView.setCursor(1,32);
    uView.print("RESTART");
    uView.display();
    }

void PANO180S() {
    digitalWrite(MOTOR_EN_PIN, LOW); // enable drive
    stepper.setStepsPerRevolution(640000); 
    stepper.setSpeedInRevolutionsPerSecond(0.0027778); // 1 degree per second, 360 degrees in 6 minutes
    //stepper.setSpeedInRevolutionsPerSecond(0.01667); // 6 degree per second, 360 degrees in 1 minutes
    //stepper.setSpeedInRevolutionsPerSecond(0.33333); // 20rpm
    stepper.setAccelerationInRevolutionsPerSecondPerSecond(1.0);
    stepper.setCurrentPositionInSteps(0);
    uView.clear(PAGE);
    uView.display();
    delay(100);
    //uView.setFontType(0);
    //uView.setCursor(7,4);
    //uView.print("SCANNING"); 
    //uView.setFontType(1);
    //uView.setCursor(17,18);
    //uView.print("180");
    uView.setFontType(0);
    uView.setCursor(5,36);
    uView.print("1 DEG/SEC");
    uView.display();
    delay(1000);
    uView.clear(PAGE);
    uView.display();
    delay(100);
    uView.setFontType(0);
    uView.setCursor(7,4);
    uView.print("SCANNING"); 
    uView.setFontType(1);
    uView.setCursor(17,18);
    uView.print("180");
    uView.setFontType(0);
    uView.setCursor(5,36);
    uView.print("3 Minutes");
    uView.display();
    digitalWrite(RECORDSTART, LOW); // starts bag record
    delay(1);
    digitalWrite(RECORDSTART, HIGH);
    //delay(500);    
    //stepper.moveToPositionInRevolutions(200); // runs 10 min at 20rpm    
    stepper.moveToPositionInRevolutions(0.53); // rotates 190.8 degrees adds 10.8 degrees overlap
    //stepper.moveToPositionInRevolutions(1.05); // rotates 378 degrees. Add some rotation to be sure and record a full 360 after TCPDUMP starts
    digitalWrite(RECORDSTOP, LOW); // stops bag record
    delay(10);
    digitalWrite(RECORDSTOP, HIGH);
    delay(2000);
    stepper.setSpeedInRevolutionsPerSecond(0.1);
    stepper.moveToPositionInRevolutions(0); // back to start use this line when doing 180 degrees
    //stepper.moveToPositionInRevolutions(1); // back to start use this line when doing 360 degrees
    delay(1000);
    digitalWrite(MOTOR_EN_PIN, HIGH); // disable driver
    uView.clear(PAGE);
    uView.display();
    delay(100);
    uView.clear(PAGE);
    uView.setFontType(1); // set font type 0: Numbers and letters. 10 characters per line (6 lines)
    uView.setCursor(12,7);
    uView.print("DONE"); // display string
    uView.display();
    delay(2000);
    uView.clear(PAGE);
    uView.display();
    delay(100);
    uView.setCursor(1,32);
    uView.print("RESTART");
    uView.display();
    }

void KILL_SCAN() {
    uView.clear(PAGE);
    uView.display();
    delay(100);
    uView.clear(PAGE);
    uView.setFontType(1); // set font type 0: Numbers and letters. 10 characters per line (6 lines)
    uView.setCursor(1,7);
    uView.print("STOPPED"); // display string
    uView.display();
    delay(500);
    uView.setFontType(0);
    uView.setCursor(18,24);
    uView.print("PRESS");
    uView.setCursor(18,36);
    uView.print("RESET");
    uView.display();
    digitalWrite(RECORDSTOP, LOW); // stops bag record
    delay(10);
    digitalWrite(RECORDSTOP, HIGH);
    digitalWrite(MOTOR_EN_PIN, HIGH); // disable drive    
    }    

    
