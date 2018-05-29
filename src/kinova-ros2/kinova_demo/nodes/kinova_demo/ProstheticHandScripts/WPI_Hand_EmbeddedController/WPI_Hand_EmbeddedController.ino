/* 
Who? Selim
What? WPI Hand control code. Written for Arduino Due.
When? 01.25.2018
*/

// SPI lib
#include <SPI.h>

// Queue library used for handling integral component
#include <FriendlyQueue.h>

// IMU libs
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// IMU Object
Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensors_event_t event; 

double euler_x = 0;
double euler_y = 0;
double euler_z = 0;

// Waypoint matrix
const int waypointLength = 2;
const int actuatedJointLength = 7;
int waypointIndex = 0;

// Standard WP matrix for testing joint PIDs
float wayPointMatrix[waypointLength][actuatedJointLength] = {

  // Open Pos
  {150.0,  240.0, 120.0, 15.0, 15.0, 15.0, 15.0    },

  // Close Pos
  {150.0,  240.0, 120.0, 65.0, 15.0, 15.0, 65.0  },
 
};

// WP matrix for showing palm kinematic workspace.
const int waypointPalmWorkSpaceLength = 4;
float wayPointMatrixPalmWorkSpace[waypointPalmWorkSpaceLength][actuatedJointLength] = {

  // Open Pos
  {150.0,  240.0, 140.0, 15.0, 15.0, 15.0, 15.0    },

  // Touch Index
  {150.0,  240.0, 120.0, 65.0, 65.0, 15.0, 15.0  },

  // Touch Middle
  {150.0,  240.0, 100.0, 85.0, 15.0, 85.0, 15.0  },

  // Touch Ring
  {150.0,  240.0, 80.0, 85.0, 15.0, 15.0, 85.0  },    
 
};

// Two WPs that gets triggered with a push button
float wayPointMatrixPushButton[waypointLength][actuatedJointLength] = {

  // Open Pos
  {150.0,  90.0, 90.0, 15.0, 15.0, 15.0, 15.0  },

  // Close Pos
  {150.0,  90.0, 90.0, 60.0, 50.0, 55.0, 15.0  },
 
};

// Pi definition
const float pi = 3.14159265359;

// Motor IDs used in PID and driver functions.
const int motorID_WristA = 0;
const int motorID_WristB = 1;
const int motorID_ThumbA = 2;
const int motorID_ThumbB = 3;
const int motorID_Index = 4;
const int motorID_Middle = 5;
const int motorID_Ring = 6;

// Pin definitions
// Potentiometer Pins
int pin_PotWristA = A0; // First joint from forearm on kinematic chain
int pin_PotWristB = A1; // Second joint from forearm on kinematic chain
int pin_ThumbA = A2;
int pin_ThumbB = A3;
int pin_Index = A7;
int pin_Middle = A6;
int pin_Ring = A4;
int pin_Little = A5;

// Push button pin
int pin_PushButton = 53;

// Motor Driver Pins
int pin_WristA_IN1 = 8;
int pin_WristA_IN2 = 9;
int pin_WristA_D1 = 34;
int pin_WristA_D2 = 36;
int pin_WristB_IN1 = 10;
int pin_WristB_IN2 = 11;
int pin_WristB_D1 = 38;
int pin_WristB_D2 = 40;
int pin_Wrist_EN = 32;

int pin_ThumbA_IN = 24;
int pin_ThumbA_EN = 2;
int pin_ThumbB_IN = 22;
int pin_ThumbB_EN = 3;
int pin_Thumb_Mode = 26;

int pin_Index_IN = 25;
int pin_Index_EN = 4;
int pin_Middle_IN = 23;
int pin_Middle_EN = 5;
int pin_IndexMiddle_Mode = 27;

int pin_Ring_IN = 28;
int pin_Ring_EN = 6;
int pin_Ring_Mode = 30;

// SPI Pins
int PinMOSI = 75;
int PinMISO = 74;
int PinSCK = 76;
int PinSS = 52;

// Control Vairables
// Min/Max angle values frmo potentiometers
float WristA_Min = 300.0;   // [ADC, a.u.]
float WristA_Max = 900.0;   // [ADC, a.u.]
float WristB_Min = 618.0;   // [ADC, a.u.]
float WristB_Max = 834.0;   // [ADC, a.u.]
float ThumbA_Min = 260.0;   // [ADC, a.u.]
float ThumbA_Max = 825.0;   // [ADC, a.u.]
float ThumbB_Min = 320.0;   // [ADC, a.u.]
float ThumbB_Max = 714.0;   // [ADC, a.u.]
float Index_Min = 40.0;   // [ADC, a.u.]
float Index_Max = 422.0;   // [ADC, a.u.]
float Middle_Min = 525.0;   // [ADC, a.u.]
float Middle_Max = 870.0;   // [ADC, a.u.]
float Ring_Min = 390.0;   // [ADC, a.u.]
float Ring_Max = 735.0;   // [ADC, a.u.]

// Angles
float angleWristA = 0.0;        // [deg]
float angleWristB = 0.0;        // [deg]
float angleThumbA = 0.0;        // [deg]
float angleThumbB = 0.0;        // [deg]
float angleIndex = 0.0;         // [deg]
float angleMiddle = 0.0;        // [deg]
float angleRing = 0.0;          // [deg]
float angleLittle = 0.0;        // [deg]

// Neutral Angles. These are subject to change based on pot zero configuration.
float angleWristB_Neutral = 245.0;
float angleWristA_Neutral = 150.0;

// Max and Min angle limits
// Wrist A is especially important because wires get destroyed if it malfunctions.
float angleWristA_Max = 200.0;
float angleWristA_Min = 100.0;
float angleWristB_Max = 204.0;
float angleWristB_Min = -32.0;

// Ref Angles
float angleRefWristA = 0.0;        // [deg]
float angleRefWristB = 0.0;        // [deg]
float angleRefThumbA = 0.0;        // [deg]
float angleRefThumbB = 0.0;        // [deg]
float angleRefIndex = 0.0;         // [deg]
float angleRefMiddle = 0.0;        // [deg]
float angleRefRing = 0.0;          // [deg]

// Angular speed
float angleSpeedWristA = 0.0;        // [deg/s]
float angleSpeedWristB = 0.0;        // [deg/s]
float angleSpeedThumbA = 0.0;        // [deg/s]
float angleSpeedThumbB = 0.0;        // [deg/s]
float angleSpeedIndex = 0.0;         // [deg/s]
float angleSpeedMiddle = 0.0;        // [deg/s]
float angleSpeedRing = 0.0;          // [deg/s]

// Angle previous values. Used for speed calculations
float anglePrevWristA = 0.0;        // [deg]
float anglePrevWristB = 0.0;        // [deg]
float anglePrevThumbA = 0.0;        // [deg]
float anglePrevThumbB = 0.0;        // [deg]
float anglePrevIndex = 0.0;         // [deg]
float anglePrevMiddle = 0.0;        // [deg]
float anglePrevRing = 0.0;          // [deg]

// PWM values
int PWMWristA = 0.0;        // [0-255, a.u.]
int PWMWristB = 0.0;        // [0-255,a.u.]
int PWMThumbA = 0.0;        // [0-255,a.u.]
int PWMThumbB = 0.0;        // [0-255,a.u.]
int PWMIndex = 0.0;         // [0-255,a.u.]
int PWMMiddle = 0.0;        // [0-255,a.u.]
int PWMRing = 0.0;          // [0-255,a.u.]

// Push button value
int pushButtonValue = 1;
int dummyPushButtonValue = 1;

// Sine tracking variables
float sineMagnitude = 20;       // [deg]
float sineOffset = 65;          // [deg]
unsigned int sinePeriod = 500;  // [ms]

// Debug parameters
uint32_t count_miliSec = 0;
uint32_t controlRate = 0;

// Interrupt frequencies
int FREQ_100Hz = 1;  // Loop: Force sensing and debug
int FREQ_1kHz = 1000; // Loop: Control

int controlFlag = 0;

// Force sensor variables

// Buffers to read/write MLX90363
char readBuffer[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
char writeBuffer[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};    

// Error bits, CRC, virtual gain and rolling counter variables
char errorBits = 0;
char rollingCounter = 0;
char CRC_Value = 0;

// Bx,By,Bz variables
int16_t Bx = 0;
int16_t By = 0;
int16_t Bz = 0;   

//Define and initialize CRC array, 256 bytes
char CRCArray[] = {
0x00, 0x2F, 0x5E, 0x71, 0xBC, 0x93, 0xE2, 0xCD, 0x57, 0x78, 0x09, 0x26,
0xEB, 0xC4, 0xB5, 0x9A, 0xAE, 0x81, 0xF0, 0xDF, 0x12, 0x3D, 0x4C, 0x63,
0xF9, 0xD6, 0xA7, 0x88, 0x45, 0x6A, 0x1B, 0x34, 0x73, 0x5C, 0x2D, 0x02,
0xCF, 0xE0, 0x91, 0xBE, 0x24, 0x0B, 0x7A, 0x55, 0x98, 0xB7, 0xC6, 0xE9,
0xDD, 0xF2, 0x83, 0xAC, 0x61, 0x4E, 0x3F, 0x10, 0x8A, 0xA5, 0xD4, 0xFB,
0x36, 0x19, 0x68, 0x47, 0xE6, 0xC9, 0xB8, 0x97, 0x5A, 0x75, 0x04, 0x2B,
0xB1, 0x9E, 0xEF, 0xC0, 0x0D, 0x22, 0x53, 0x7C, 0x48, 0x67, 0x16, 0x39,
0xF4, 0xDB, 0xAA, 0x85, 0x1F, 0x30, 0x41, 0x6E, 0xA3, 0x8C, 0xFD, 0xD2,
0x95, 0xBA, 0xCB, 0xE4, 0x29, 0x06, 0x77, 0x58, 0xC2, 0xED, 0x9C, 0xB3,
0x7E, 0x51, 0x20, 0x0F, 0x3B, 0x14, 0x65, 0x4A, 0x87, 0xA8, 0xD9, 0xF6,
0x6C, 0x43, 0x32, 0x1D, 0xD0, 0xFF, 0x8E, 0xA1, 0xE3, 0xCC, 0xBD, 0x92,
0x5F, 0x70, 0x01, 0x2E, 0xB4, 0x9B, 0xEA, 0xC5, 0x08, 0x27, 0x56, 0x79,
0x4D, 0x62, 0x13, 0x3C, 0xF1, 0xDE, 0xAF, 0x80, 0x1A, 0x35, 0x44, 0x6B,
0xA6, 0x89, 0xF8, 0xD7, 0x90, 0xBF, 0xCE, 0xE1, 0x2C, 0x03, 0x72, 0x5D,
0xC7, 0xE8, 0x99, 0xB6, 0x7B, 0x54, 0x25, 0x0A, 0x3E, 0x11, 0x60, 0x4F,
0x82, 0xAD, 0xDC, 0xF3, 0x69, 0x46, 0x37, 0x18, 0xD5, 0xFA, 0x8B, 0xA4,
0x05, 0x2A, 0x5B, 0x74, 0xB9, 0x96, 0xE7, 0xC8, 0x52, 0x7D, 0x0C, 0x23,
0xEE, 0xC1, 0xB0, 0x9F, 0xAB, 0x84, 0xF5, 0xDA, 0x17, 0x38, 0x49, 0x66,
0xFC, 0xD3, 0xA2, 0x8D, 0x40, 0x6F, 0x1E, 0x31, 0x76, 0x59, 0x28, 0x07,
0xCA, 0xE5, 0x94, 0xBB, 0x21, 0x0E, 0x7F, 0x50, 0x9D, 0xB2, 0xC3, 0xEC,
0xD8, 0xF7, 0x86, 0xA9, 0x64, 0x4B, 0x3A, 0x15, 0x8F, 0xA0, 0xD1, 0xFE,
0x33, 0x1C, 0x6D, 0x42 };


// Interrupt routine initialization
void StartTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {

  //Enable or disable write protect of PMC registers.
  pmc_set_writeprotect(false);
  //Enable the specified peripheral clock.
  pmc_enable_periph_clk((uint32_t)irq);

  // TC_CMR_WAVE: Provides wave generation. In pg 862.
  // TC_CMR_TCCLKS_TIMER_CLOCK4: In datasheet pg 881 CLOCK4 is MCK/128
  // TC_CMR_WAVSEL_UP_RC: Value of TC_CV is incremented from 0 to RC. In pg 870.
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
  uint32_t rc = VARIANT_MCK / 128 / frequency;

  // These values should be setting frequency.
  TC_SetRA(tc, channel, rc / 2);
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);

  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;

  // Enable interrupt service routine related to irq.
  NVIC_EnableIRQ(irq);
}

// 1 Hz debug loop
// Use at other frequencies too if you need to debug something else.
void TC3_Handler() {
  //Serial.println(controlRate);
  controlRate = 0;
  
  //Serial.println(digitalRead(pin_PushButton));
  TC_GetStatus(TC1, 0);
}

// 1000 Hz Control Loop
// Don't mess with this. Keep it 1 kHz all time.
void TC4_Handler() {
  TC_GetStatus(TC1, 1);
  count_miliSec++;

  controlFlag = 1;

    ////////////////////////////////
    //// CONTROL CODE GOES HERE \\\\ 
  
    // Compute angle and its derivative
    angleWristA = ReadPotentiometer(pin_PotWristA, motorID_WristA);
    angleWristB = ReadPotentiometer(pin_PotWristB, motorID_WristB);
    angleThumbA = ReadPotentiometer(pin_ThumbA, motorID_ThumbA);
    angleThumbB = ReadPotentiometer(pin_ThumbB, motorID_ThumbB);
    angleIndex = ReadPotentiometer(pin_Index, motorID_Index);
    angleMiddle = ReadPotentiometer(pin_Middle, motorID_Middle);
    angleRing = ReadPotentiometer(pin_Ring, motorID_Ring);
    //angleLittle = ReadPotentiometer(pin_Little, motorID_);
  
    angleSpeedWristA = (angleWristA-anglePrevWristA)/0.001;
    angleSpeedWristB = (angleWristB-anglePrevWristB)/0.001;
    angleSpeedThumbA = (angleThumbA-anglePrevThumbA)/0.001;
    angleSpeedThumbB = (angleThumbB-anglePrevThumbB)/0.001;
    angleSpeedIndex = (angleIndex-anglePrevIndex)/0.001;
    angleSpeedMiddle = (angleMiddle-anglePrevMiddle)/0.001;
    angleSpeedRing = (angleRing-anglePrevRing)/0.001;


  
    // Do force sensor and push button reading at 1/10 of the control rate
    if(count_miliSec%10 == 0)
    {
    Serial.print(Bx); 
    Serial.print(" ");
    Serial.print(By);
    Serial.print(" ");
    Serial.print(Bz);
    Serial.println(""); 

    /*Serial.print(angleWristA);
    Serial.print(" ");
    Serial.print(angleWristB);
    Serial.print(" ");
    Serial.print(angleThumbA);
    Serial.print(" ");
    Serial.print(angleThumbB);
    Serial.print(" ");
    Serial.print(angleIndex); // Index
    Serial.print(" ");
    Serial.print(angleMiddle); // Middle
    Serial.print(" ");
    Serial.println(angleRing); // Ring*/
    //Serial.print(" ");
    //Serial.println(angleWristA);  // Ring or not used
    //Serial.print(" ");
    //Serial.println(angleWristB);

    /*Serial.print(angleIndex); // Index
    Serial.print(" ");
    Serial.print(angleRefIndex); // Middle
    Serial.println(" ");*/
        
      // Get forces
      ReadForceSensor();
    
      // Read push button
      // Don't use the following line when connected to python. Selim. 2018/05.
      /*dummyPushButtonValue = digitalRead(pin_PushButton);

      if(dummyPushButtonValue != pushButtonValue)
      {
        // Only set flag at down edge
        if(dummyPushButtonValue == 0)
        {
          waypointIndex += 1;
          waypointIndex %= 2;
        }
      }
      pushButtonValue = dummyPushButtonValue;*/
      
      angleRefWristA = wayPointMatrixPushButton[waypointIndex][0];
      angleRefWristB = wayPointMatrixPushButton[waypointIndex][1];
      angleRefThumbA = wayPointMatrixPushButton[waypointIndex][2];
      angleRefThumbB = wayPointMatrixPushButton[waypointIndex][3];
      angleRefIndex = wayPointMatrixPushButton[waypointIndex][4];
      angleRefMiddle = wayPointMatrixPushButton[waypointIndex][5];
      angleRefRing = wayPointMatrixPushButton[waypointIndex][6]; 
      
    }

    // Do IMU Read at 1/50 of the control rate
    //if(count_miliSec%50 == 0)
    //{
      // put your main code here, to run repeatedly:
      /* Get a new sensor event */ 
      
      //bno.getEvent(&event);

      /*imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
 
      euler_x = euler.z();
      euler_z = euler.x()+150.0;
      euler_y = euler.y()+90.0;
      if(euler_z > 360.0){euler_z = (int)euler_z % 360;}
      if(euler_y > 360.0){euler_y = (int)euler_y % 360;}

      angleRefWristA = 150.0 + (euler_z - 150.0);
      angleRefWristB = 90.0 - (euler_y - 90.0);*/

      /*Serial.print(angleWristB);
      Serial.print(" ");
      Serial.print(euler_y);
      Serial.print(" ");
      Serial.println(angleRefWristB);   */   
      //Serial.print(" ");
      //Serial.print(PWMWristB);
      //Serial.print(" ");
      //serial.println(angleWristB);

      /* Display the floating point data */
      /*Serial.print("X: ");
      Serial.print(euler_x, 4);
      Serial.print("\tY: ");
      Serial.print(euler_y, 4);
      Serial.print("\tZ: ");
      Serial.print(euler_z, 4);
      Serial.println(" ");*/
      
    //}
    
    // Compute control action and drive motor
    //angleRefThumbA = SignalGenerator_Sine(sineMagnitude,sineOffset,sinePeriod);
  
    // Stays at waypoints for "delta_t" ms.
    /*int delta_t = 1000waypointIndex;
    if(count_miliSec % delta_t == 0)
    {
      waypointIndex++;
      waypointIndex%=waypointPalmWorkSpaceLength;
    }
    angleRefWristA = wayPointMatrixPalmWorkSpace[waypointIndex][0];
    angleRefWristB = wayPointMatrixPalmWorkSpace[waypointIndex][1];
    angleRefThumbA = wayPointMatrixPalmWorkSpace[waypointIndex][2];
    angleRefThumbB = wayPointMatrixPalmWorkSpace[waypointIndex][3];
    angleRefIndex = wayPointMatrixPalmWorkSpace[waypointIndex][4];
    angleRefMiddle = wayPointMatrixPalmWorkSpace[waypointIndex][5];
    angleRefRing = wayPointMatrixPalmWorkSpace[waypointIndex][6];  */


    // IMU based stabilization
    //angleRefWristA = event.orientation.x - angleWristA;
  
    PWMWristA = PIDControl(angleWristA, angleSpeedWristA, angleRefWristA, motorID_WristA);
    DriveMotor_33926(PWMWristA, motorID_WristA);
  
    //PWMWristB = PIDControl(angleWristB, angleSpeedWristB, angleRefWristB, motorID_WristB);
    //DriveMotor_33926(PWMWristB, motorID_WristB);  
  
    PWMThumbA = -PIDControl(angleThumbA, angleSpeedThumbA, angleRefThumbA, motorID_ThumbA);
    DriveMotor_8835(PWMThumbA, motorID_ThumbA);
    
    PWMThumbB = PIDControl(angleThumbB, angleSpeedThumbB, angleRefThumbB, motorID_ThumbB);
    DriveMotor_8835(PWMThumbB, motorID_ThumbB);
    //DriveMotor_8835(-255, motorID_ThumbB);
  
    PWMIndex = -PIDControl(angleIndex, angleSpeedIndex, angleRefIndex, motorID_Index);
    DriveMotor_8835(PWMIndex, motorID_Index);
    
    PWMMiddle = PIDControl(angleMiddle, angleSpeedMiddle, angleRefMiddle, motorID_Middle);
    DriveMotor_8835(PWMMiddle, motorID_Middle);  
    //DriveMotor_8835(255, motorID_Middle);  
  
    PWMRing = -PIDControl(angleRing, angleSpeedRing, angleRefRing, motorID_Ring);
    DriveMotor_8835(PWMRing, motorID_Ring);   
  
    // Update derivative variables
    anglePrevWristA = angleWristA;
    anglePrevWristB = angleWristB;
    anglePrevThumbA = angleThumbA;
    anglePrevThumbB = angleThumbB;
    anglePrevIndex = angleIndex;
    anglePrevMiddle = angleMiddle;
    anglePrevRing = angleRing;
  
    // Send data back to matlab over fast serial of arduino due.
    //DataAcquisition();
    DataAcquisition_Python();
    
    controlRate++;  
    //// END OF CONTROL CODE \\\\ 
    /////////////////////////////
  
}

// Queue for computing integral action.
int controlPID_IntegralQueueLength = 100; 
FriendlyQueue <float> controlPID_IntegralQueue_WristA(controlPID_IntegralQueueLength); 
FriendlyQueue <float> controlPID_IntegralQueue_WristB(controlPID_IntegralQueueLength); 
FriendlyQueue <float> controlPID_IntegralQueue_ThumbA(controlPID_IntegralQueueLength); 
FriendlyQueue <float> controlPID_IntegralQueue_ThumbB(controlPID_IntegralQueueLength); 
FriendlyQueue <float> controlPID_IntegralQueue_Index(controlPID_IntegralQueueLength); 
FriendlyQueue <float> controlPID_IntegralQueue_Middle(controlPID_IntegralQueueLength); 
FriendlyQueue <float> controlPID_IntegralQueue_Ring(controlPID_IntegralQueueLength); 
// Obvious name. Returns PWM in integer.
int PIDControl(float angle_IN, float angleSpeed_IN, float angleRef_IN, int motorID_IN)
{
  // Integral component calculations drop the controlRate below 1kHz.
  // I omit them at the time begin. Selim. 04/25/2018
  
  // angle_IN         [deg]
  // angleSpeed_IN    [deg/s]
  // angleRef_IN      [deg]

  float kp = 0.0;
  float kd = 0.0;
  float ki = 0.0;

  float error = angleRef_IN - angle_IN;
  float errorDot = 0.0 - angleSpeed_IN;
  float errorIntegral = 0.0;

  switch (motorID_IN) {
    case motorID_WristA:
      kp = 20.0;
      kd = 0.5;
      ki = 0.01;

      // Compute integral term
      // Queue is not full
      /*if(controlPID_IntegralQueue_WristA.IsFull() == 0)
      {
        controlPID_IntegralQueue_WristA.enQueue(error);
      }
      // Queue is full
      if(controlPID_IntegralQueue_WristA.IsFull() == 1)
      {
        // Remove the first in element
        controlPID_IntegralQueue_WristA.deQueue();
        // Add new element to the end
        controlPID_IntegralQueue_WristA.enQueue(error);
      }      
      errorIntegral = controlPID_IntegralQueue_WristA.SumQueue(); */

      // Saturate input ref angle
      if(angleWristA_Max < angleRef_IN)
      {
        angleRef_IN = angleWristA_Max;
      }
      if(angleWristA_Min > angleRef_IN)
      {
        angleRef_IN = angleWristA_Min;
      }
      error = error = angleRef_IN - angle_IN;
        
      break;    
    case motorID_WristB:
      kp = 1000.0;
      kd = 10.00;
      ki = 6.5;

      // Saturate input ref angle
      if(angleWristB_Max < angleRef_IN)
      {
        angleRef_IN = angleWristB_Max;
      }
      if(angleWristB_Min > angleRef_IN)
      {
        angleRef_IN = angleWristB_Min;
      }     
      error = error = angleRef_IN - angle_IN; 

      // Compute integral term
      // Queue is not full
      /*if(controlPID_IntegralQueue_WristB.IsFull() == 0)
      {
        controlPID_IntegralQueue_WristB.enQueue(error);
      }
      // Queue is full
      if(controlPID_IntegralQueue_WristB.IsFull() == 1)
      {
        // Remove the first in element
        controlPID_IntegralQueue_WristB.deQueue();
        // Add new element to the end
        controlPID_IntegralQueue_WristB.enQueue(error);
      }      
      errorIntegral = controlPID_IntegralQueue_WristB.SumQueue(); */
        
      break;        
    case motorID_ThumbA:
      kp = 10.0;
      kd = 0.05;
      ki = 0.03;

      // Compute integral term
      // Queue is not full
      /*if(controlPID_IntegralQueue_ThumbA.IsFull() == 0)
      {
        controlPID_IntegralQueue_ThumbA.enQueue(error);
      }
      // Queue is full
      if(controlPID_IntegralQueue_ThumbA.IsFull() == 1)
      {
        // Remove the first in element
        controlPID_IntegralQueue_ThumbA.deQueue();
        // Add new element to the end
        controlPID_IntegralQueue_ThumbA.enQueue(error);
      }      
      errorIntegral = controlPID_IntegralQueue_ThumbA.SumQueue(); */
        
      break;
    case motorID_ThumbB:
      kp = 20.0;
      kd = 0.05;
      ki = 0.05;

      // Compute integral term
      // Queue is not full
      /*if(controlPID_IntegralQueue_ThumbB.IsFull() == 0)
      {
        controlPID_IntegralQueue_ThumbB.enQueue(error);
      }
      // Queue is full
      if(controlPID_IntegralQueue_ThumbB.IsFull() == 1)
      {
        // Remove the first in element
        controlPID_IntegralQueue_ThumbB.deQueue();
        // Add new element to the end
        controlPID_IntegralQueue_ThumbB.enQueue(error);
      }      
      errorIntegral = controlPID_IntegralQueue_ThumbB.SumQueue(); */
      
      break;
    case motorID_Index:
      kp = 100.0;
      kd = 3.0;
      ki = 0.00;

      // Compute integral term
      // Queue is not full
      /*if(controlPID_IntegralQueue_Index.IsFull() == 0)
      {
        controlPID_IntegralQueue_Index.enQueue(error);
      }
      // Queue is full
      if(controlPID_IntegralQueue_Index.IsFull() == 1)
      {
        // Remove the first in element
        controlPID_IntegralQueue_Index.deQueue();
        // Add new element to the end
        controlPID_IntegralQueue_Index.enQueue(error);
      }      
      errorIntegral = controlPID_IntegralQueue_Index.SumQueue(); */
      
      break;
    case motorID_Middle:
      kp = 15.0;
      kd = 0.05;
      ki = 0.05;

      // Compute integral term
      // Queue is not full
      /*if(controlPID_IntegralQueue_Middle.IsFull() == 0)
      {
        controlPID_IntegralQueue_Middle.enQueue(error);
      }
      // Queue is full
      if(controlPID_IntegralQueue_Middle.IsFull() == 1)
      {
        // Remove the first in element
        controlPID_IntegralQueue_Middle.deQueue();
        // Add new element to the end
        controlPID_IntegralQueue_Middle.enQueue(error);
      }      
      errorIntegral = controlPID_IntegralQueue_ThumbB.SumQueue(); */
      
      break;      
    case motorID_Ring:
      kp = 10.0;
      kd = 0.05;
      ki = 0.05;

      // Compute integral term
      // Queue is not full
      /*if(controlPID_IntegralQueue_Ring.IsFull() == 0)
      {
        controlPID_IntegralQueue_Ring.enQueue(error);
      }
      // Queue is full
      if(controlPID_IntegralQueue_Ring.IsFull() == 1)
      {
        // Remove the first in element
        controlPID_IntegralQueue_Ring.deQueue();
        // Add new element to the end
        controlPID_IntegralQueue_Ring.enQueue(error);
      }      
      errorIntegral = controlPID_IntegralQueue_Ring.SumQueue(); */
      
      break;    

  }   

  return (kp*error + ki*errorIntegral + kd*errorDot);
  //return 0;
}

// Drives motor forward/backward with a given PWM.
// DRV8835
void DriveMotor_8835(int PWM_IN, int motorID_IN)
{
  int pin_MotorIN = 0;
  int pin_MotorEN = 0;
  
  switch (motorID_IN) {
    case motorID_ThumbA:
      pin_MotorIN = pin_ThumbA_IN;
      pin_MotorEN = pin_ThumbA_EN;
      break;
    case motorID_ThumbB:
      pin_MotorIN = pin_ThumbB_IN;
      pin_MotorEN = pin_ThumbB_EN;
      break;
    case motorID_Index:
      pin_MotorIN = pin_Index_IN;
      pin_MotorEN = pin_Index_EN;
      break;
    case motorID_Middle:
      pin_MotorIN = pin_Middle_IN;
      pin_MotorEN = pin_Middle_EN;
      break;      
    case motorID_Ring:
      pin_MotorIN = pin_Ring_IN;
      pin_MotorEN = pin_Ring_EN;
      break;    

  }
  
  // PWM out is the variable passed to driver.
  // PWM_IN processed for saturation and direction.
  int PWM_OUT = PWM_IN;
  
  // Curls the finger. Clockwise.
  if(PWM_IN > 0)
  {
    // Saturate
    if(PWM_IN > 255)
    {
      PWM_IN = 255;
      PWM_OUT = PWM_IN;
    }
    else
    {
      PWM_OUT = PWM_IN;
    }

    // Do motor driver inputs
    analogWrite(pin_MotorEN, PWM_OUT);
    digitalWrite(pin_MotorIN, HIGH);
    
  }

  // Takes the finger to initial position. Counter clockwise.
  if(PWM_IN<0)
  {
    // Saturate
    if(PWM_IN < -255)
    {
      PWM_IN = -255;
      // Switch sign.
      PWM_OUT = -PWM_IN;
    }    
    else
    {
      PWM_OUT = -PWM_IN;
    }

    // Do motor driver inputs
    analogWrite(pin_MotorEN, PWM_OUT);
    digitalWrite(pin_MotorIN, LOW);  
  }
}

// Drives motor forward/backward with a given PWM.
// MC33926
void DriveMotor_33926(int PWM_IN, int motorID_IN)
{
  int pin_MotorIN1 = 0;
  int pin_MotorIN2 = 0;
  int pin_MotorPWM = 0;

  switch (motorID_IN) {
    case motorID_WristA:
      pin_MotorIN1 = pin_WristA_IN1;
      pin_MotorIN2 = pin_WristA_IN2;
      pin_MotorPWM = pin_WristA_D2;
      break;
    case motorID_WristB:
      pin_MotorIN1 = pin_WristB_IN1;
      pin_MotorIN2 = pin_WristB_IN2;
      break;
  }
  
  // PWM out is the variable passed to driver.
  // PWM_IN processed for saturation and direction.
  int PWM_OUT = PWM_IN;
  
  // Curls the finger. Clockwise.
  if(PWM_IN > 0)
  {
    // Saturate
    if(PWM_IN > 255)
    {
      PWM_IN = 255;
      PWM_OUT = PWM_IN;
    }
    else
    {
      PWM_OUT = PWM_IN;
    }

    // Do motor driver inputs
    //analogWrite(pin_MotorPWM,PWM_OUT);
    analogWrite(pin_MotorIN1, PWM_OUT);
    digitalWrite(pin_MotorIN2, LOW);
    
  }

  // Takes the finger to initial position. Counter clockwise.
  if(PWM_IN<0)
  {
    // Saturate
    if(PWM_IN < -255)
    {
      PWM_IN = -255;
      // Switch sign.
      PWM_OUT = -PWM_IN;
    }    
    else
    {
      PWM_OUT = -PWM_IN;
    }

    // Do motor driver inputs
    //analogWrite(pin_MotorPWM,PWM_OUT);
    analogWrite(pin_MotorIN2, PWM_OUT);
    digitalWrite(pin_MotorIN1, LOW);
  }
}

// Returns shaft angle at finger base in deg.
float ReadPotentiometer(int pin_IN, int motorID_IN)
{

  float DAC2DEG = 0.0;
  float DACOFFSET = 0.0;

  switch (motorID_IN) {
    case motorID_WristA:
      DAC2DEG = 120.0/(WristA_Max-WristA_Min);
      DACOFFSET = WristA_Min*DAC2DEG - 138.0; // Last constant is added to match z-axis offset of the IMU.
      break;
    case motorID_WristB:
      DAC2DEG = 140.0/(WristB_Max-WristB_Min);
      DACOFFSET = WristB_Min*DAC2DEG;
      //DAC2DEG = 1;
      //DACOFFSET =0 ; 
      break;      
    case motorID_ThumbA:
      DAC2DEG = 130.0/(ThumbA_Max-ThumbA_Min);
      DACOFFSET = ThumbA_Min*DAC2DEG;
      break;
    case motorID_ThumbB:
      DAC2DEG = 85.0/(ThumbB_Max-ThumbB_Min);
      DACOFFSET = ThumbB_Min*DAC2DEG;
      //DAC2DEG = 1;
      //DACOFFSET =0 ;       
      break;
    case motorID_Index:
      DAC2DEG = -85.0/(Index_Max-Index_Min);
      DACOFFSET = Index_Min*DAC2DEG - 90.0;  
      break;
    case motorID_Middle:
      DAC2DEG = -85.0/(Middle_Max-Middle_Min);
      DACOFFSET = Middle_Min*DAC2DEG - 90.0;
      break;      
    case motorID_Ring:
      DAC2DEG = -85.0/(Ring_Max-Ring_Min);
      DACOFFSET = Ring_Min*DAC2DEG - 90.0;         
      break;    

  }
  
  // Finger Calibration
  // 201 -> 55 deg
  // 467 -> 0 deg  
  // DAC2DEG = 0.2068 = (55-0)/(467-201)

  return (DAC2DEG*float(analogRead(pin_IN))-DACOFFSET);
  //return analogRead(pin_IN);
}

// Generates a sine wave with a certain period.
float SignalGenerator_Sine(float magnitude_IN, float offset_IN, unsigned int period_IN){
  return magnitude_IN*sin(2*pi*count_miliSec/period_IN)+offset_IN;
}

void DataAcquisition_Python()
{
  /*
  int packageLength = 1;
  byte dataPackage[packageLength];
  dataPackage[0] = 13;
  SerialUSB.write(dataPackage, packageLength);
  */
  int input = SerialUSB.read();

  // Received open command from python. ASCII 'a' = 97 ASCII 'b' = 98
  if(input == 97)
  {
    waypointIndex = 1;
  }
  if(input == 98)
  {
    waypointIndex = 0;
  }
  SerialUSB.print(controlRate);
  SerialUSB.print(" ");
  SerialUSB.print(angleThumbB); 
  SerialUSB.print(" ");
  SerialUSB.print(angleRefThumbB);
  SerialUSB.print(" ");
  SerialUSB.print(angleIndex);
  SerialUSB.print(" ");
  SerialUSB.print(angleRefIndex);  
  SerialUSB.print(" ");
  SerialUSB.print(Bx);  
  SerialUSB.print(" ");
  SerialUSB.print(By);  
  SerialUSB.print(" ");
  SerialUSB.print(Bz);   
  SerialUSB.print(" ");
  SerialUSB.print(waypointIndex);     
  SerialUSB.println(""); 
}

void DataAcquisition()
{
  int packageLength = 16;
  byte dataPackage[packageLength]; 
  
  // Begin condition
  dataPackage[0] = 24;

  // Data to be transferred     
  dataPackage[1] = (int)(angleThumbB*100) >> 8;
  dataPackage[2] = (int)(angleThumbB*100);
  dataPackage[3] = (int)(angleIndex*100) >> 8;
  dataPackage[4] = (int)(angleIndex*100);      
  dataPackage[5] = (int)(angleRefIndex*100) >> 8;
  dataPackage[6] = (int)(angleRefIndex*100);
  dataPackage[7] = (int)(angleRefThumbB*100) >> 8;
  dataPackage[8] = (int)(angleRefThumbB*100);
  dataPackage[9] = Bx >> 8;
  dataPackage[10] = Bx;  
  dataPackage[11] = By >> 8;
  dataPackage[12] = By;  
  dataPackage[13] = Bz >> 8;
  dataPackage[14] = Bz;      

  // End condition
  dataPackage[15] = 129;
  //Serial.write(dataPackage);
}

// CRC function. Used for reading MLX90363
char ComputeCRC(char Byte0, char Byte1, char Byte2, char Byte3, char Byte4, char Byte5, char Byte6){
  char CRC_value = 0xFF;
  CRC_value = CRCArray[CRC_value ^ Byte0];
  CRC_value = CRCArray[CRC_value ^ Byte1];
  CRC_value = CRCArray[CRC_value ^ Byte2];
  CRC_value = CRCArray[CRC_value ^ Byte3];
  CRC_value = CRCArray[CRC_value ^ Byte4];
  CRC_value = CRCArray[CRC_value ^ Byte5];
  CRC_value = CRCArray[CRC_value ^ Byte6];
  CRC_value = ~CRC_value;
  return CRC_value;
}



// Reads MLX90363 sensor over SPI
void ReadForceSensor()
{
    
  
    // Create a GET1 message. Format of messages are explained in both DataSheet 
    // [DS] and GSG. 
    writeBuffer[0] = 0x00;
    writeBuffer[1] = 0x00;
    writeBuffer[2] = 0xFF; // Timeout value is set as 65 ms
    writeBuffer[3] = 0xFF; // Timeout value is set as 65 ms
    writeBuffer[4] = 0x00;
    writeBuffer[5] = 0x00;
    writeBuffer[6] = 0x93; // Marker is set as 2 to get XYZ measurement. OP Code for GET1 message: 19 in Decimal. 
    writeBuffer[7] = ComputeCRC(0x00,0x00,0xFF,0xFF,0x00,0x00,0x93); // CRC
    
    // Transfer the content of writeBuffer to MLX90363. 
    errorBits = 0;
    int loopCounter = 0;

    //csForce = 0;
    
    // Begin transaction
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    digitalWrite(PinSS,LOW);
    
    for (int i=0; i<8; i++){
      //wait_us(1);
      delayMicroseconds(1);
      readBuffer[i] = SPI.transfer(writeBuffer[i]);
      
    }
    //csForce = 1;
    //delayMicroseconds(1500);
    digitalWrite(PinSS,HIGH);

    // End transaction
    SPI.endTransaction();
    
    errorBits = readBuffer[0]>>6;

    Bx = (readBuffer[1] & 0x3F) << 8;
    Bx += readBuffer[0];
    if(Bx>= 8192){
      Bx -= 16384;
    }
    By = (readBuffer[3] & 0x3F) << 8;
    By += readBuffer[2]; 
    if(By>= 8192){
      By -= 16384;
    }
        
    Bz = (readBuffer[5] & 0x3F) << 8;
    Bz += readBuffer[4]; 
    if(Bz>= 8192){
      Bz -= 16384;
    }
    
    Bx += 0;
    By += 180;
    Bz += -715;
    
    // Extract error bits E0 and E1, CRC and rolling counter.
    errorBits = readBuffer[0]>>6;
    CRC_Value = readBuffer[7];
    rollingCounter = readBuffer[6] & 0x3F;    
    
    //pc.printf("%d %\n", Bz);
    //pc.printf("Error bits %d ",errorBits);  
    //pc.printf("m: "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(CRC_Value));

    //return 0;
}
// Initialize stuff here
void setup() {
  // Start serial comm
  Serial.begin(115200);
  SerialUSB.begin(4000000); 
  
  // Init IMU
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
  Wire.setClock( 400000 );
  
  // Init push button
  pinMode(pin_PushButton, INPUT);
  digitalWrite(pin_PushButton, HIGH);
  
  // Initialize potentiometer pins as input
  pinMode(pin_PotWristA, INPUT);
  pinMode(pin_PotWristB, INPUT);
  pinMode(pin_ThumbA, INPUT);
  pinMode(pin_ThumbB, INPUT);
  pinMode(pin_Index, INPUT);
  pinMode(pin_Middle, INPUT);
  pinMode(pin_Ring, INPUT);

  pinMode(pin_WristA_IN1, OUTPUT);  
  pinMode(pin_WristA_IN2, OUTPUT);
  pinMode(pin_WristA_D1, OUTPUT);
  pinMode(pin_WristA_D2, OUTPUT);
  pinMode(pin_Wrist_EN, OUTPUT); 

  pinMode(pin_WristB_IN1, OUTPUT);  
  pinMode(pin_WristB_IN2, OUTPUT);
  pinMode(pin_WristB_D1, OUTPUT);
  pinMode(pin_WristB_D2, OUTPUT);

  pinMode(pin_ThumbA_IN, OUTPUT);  
  pinMode(pin_ThumbA_EN, OUTPUT);
  pinMode(pin_ThumbB_IN, OUTPUT);  
  pinMode(pin_ThumbB_EN, OUTPUT);  
  pinMode(pin_Thumb_Mode, OUTPUT);  
  
  pinMode(pin_Index_IN, OUTPUT);  
  pinMode(pin_Index_EN, OUTPUT);
  pinMode(pin_Middle_IN, OUTPUT);  
  pinMode(pin_Middle_EN, OUTPUT);  
  pinMode(pin_IndexMiddle_Mode, OUTPUT);  

  pinMode(pin_Ring_IN, OUTPUT);  
  pinMode(pin_Ring_EN, OUTPUT);  
  pinMode(pin_Ring_Mode, OUTPUT);    

  digitalWrite(pin_WristA_IN1, LOW);  
  digitalWrite(pin_WristA_IN2, LOW);
  digitalWrite(pin_WristA_D1, LOW);
  digitalWrite(pin_WristA_D2, HIGH);  
  digitalWrite(pin_Wrist_EN, HIGH); 

  digitalWrite(pin_WristB_IN1, LOW);  
  digitalWrite(pin_WristB_IN2, LOW);
  digitalWrite(pin_WristB_D1, LOW);
  digitalWrite(pin_WristB_D2, HIGH);  
  
  digitalWrite(pin_ThumbA_IN,LOW);
  digitalWrite(pin_ThumbA_EN,LOW);
  digitalWrite(pin_ThumbB_IN,LOW);
  digitalWrite(pin_ThumbB_EN,LOW);
  digitalWrite(pin_Thumb_Mode,HIGH);

  digitalWrite(pin_Index_IN,LOW);
  digitalWrite(pin_Index_EN,LOW);
  digitalWrite(pin_Middle_IN,LOW);
  digitalWrite(pin_Middle_EN,LOW);
  digitalWrite(pin_IndexMiddle_Mode,HIGH);  

  digitalWrite(pin_Ring_IN,LOW);
  digitalWrite(pin_Ring_EN,LOW);
  digitalWrite(pin_Ring_Mode,HIGH);   

  //DriveMotor_8835(255,motorID_ThumbB);
  //DriveMotor_33926(255, motorID_WristB);
  
  // Setup interrupts
  StartTimer(TC1, 0, TC3_IRQn, FREQ_100Hz);
  StartTimer(TC1, 1, TC4_IRQn, FREQ_1kHz);

  // Mark comm pins as output or input
  pinMode (PinMOSI,OUTPUT);
  pinMode (PinMISO,INPUT);
  pinMode (PinSCK,OUTPUT);
  
  // Make the MLX90363 sensor the active slave device
  pinMode (PinSS, OUTPUT);
  digitalWrite(PinSS,HIGH);

  // Required SPI confugration to communicate with MLX90363
  // Details of SPI settings can be found in "Getting Started 
  // Guide" [GSG], under "SPI bus protocol".
  SPI.begin(); 

}


void loop() {

  if(controlFlag == 1)
  {
    controlFlag = 0;
    
  }

}
