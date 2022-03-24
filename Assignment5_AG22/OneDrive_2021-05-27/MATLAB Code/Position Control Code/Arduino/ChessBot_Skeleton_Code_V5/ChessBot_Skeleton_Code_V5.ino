
// ChessBot_Skeleton_Code.ino
// --------------------------
// Licenting Information: You are free to use or extend this project
// for educational purposes provided that (1) you do not distribute or
// publish solutions, (2) you retain this notice, and (3) you provide 
// clear attribution to the University of Melbourne, Department of 
// Mechanical Engineering.

// Attribution Information: The ChessBot project was developed at the
// University of Melbourne. The core project was primarily developed
// by Professor Denny Oetomo (doetomo@unimelb.edu.au). The ChessBot 
// Skeleton Code was developed by Nathan Batham 
// (nathan.batham@unimelb.edu.au)


// ChessBot_Skeleton_Code.ino provides core functions, variables and 
// operations to interface directly with FeeTech SCS15 and SCS009 servo 
// motors. Students should interface with the motors using the provided
// functions. This file is divided into 4 major sections
//        (i)   Arduino Setup - This contains setup code that runs
//              once to establish variables and serial connections.
//              It is divided into two parts; 1) System Setup - where
//              key serial setup is done to establish connects with
//              MATLAB, and 2) Student Setup - where students can
//              add their own routines to execute on system setup.
//
//        (ii)  Arduino Run - This contains code that will run indefinitely
//              while the Arduino is powered on. This is again broken
//              up into two parts; 1) System Run - where key opperations
//              for communicating directly with hardware are executed,
//              and Student Run - where additional student code can
//              be added to expand functionality. Note: Some System Run
//              variables and functions may need modification to expand
//              certain functionality.
//
//        (iii) Student Functions - This is where students should add
//              their own functions when expanding the functionality of
//              this code.
//
//        (iv)  System Functions - This section contains all of the 
//              lower-level functions used for communicating with 
//              the specific hardware and setting up the serial connections.


// NOTES FOR STUDENTS
//    The FeeTech motors opperate on their own I/O format. 0-1023 corresponds 
//    to negative rotation velocity between 0 rpm and the motor's max RPM.
//    1024-2048 corresponds to positive rotation velocity between 0 rpm 
//    and the motor's max RPM.


// BUG REPORTING
// If you believe there may be a bug in the code, please report it using the 
// subject discussion board. Any revisions will be uploaded to CANVAS and all
// students will be notified.


// Library Inclusions
#include <SCServo.h>
#include <math.h>

// Global Definitions
#define MAX_ID 6                                // Number of motors
#define SCS15_MAX_RPM 55                        // Maximum capable RPM of SCS15 motors
#define SCS15_MID_RPM 27                        // Middle SCS15 RPM value - Change of Linearity
#define SCS15_MIN 88                            // SCS15 Input Value of Zero RPM
#define SCS009_MAX_RPM 110                      // Maximum capable RPM of SCS009 motors
#define SCS009_MID_RPM 55                       // Middle SCS009 RPM value - Change of Linearity
#define SCS009_MIN 60                           // SCS009 Input Value of Zero RPM
#define SCS_MID 500                             // Middle motor velocity value - Change of Linearity
#define EPS 0.005                               // Point below which a value should be considered zero
#define RAD_2_RPM 0.1047                        // Conversions from radians to RPM
#define XJOY1_PIN A0                            // Joystick 1 x-axis input pin 
#define YJOY1_PIN A1                            // Joystick 1 y-axis input pin 
#define XJOY2_PIN A2                            // Joystick 2 x-axis input pin 
#define YJOY2_PIN A3                            // Joystick 2 y-axis input pin 
#define LED_PIN 13                              // Pin attached to onboard LED
#define SCS15_2_RAD 0.00366450                  // Convert SCS Format to Radians
#define SCS009_2_RAD 0.00511327                 // Convert SCS Format to Radians

// Global Variables
SCSCL sc;                                       // Motor Object used to send & receive motor data
long int motorBaudRate = 250000;                // BaudRate used between Arduino and Motors
long int usbBaudRate = 230400;                  // BaudRate used between MATLAB and Arduino
int motorFB[MAX_ID]={0,0,0,0,0,0};              // Array to store motor position feedback
double qVel[MAX_ID]={0,0,0,0,0,0};              // Array to store motor reference velocity
double q[MAX_ID]={0,0,0,0,0,0};                 // Array to store motor reference position
const double stopMotors[MAX_ID]={0,0,0,0,0,0};  // Velocity vector to stop motors
u8 ID[MAX_ID]={255,255,255,255,255,255};        // Vector of motor identification numbers
int joy1[2] = {0,0},xJoy2=0,yJoy2=0;            // Initialise joystick values
String runMode = "NULL";                        // Stores operation requested by MATLAB
int numID = 0;



// --------------------------------------- //
//    Manual Calibration of Motor Offset   //
// --------------------------------------- //

// May Need To Be Tuned For Specific Motors or Home Position Configurations.
// The total range of motion is from 0-1024, representing 215 degrees for the SCS15 motors 
// and 300 degrees for the SCS009 motors. A 90 degree offset has been chosen as a base 
// to allow -pi/2 to +pi/2 opperation.

const int motorOffset[MAX_ID] = {-512,-512,-512,-512,-512,-512};


// --------------------------------------- //
//              ARDUINO SETUP              //
// --------------------------------------- //

// Flags
bool changeMode = false;
bool velocityMode = true;

void setup() {
  // ------------ System Setup ------------ // 
  // Establish serial connections to both MATLAB and motors, and
  // set motor control mode. 

  // @ Serial               - This serial object communicates with 
  //                with MATLAB via the USB cable
  // @ Serial1              - This serial object communicates with 
  //                with the motors via the TX1 & RX1 pins
  //
  // @ changeControlMode()  - Change between position & velocity 
  //                control using velociyMode and changeMode flags


  // Set LED pin to output for EPROM indication
  pinMode(LED_PIN, OUTPUT);
  
  // Setup USB Serial
  Serial.begin(usbBaudRate);    // Set serial baud rate for USB
  while(!Serial);               // Wait For Serial To Connect

  // Setup Motor Serial
  Serial1.begin(motorBaudRate); // Set serial baud rate for motors
  while(!Serial1);              // Wait For Serial To Connect
  sc.pSerial = &Serial1;        // Assign serial port to motor

  // Read Connected Motors & Get IDs
  numID = getID(ID);

  // Establish serial connection with MATLAB
  establishSerial();  

  // Send Motor IDs To MATLAB
  sendMotorIDs();

  // ------------ Student Setup ------------ // 
  // Add any additional setup code you wish to run.







}


// --------------------------------------- //
//               ARDUINO RUN               //
// --------------------------------------- //

void loop() {
  // ------------- System Run ------------- // 
  // Run system functions each cycle. These are responsible for
  // dialoguing with the motors.

  // @ getRunMode()         - Get desired operation to be performed from MATLAB
  // @ readJoy()            - Read input from joystick pins
  // @ sendJoySerial()      - Send joystick data to MATLAB
  // @ getMultiMotorPos()   - Read all motor positions simultaniously
  // @ sendMotorPosSerial() - Send motor feedback to MATLAB
  // @ getMotorVelSerial()  - Read reference velocities from MATLAB
  // @ driveMotorsVel()     - Send velocity command to motors
  // @ driveMotorsPos()     - Send position command to motors

  if (Serial.available() > 3) {
    runMode = getRunMode();     // Get command type from MATLAB
    
    if (runMode == "joy") {
      readJoy(joy1, XJOY1_PIN, YJOY1_PIN); // Read joystick axis. Modify to add second joystick.
      sendDataSerial(joy1, 2);
    }
    else if (runMode == "fbk") {
      getMultiMotorPos(ID, motorFB);
      sendDataSerial(motorFB, numID);
    }
    else if (runMode == "vel") {
      getJointDataSerial(qVel);
      driveMotorVel(qVel);
      Serial.print("d");          // Send acknowledgement to MATLAB
      
    }
    else if (runMode == "pos") {
      getJointDataSerial(q);
      driveMotorPos(q);
      Serial.print("d");          // Send acknowledgement to MATLAB
      
    }
    else if (runMode == "cnp") {
      changeMode = true;
      velocityMode = false;
      if (changeMode) {
        changeControlMode();
      }
      changeMode = false;
    }
  
    else if (runMode == "cnv") {
      changeMode = true;
      velocityMode = true;
      if (changeMode) {
        changeControlMode();
      }
      changeMode = false;
    }
    else if (runMode == "pid") {
      int pid[4];
      getPIDGains(pid);
      tunePID(pid[0], pid[1], pid[2], pid[3]);
      Serial.print("d"); 
    }
    else if (runMode == "cid") {
      int IDs[2];
      getChangeIDs(IDs);
      changeID(IDs[0], IDs[1]);
      Serial.print("d");
    }
    
  
    while(Serial.available()) {   // Absorb any left over serial data before next command
      Serial.read();
    }
    runMode = "NULL";             // Reset runMode
  }

  // ------------- Student Run ------------- // 
  // Add any additional loop code you wish to run every cycle.
}



// --------------------------------------- //
//            STUDENT FUNCTIONS            //
// --------------------------------------- //
// Add your additional functions here.






// --------------------------------------- //
//            SYSTEM FUNCTIONS             //
// --------------------------------------- //
// Functions used for general system operation. Read these to
// get an understanding of how the system works, but they should not
// need editing for standard functionality.


String getRunMode() {
  // Read desired function to be performed from serial

  // Internal Variables
  // @ input         - Null terminated string read from MATLAB. This
  //          is sent prior to reading or writing data over serial.
  
  String input = Serial.readStringUntil('\n');
  return input;
}


void readJoy(int *joy, int xPin, int yPin) {

  // Reads x and y inputs from analog pins.

  // External Variables
  // @ joy              - Vector containing X & Y axis joystick values  
  // @ xPin             - Analog input pin which x-axis of joystick is connected 
  // @ yPin             - Analog input pin which y-axis of joystick is connected 
  
  
  joy[0] = analogRead(xPin);  
  joy[1] = analogRead(yPin);
}


void getMultiMotorPos(u8 *ID, int *motorFB){
  
  // Get position of all motors simultaniously

  // External Variables
  // @ ID             - Vector of all motor identifiers
  // @ motorFB        - Returned position of each motor in SCS format.
  // @ motorOffset    - Constant vector containing the offset required to
  //          give feedback between -pi/2 and +pi/2.

  // Internal Variables
  // @ sc             - Motor object allowing access to library functions

  // Internal Functions
  // @ ReadPosMulti() - External library for reading motor feedback 
  //          simultaniously.
  
  sc.ReadPosMulti(ID, numID, motorFB);
  
  for (int i = 0; i < numID; i++) {
    motorFB[i] += motorOffset[ID[i]-1];       // Compensate for motor offset
  }
}




int rad2scs(double qVel, long rpmMax, long rpmMid, long scsMin) {

  // Convert rad/s to scs motor command via known RPM values. The motors
  // have a break in linearity at half of their max input, splitting the 
  // required input mapping into two linear sections. 

  // External Variables
  // @ qVel           - Input joint-space velocity in rad/s
  // @ rpmMax         - Maximum RPM capable of the motor
  // @ rpmMid         - Middle RPM value of the motor at which there is a 
  //            change in velocity linearity
  // @ scsMin         - Motor input signal corresponding to zero RPM

  // Internal Variables
  // @ scsVel         - Mapped velocity value sent to motors

  int scsVel = -1;            // Initialise motor command
  qVel = qVel / RAD_2_RPM;    // Convert input from rad/s to RPM

  // Check direction and map from rpm to SCS motor command
  if (qVel < EPS && qVel > -EPS) {
    scsVel = 0;
  }

  // If velocity is positive, send velocity command between 1024 - 2024
  else if (qVel > 0) {
    
    // Protect from over saturation 
    if  (qVel > rpmMax) {
      qVel = rpmMax;
    }
    
    // Map rad/s to scs command for first linear segment
    if (qVel < rpmMid) {
      scsVel = map(qVel*100, 0, rpmMid*100, 1024 + scsMin, 1024 + SCS_MID);
    }

    // Map rad/s to scs command for last linear segment
    else {
      scsVel = map(qVel*100, rpmMid*100, rpmMax*100, 1024 + SCS_MID, 2024);
    }  
  }

  // If velocity is negative, send velocity command between 0 - 1000
  else {
    qVel = abs(qVel);

    // Protect from over saturation 
    if  (qVel > rpmMax) {
      qVel = rpmMax;
    }

    // Map rad/s to scs command for first linear segment
    if (qVel < rpmMid) {
      scsVel = map(qVel*100, 0, rpmMid*100, scsMin, SCS_MID);
    }

    // Map rad/s to scs command for last linear segment
    else {
      scsVel = map(qVel*100, rpmMid*100, rpmMax*100, SCS_MID, 1000);
    }
  }

  return scsVel;
}




void driveMotorVel(double *qVel) {
  
  // Drive motors at desired velocity by converting input to motor command

  // External Variables
  // @ qVel           - Input joint-space velocity in rad/s

  // Internal Variables
  // @ scsVel         - Array of mapped velocity values to be 
  //          sent to motors
  // @ sc             - Motor object allowing access to library functions

  // Internal Functions
  // @ rad2scs()      - Convert rad/s to motor command
  // @ SyncWriteVel() - Send motor commands to all motors simultaniously
  //    Takes:  ID[]        - array of motor ID numbers
  //            IDN         - number of motors being addressed
  //            Positions   - has NO EFFECT in velocity mode
  //            Time[]      - array of desired velocities 
  //            Speed       - has NO EFFECT in velocity mode
  
  int scsVel[numID];

  for (int i = 0; i < numID; i++) { 

    // Convert velocities to motor commands
    if (ID[i] >= 5) {
      // Apply SCS009 Scaling
      scsVel[i] = rad2scs(qVel[i], SCS009_MAX_RPM, SCS009_MID_RPM, SCS009_MIN);
    }
    else {
      // Apply SCS15 Scaling
      scsVel[i] = rad2scs(qVel[i], SCS15_MAX_RPM, SCS15_MID_RPM, SCS15_MIN);
    }
  }


  

  // Send velocity commands to all motors simultaniously
  sc.SyncWriteVel(ID, numID, 0, scsVel, 0);
}



void driveMotorPos(double *q) {
  
  // Drive motors to desired position by converting input to motor command

  // External Variables
  // @ q            - Input joint-space position in rad

  // Internal Variables
  // @ scsVel         - Array of mapped velocity values to be 
  //          sent to motors
  // @ sc             - Motor object allowing access to library functions

  // Internal Functions
  // @ rad2scs()      - Convert rad/s to motor command
  // @ SyncWriteVel() - Send motor commands to all motors simultaniously
  //    Takes:  ID[]        - array of motor ID numbers
  //            IDN         - number of motors being addressed
  //            Positions[] - array of desired positions
  //            Time        - how long it should take to move. Default: 0
  //            Speed       - how fast a move should be performed. Default: 0

  int scsPos[numID];
  
  
  for (int i = 0; i < numID; i++) {
    if (ID[i] >= 5) {
      scsPos[i] = (int) (q[i] / SCS009_2_RAD);   // Convert radian input to SCS input format
    }
    else {
      scsPos[i] = (int) (q[i] / SCS15_2_RAD);   // Convert radian input to SCS input format
    }
    scsPos[i] -= motorOffset[ID[i]-1];            // Compensate for motor offset
    
    if (scsPos[i] < 0) {
      scsPos[i] = 0;
    }
    else if (scsPos[i] > 1024) {
      scsPos[i] = 1024;
    }
  }

  // Send position commands to all motors simultaniously
  sc.SyncWritePos2(ID, numID, scsPos, 10, 0);
  
}



void sendDataSerial(int *data, int dataSize) {

  // Send integer data to MATLAB via serial.

  // External Variables
  // @ data         - Array of int data (e.g. motor feedback or joystick positions).
  
  for (int i = 0; i < dataSize; i++) {

    // Notify MATLAB of how many digits to expect prior to sending data.
    if (data[i] < 10 && data[i] >= 0) {
        Serial.print("1");
        Serial.print(data[i]);
      }
      else if ((data[i] >= 10 && data[i] < 100) || (data[i] < 0 && data[i] > -10)) {
        Serial.print("2");
        Serial.print(data[i]);
      }
      else if ((data[i] >= 100 && data[i] < 1000) || (data[i] <= -10 && data[i] > -100)) {
        Serial.print("3");
        Serial.print(data[i]);
      }
      else if ((data[i] > 1000) || (data[i] <= -100 && data[i] > -1000)) {
        Serial.print("4");
        Serial.print(data[i]);
      }
      else {
        Serial.print("e");
      }     
  }
}




void getJointDataSerial(double *qData) {

  // Read joint positions/velocities sent from MATLAB in rad/s.

  // External Variables
  // @ qData           - Float array to store joint positions/velocities.

  for (int i = 0; i < numID; i++) {
    qData[i] = Serial.parseFloat();
  }
}



void establishSerial() {
  
  // Establish serial connection with MATLAB by sending a single char,
  // waiting for a single char response, and then sending a final char
  // to acknoledge one was received.
  
  char c = 'b';
  Serial.println("a");
  while ( c != 'a' ) {
    c = Serial.read();
  }
  Serial.println("b");
}



void changeControlMode() {

  // Change the EPROM of all motors to allow for velocity or position
  // input. NOTE: the hex code 0xfe accesses all motor IDs.

  int angleLimit = 0;

  if (changeMode && !velocityMode) {
    angleLimit = 1023;
  }
  
  digitalWrite(LED_PIN, LOW);
  
  sc.writeByte(0xfe, SCSCL_LOCK, 0);
  delay(200);
  sc.writeByte(0xfe, SCSCL_MIN_ANGLE_LIMIT_L, 0);
  delay(100);
  sc.writeByte(0xfe, SCSCL_MAX_ANGLE_LIMIT_L, angleLimit);
  delay(100);
  sc.writeByte(0xfe, SCSCL_LOCK, 1);
  delay(200);
  
  digitalWrite(LED_PIN, HIGH);
  delay(50);
}


void tunePID(int id, int Kp, int Ki, int Kd) {

  // Tune the PID values of the specified motor. This only has an effect
  // when in position control mode. All gains must be whole numbers.
  
  digitalWrite(LED_PIN, LOW);

  // Unlock EPROM
  sc.writeByte(id, SCSCL_LOCK, 0);
  delay(200);

  // Change Values
  sc.writeByte(id, SCSCL_COMPLIANCE_P, Kp);
  delay(100);
  sc.writeByte(id, SCSCL_COMPLIANCE_I, Ki);
  delay(100);
  sc.writeByte(id, SCSCL_COMPLIANCE_D, Kd);
  delay(100);

  // Lock EPROM
  sc.writeByte(id, SCSCL_LOCK, 1);
  delay(200);
  
  digitalWrite(LED_PIN, HIGH);
  delay(50);
  
}

void getPIDGains(int *pid) {
  
  for (int i=0; i < 4; i++) {
    pid[i] = Serial.parseInt();
  }
}

void changeID(int oldID, int newID) {

  // Tune the PID values of the specified motor. This only has an effect
  // when in position control mode. All gains must be whole numbers.
  
  digitalWrite(LED_PIN, LOW);

  // Unlock EPROM
  sc.writeByte(0xfe, SCSCL_LOCK, 0);
  delay(200);

  // Change Values
  sc.writeByte(oldID, SCSCL_ID, newID);
  delay(200);

  // Lock EPROM
  sc.writeByte(0xfe, SCSCL_LOCK, 1);
  delay(200);
  
  digitalWrite(LED_PIN, HIGH);
  delay(50);
  
}

void getChangeIDs(int *IDs) {
  
  for (int i=0; i < 2; i++) {
    IDs[i] = Serial.parseInt();
  }
}



int getID(u8 *ID) {
  
  // Get the number if motors connected and their IDs.

  // External Variables
  // @ ID       - Array of connected motor IDs.

  // Internal Variables
  // @ numID    - Number of connected IDs.
  // @ tempID   - Temporary value for storing pre-checked ID value.
  // @ SCSCL_ID - EPROM reference for accessing motor ID.

  // Internal Functions
  // readByte() - Read a byte of data returned by the motors.
  
  int numID = 0;

  
  for (int i = 1; i <= MAX_ID; i++) {
    // Read ID value from motor
    int tempID = sc.readByte(i, SCSCL_ID);

    // If returned ID is valid, store.
    if (tempID > 0 && tempID <= MAX_ID) {
      ID[numID] = tempID;
      numID++;
    }
      
  }
  
  return numID;
}



void sendMotorIDs() {

  // Send ID of each connected motor and total number of connected
  // motors to MATLAB.

  // Internal Variables
  // @ numID          - Number of connected IDs.
  // @ ID             - Array of connected motor IDs.

  // Internal Functions
  // sendDataSerial() - Sends desired data over serial to MATLAB.

  // Convert numID into single element array to send over serial.
  int numIDTemp[1] = {numID};
  sendDataSerial(numIDTemp, 1);

  // Convert ID from type u8 to int for sending over serial.
  int idTemp[numID];
  for (int i=0; i < numID; i++) {
    idTemp[i] = (int) ID[i];
  }
  
  sendDataSerial(idTemp, numID);
}
