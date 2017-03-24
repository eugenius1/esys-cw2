#include "mbed.h"
#include "rtos.h"
#include "RawSerial.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHApin   D7
#define CHBpin   D8  

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
volatile int8_t lead = 1;  //2 for forwards, -2 for backwards

//Status LED
DigitalOut led1(LED1);

InterruptIn I1(I1pin);
DigitalIn I1set(I1pin);
InterruptIn I2(I2pin);
DigitalIn I2set(I2pin);
InterruptIn I3(I3pin);
DigitalIn I3set(I3pin);
InterruptIn CHA(CHApin);
InterruptIn CHB(CHBpin);
RawSerial pc(SERIAL_TX, SERIAL_RX); // tx rx
Thread pidThread(osPriorityNormal, 1024);
Thread serialThread(osPriorityNormal, 1024);
Timer speedTimer;

//Motor Drive outputs
PwmOut L1L(L1Lpin);
PwmOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
PwmOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
PwmOut L3H(L3Hpin);

volatile int angle = 0;
volatile int angleTemp = 0;
float angleDeg = 0;
volatile int deltaA = 0;
volatile float target = 0;
volatile int8_t I1var = 0;
volatile int8_t I2var = 0;
volatile int8_t I3var = 0;
volatile int dirFlag = 0;
volatile int i = 0;
const float kp = 1;
const float ki = 0.0;
const float kd = 0.0;
volatile float fbError = 0.0; // feedback error
volatile float lastError = 0.0;
volatile float inter = 0.0;
volatile float diff = 0.0;
volatile float dutyCycle = 0.0;
volatile float period = 0.0005;
const float degFactor = 0.76923076923076923076923076923077;
const float timerInterval = 120;
const float speedFactor = timerInterval / 468;
char serialBuffer[17]; 
char tempBuffer [6];
char inputChar; 
int index = 0;
int speedIndex = 0;
volatile double speedTarget = 0; 
volatile double rotations = 0; 
bool printFlag = false;
bool infRotate = true;
bool maxSpeed = true; 
bool speedRotateFlag = false;

void Arise () {
    if (CHB == 1) {
        angle--;
        dirFlag = -2;
    }
    if (CHB == 0) {
        angle++;
        dirFlag = 1;
    }
}

void Afall () {
    if (CHB == 0) {
        angle--;
        dirFlag = -2;
    }
    if (CHB == 1) {
        angle++;
        dirFlag = 1;
    }
}

void Brise () {
    if (CHA == 0) {
        angle--;
        dirFlag = -2;
    }
    if (CHA == 1) {
        angle++;
        dirFlag = 1;
    }
}

void Bfall () {
    if (CHA == 1) {
        angle--;
        dirFlag = -2;
    }
    if (CHA == 0) {
        angle++;
        dirFlag = 1;
    }
}

void I1rise () {
    I1var = 1;
}

void I1fall () {
    I1var = 0;
}

void I2rise () {
    I2var = 1;
}

void I2fall () {
    I2var = 0;
}

void I3rise () {
    I3var = 1;   
}

void I3fall () {
    I3var = 0;
}

//Set a given drive state
void motorOut (int8_t driveState) {
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L.write(0);
    if (~driveOut & 0x02) L1H.write(dutyCycle);
    if (~driveOut & 0x04) L2L.write(0);
    if (~driveOut & 0x08) L2H.write(dutyCycle);
    if (~driveOut & 0x10) L3L.write(0);
    if (~driveOut & 0x20) L3H.write(dutyCycle);
    
    //Then turn on
    if (driveOut & 0x01) L1L.write(dutyCycle);
    if (driveOut & 0x02) L1H.write(0);
    if (driveOut & 0x04) L2L.write(dutyCycle);
    if (driveOut & 0x08) L2H.write(0);
    if (driveOut & 0x10) L3L.write(dutyCycle);
    if (driveOut & 0x20) L3H.write(0);
    }
    
    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState (){ 
    return stateMap[I1set + 2*I2set + 4*I3set];
}

//Basic synchronisation routine    
int8_t motorHome () {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(1.0);
    
    //Get the rotor state
    return readRotorState();
}

void pid () {
    while (1) {
        // time how long it takes to advance by timerInterval steps
        if (dirFlag > 0) {
            angleTemp = angle + timerInterval;
            speedTimer.start();
            while (angle < angleTemp) {
                1;
            }
        }
        else if (dirFlag < 0) {
            angleTemp = angle - timerInterval;
            speedTimer.start();
            while (angle > angleTemp) {
                1;
            }
        }
        speedTimer.stop();
        
        lastError = fbError;
        
        // calculate the feedback error
        fbError = speedTimer.read() - target;
        
        speedTimer.reset();
        
        // integral calculation
        inter = inter + fbError;
        
        // differential calculation
        diff = fbError - lastError;
        // PID result is added to the current duty cycle
        dutyCycle = dutyCycle + (kp * fbError) + (ki * inter) + (kd * diff);
        // duty cycle limited to between 0 and 1
        if (dutyCycle > 1) {
            dutyCycle = 1;
        }
        if ((dutyCycle < 0) || (dirFlag != lead)) {
            dutyCycle = 0;
        }
    }
}

void serial () {
    pc.printf("Please enter expression for speed and/or rotations. \r\n");
    // regular expression (R-?\d{1,3}(\.\d{1,2})?)?(V\d{1,3}(\.\d{1,3})?)?
    while (1) {
        
        // while the user is inputting 
        while (pc.readable()){
            //takes the input characters into a variable
            inputChar = pc.getc();
            //writes back to the screen so the user can see input
            pc.putc(inputChar);
            // checks if terminating character
            if (inputChar == '\r'){
                pc.printf("\r\n");
                //set print flag high, triggers parsing
                printFlag = true;
            } 
            // otherwise add the input character to the buffer
            // and move through the buffer
            else {
                serialBuffer[index] = inputChar;
                index++;
            }
        }
        
        // if flag has been set 
        if (printFlag){
            
            /*pc.printf("user input was: \r\n");
            for(int j =0; j<i; j++){
                 pc.printf("%c", buffer[j]);
            }
            pc.printf("\r\n");
            */
            
            
            // user input begins with V so only speed set by user
            if (serialBuffer[0] == 'V' || serialBuffer[0] == 'v'){
                // flags set
                infRotate = true;
                maxSpeed = false; 
                //the rest of the array is converted into a double
                speedTarget = atof(&serialBuffer[1]);
                pc.printf("User set speed only. Motor will rotate continuously at: %f rps", speedTarget);
            }
            
            // statement begins with R, so could be rotations or rotations and speed
            if (serialBuffer[0] == 'R' || serialBuffer[0] == 'r'){
                
                //scan array looking for 'v' indicating speed and rotations have been set
                for (int j=1; j<index; j++){ 
                   if (serialBuffer[j] == 'v' || serialBuffer[j] =='V'){
                        //set flag to say both speed and rotation have been set 
                        speedRotateFlag = true;
                        //store where the speed value starts in buffer 
                        speedIndex = j+1; 
                    }
                    else if (!speedRotateFlag){
                        //otherwise move the values into temp buffer
                        tempBuffer[j-1] = serialBuffer[j]; 
                        //pc.printf("%c", tempBuffer[j-1]);
                    }
                }
                //pc.printf("\r\n");
                // convert the tempBuffer into a double, 
                //will hold the value of number of rotations 
                rotations = atof(&tempBuffer[0]);
                
                //if 'v' is found -> both R and V
                if (speedRotateFlag){
                    // convert the speed value from the buffer array into double 
                    speedTarget = atof(&serialBuffer[speedIndex]);
                    pc.printf("User set number of rotations and speed. Motor will rotate %f times at %f rps", rotations, speedTarget);
                    //reset the flag
                    speedRotateFlag = false;
                    
                    infRotate = false; 
                    maxSpeed = false;  
                }
                else {
                    infRotate = false;
                    maxSpeed = true;
                    pc.printf("User set number of rotations only. Motor will rotate %f times at full speed", rotations);
                }
            }
                    
            pc.printf("\r\n");
            
            // reset the buffer
            for (int j =0; j<sizeof(serialBuffer); j++){
                 serialBuffer[j] = NULL;
            }
            for (int j=0; j<sizeof(tempBuffer); j++){
                tempBuffer[j] = NULL;
            } 
            //reset the print flag
            printFlag = false;
            // reset index
            index=0; 
            
            // pass speed and rotations to processing module
            pidThread.terminate(); 
            target = abs(speedFactor / speedTarget);
            if (speedTarget > 0) {
                lead = 1;
                pidThread.start(&pid);
            }
            else if (speedTarget < 0) {
                lead = -2;
                pidThread.start(&pid);
            }
            else if (speedTarget == 0) {
                lead = 0;
                dutyCycle = 0;
            }
        }
    }
}

//Main
int main () {    
    int8_t orState = 0;    //Rotot offset at motor state 0
    
    int8_t intState = 0;
    int8_t intStateOld = 0;
    pc.printf("Hello\n\r");
    
    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r", orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states
    
    I1var = I1set;
    I2var = I2set;
    I3var = I3set;

    CHA.rise(&Arise);
    CHA.fall(&Afall);
    CHB.rise(&Brise);
    CHB.fall(&Bfall);
    I1.rise(&I1rise);
    I1.fall(&I1fall);
    I2.rise(&I2rise);
    I2.fall(&I2fall);
    I3.rise(&I3rise);
    I3.fall(&I3fall);

    L1L.period(period);
    L1H.period(period);
    L2L.period(period);
    L2H.period(period);
    L3L.period(period);
    L3H.period(period);

    serialThread.start(&serial);
    
    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    while (1) {
        intState = readRotorState();
        if (intState != intStateOld) {
            intStateOld = intState;
            motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
        }
    }
}

