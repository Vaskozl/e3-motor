#include "mbed.h"
#include "SHA256.h"
#include "rtos.h"

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8  

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

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid as a photointerrupter output (which is an index)
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
const int8_t lead = 2;  //2 for forwards, -2 for backwards
int revCount = 0; //number of revolutions


//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//Define global variables for position contol
    int8_t orState = 0;    //Rotor offset at motor state 0
    int8_t intState = 0;
    
    
//Initialise the serial port
RawSerial pc(SERIAL_TX, SERIAL_RX);
    
//Define global variables for threading
Thread commOutT; 

typedef struct {
    uint8_t code;
    uint32_t data;
    } message_t ;

Mail<message_t,16> outMessages;

enum messageCode {motorState, nonceFound, hashRate, keyChange, rotationCount, err=255};

void putMessage(uint8_t code, uint32_t data){
        message_t *pMessage = outMessages.alloc();
        pMessage->code = code;
        pMessage->data=data;
        outMessages.put(pMessage);
}

void commOutFn(){
    while(1){
        osEvent newEvent = outMessages.get();
        message_t *pMessage = (message_t*)newEvent.value.p;
        pc.printf("Message %d with data 0x%016x\n\r", pMessage->code, pMessage->data);
        outMessages.free(pMessage);
    }
}

Queue<void, 8> inCharQ;
    
void serialISR(){
    uint8_t newChar = pc.getc();
    inCharQ.put((void*)newChar);
}

Thread commInT;

#define MAX_CMD_LENGTH 18
char newCmd[MAX_CMD_LENGTH];
int newCmdPos = 0;

volatile uint64_t newKey;
volatile float newSpeed;
volatile float newRotation;
volatile uint32_t motorTorque = 10;

Mutex newKey_mutex;
Mutex newRotation_mutex;
Mutex newSpeed_mutex;
Mutex motorTorque_mutex;

void parseIn(){
        switch(newCmd[0]) {
                case 'R':
                        newRotation_mutex.lock();
                        sscanf(newCmd, "R%f", &newRotation);
                        newRotation_mutex.unlock();
                        break;
                case 'V':
                        newSpeed_mutex.lock();
                        sscanf(newCmd, "V%f", &newSpeed);
                        newSpeed_mutex.unlock();
                        break;
                case 'K':
                        newKey_mutex.lock();
                        sscanf(newCmd, "K%x", &newKey);
                        newKey_mutex.unlock();
                        break;
                        
                case 'S':
                        motorTorque_mutex.lock();
                        sscanf(newCmd, "S%d", &motorTorque);
                        motorTorque_mutex.unlock();
                        break;
        }
}

void commInFn() {
        pc.attach(&serialISR);
        while(1) {
            osEvent newEvent = inCharQ.get();
            uint8_t newChar = (uint8_t)newEvent.value.p;
            if(newCmdPos >= MAX_CMD_LENGTH){
                newCmdPos = 0;
                putMessage(err, 0x1);    
            } else {
                if (newChar != '\r') {
                    newCmd[newCmdPos++] = newChar;    
                } else {          
                    newCmd[newCmdPos] = '\0';
                    newCmdPos = 0;
                    parseIn();
                }   
            }   
        }
}

//Set a given drive state
void motorOut(int8_t driveState, uint32_t pulseWidth){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L.pulsewidth_us(0);
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L.pulsewidth_us(0);
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L.pulsewidth_us(0);
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L.pulsewidth_us(pulseWidth);
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L.pulsewidth_us(pulseWidth);
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L.pulsewidth_us(pulseWidth);
    if (driveOut & 0x20) L3H = 0;
    }
    
    //Convert photointerrupter inputs to a rotor state
    inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0, 256);
    wait(2.0);
    
    //Get the rotor state
    return readRotorState();
}

void positionChangeHandler(){
    intState = readRotorState();
    motorOut((intState-orState+lead+6)%6, motorTorque); //+6 to make sure the remainder is positive
    revCount++;
}

    
//Main
int main() {
    
    L1L.period_us(256);
    L2L.period_us(256);
    L3L.period_us(256);
    
    
    pc.printf("Hello\n\r");
    
    commInT.start(commInFn);
    commOutT.start(commOutFn);
    
    
    
    //Run the motor synchronisation
    orState = motorHome();
    putMessage(motorState, orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states
    
    //Bitcoin mining
    
    SHA256 SHA256instance;
    
    uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
    0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
    0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
    0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
    0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
    0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint64_t* key = (uint64_t*)((int)sequence + 48);
    uint64_t* nonce = (uint64_t*)((int)sequence + 56);
    uint8_t hash[32];
    uint32_t lengthOfSequence = 64;
    
    
    //Initialise the interrupts
    I1.rise(&positionChangeHandler);
    I2.rise(&positionChangeHandler);
    I3.rise(&positionChangeHandler);
    I1.fall(&positionChangeHandler);
    I2.fall(&positionChangeHandler);
    I3.fall(&positionChangeHandler);
    
    positionChangeHandler();
    
    float timePassed;
    Timer timer;
    timer.start();
    int hashCount = 0;
    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    while (1) {
        newKey_mutex.lock();
        (*key) = newKey;
        newKey_mutex.unlock();
        SHA256instance.computeHash(hash, sequence, lengthOfSequence);
        hashCount++;
        if ((hash[0]==0) && (hash[1]==0)){
            //putMessage(nonceHigh, (*nonce) >> 32);
            putMessage(nonceFound, *nonce);
        }
     //   if (*nonce%102==0){
     //       pc.printf("h");
     //   }
        (*nonce) += 1;
        timePassed = timer.read();
        if (timePassed > 1){
            //putMessage(hashRate, hashCount);
            putMessage(rotationCount, revCount/6);
            revCount=0;
            hashCount=0;
            timer.reset();
        }
    }
}