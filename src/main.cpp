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
int8_t lead = 2;  //2 for forwards, -2 for backwards


//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
PwmOut L2L(L2Lpin);
PwmOut L3L(L3Lpin);

DigitalOut L1H(L1Hpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3H(L3Hpin);

//Define global variables for position contol
int8_t orState = 0;    //Rotor offset at motor state 0


//Initialise the serial port
RawSerial pc(SERIAL_TX, SERIAL_RX);

//Define global variables for threading
Thread commOutT;
Thread commInT;

typedef struct {
    uint8_t code;
    uint32_t data;
    } message_t ;

Mail<message_t,16> outMessages;

enum messageCode {motorState, nonceFound, hashRate, keyChange, positionReport,
                    velocityReport, rotationCount, err=255};

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
       switch(pMessage->code){
        case hashRate:
            pc.printf("Mining with %.2f kHash/s\n\r", (int32_t)pMessage->data/1000.0f);
            break;
        case keyChange:
            pc.printf("Mining key changed to %x\n\r", pMessage->data);
            break;
        case positionReport:
            pc.printf("Position %.1f, ", (int32_t)pMessage->data / 6.0);
            break;
        case velocityReport:
            pc.printf("velocity %.1f\r\n", (int32_t)pMessage->data / 6.0);
            break;
        case motorState:
            pc.printf("Current motorState is %x\n\r", pMessage->data);
            break;
        case nonceFound:
            pc.printf("Nonce found: %x\n\r", pMessage->data);
            break;
        case err:
            pc.printf("Debug code of %x\n\r", pMessage->data);
            break;
        default:
            pc.printf("Unknown error with data %x\n\r", pMessage->data);
            break;
        }
        outMessages.free(pMessage);
    }
}

Queue<void, 8> inCharQ;

void serialISR(){
    uint8_t newChar = pc.getc();
    inCharQ.put((void*)newChar);
}


#define MAX_CMD_LENGTH 18
char newCmd[MAX_CMD_LENGTH];
int newCmdPos = 0;

volatile uint64_t newKey;
volatile float targetVelocity = 500.0;
volatile float targetRotation = 600.0;
volatile int32_t motorTorque = 300;

Mutex newKey_mutex;

void parseIn(){
        switch(newCmd[0]) {
                case 'R':
                        sscanf(newCmd, "R%f", &targetRotation);
                        pc.printf("Changed target rotation to %.1f \n\r", targetRotation);
                        break;
                case 'V':
                        sscanf(newCmd, "V%f", &targetVelocity);
                        pc.printf("Changed target velocity to %.1f \n\r", targetVelocity);
                        break;
                case 'K':
                        newKey_mutex.lock();
                        sscanf(newCmd, "K%x", &newKey);
                        newKey_mutex.unlock();
                        break;

                case 'S':
                        sscanf(newCmd, "S%d", &motorTorque);
			pc.printf("Setting torque to %d\n", motorTorque);
                        break;
        }
}

void commInFn() {
        pc.attach(&serialISR);
        //putMessage(err, 0x100);
        while(1) {
            //putMessage(err, 0x101);
            osEvent newEvent = inCharQ.get();
            uint8_t newChar = *((uint8_t*)(&newEvent.value.p));
	    pc.putc(newChar);
            //putMessage(err, 0x102);
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

int32_t motorPosition;

void motorISR(){
    static int8_t oldRotorState;
    int8_t rotorState = readRotorState();

    motorOut((rotorState-orState+lead+6)%6, motorTorque); //+6 to make sure the remainder is positive
    if (rotorState - oldRotorState == 5) motorPosition--;
    else if (rotorState - oldRotorState == -5) motorPosition++;
    else motorPosition += (rotorState - oldRotorState);
    oldRotorState = rotorState;
}

Thread motorCtrlT (osPriorityNormal, 512);

void motorCtrlTick(){
    motorCtrlT.signal_set(0x1);
}

void motorCtrlFn(){
    Ticker motorCtrlTicker;
    motorCtrlTicker.attach_us(&motorCtrlTick, 100000);
    static int32_t oldMotorPosition;
    int32_t velocity = 0;

    float error_s;
    static float error_s_int = 0;

    float error_r;
    static float error_r_int = 0 ;
    static float old_error_r;

    int32_t ctorque;

    uint8_t iterations = 0;
    while(1){
        motorCtrlT.signal_wait(0x1);
        //putMessage(err, motorPosition);
        int32_t currPosition = motorPosition;
        velocity = (currPosition - oldMotorPosition) * 10;
        oldMotorPosition = currPosition;
        iterations = (iterations + 1)% 20;
        if (!iterations) {
            putMessage(positionReport, motorPosition);
            putMessage(velocityReport, velocity);
        }
        // Proportional control with k_p = 10

        error_s = (targetVelocity*6.0f - abs(velocity));
        int32_t y_s = (int)(15.0f * error_s);
        error_s_int += error_s;

        error_r = targetRotation - currPosition/6.0f;
        int32_t y_r = (int)( 11.5f * error_r + 95.7f * (error_r - old_error_r));
        old_error_r = error_r;
        error_r_int += error_r;

        int32_t tmp;

        if (error_r < 0) y_s = -y_s;

        if (velocity >= 0){
            // pick min of (y_s and y_r)
            if (y_s < y_r)
                ctorque = y_s;
            else
                ctorque = y_r;
        } else {
            if (y_s > y_r){
                ctorque = y_s;
            } else {
                ctorque = y_r;
            }
        }

        if (ctorque < 0){
            motorTorque = -ctorque;
            lead = -2;
        } else {
            motorTorque = ctorque;
            lead = 2;
        }

        // Spin up motor if velocity = 0 as no interrupts get triggered
        if (velocity == 0)
            motorISR();
    }
}

//Main
int main() {

    L1L.period_us(2000);
    L2L.period_us(2000);
    L3L.period_us(2000);


    pc.printf("Hello\n\r");

    commOutT.start(commOutFn);
    commInT.start(commInFn);

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
    I1.rise(&motorISR);
    I2.rise(&motorISR);
    I3.rise(&motorISR);
    I1.fall(&motorISR);
    I2.fall(&motorISR);
    I3.fall(&motorISR);

    motorISR();


    motorCtrlT.start(motorCtrlFn);

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
            //putMessage(nonceFound, *nonce);
        }
     //   if (*nonce%102==0){
     //       pc.printf("h");
     //   }
        (*nonce) += 1;
        timePassed = timer.read();
        if (timePassed > 8){
            putMessage(hashRate, hashCount);
            hashCount=0;
            timer.reset();
        }
    }
}
