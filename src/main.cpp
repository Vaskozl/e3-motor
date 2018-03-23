#include "SHA256.h"
#include "mbed.h"
#include "rtos.h"
#include "float.h"

// PID coefficients
#define KS_P 17.0
#define KS_I 0.11

#define KR_P 29.7
#define KR_D 252.0
// #define KR_I 0.01

// Duty cycle used for PWM, as specified to avoid non linear behaviour
#define MAX_DUTYCL 1000
#define PWM_PERIOD 2000

// How often to print the hash rate and speed/ velocity report
#define HASHRATE_INTERVAL 8.0 // 8s
#define STATE_INTERVAL    1   // 2s

// Max input command size, based on size of key used for hashing
#define MAX_CMD_LENGTH 18 

// Define initial speed and rotation, so that the motor does something on startup
#define INIT_SPEED    80.0
#define INIT_ROTATION 400.0

// Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

// Incremental encoder input pins
#define CHA D7
#define CHB D8

// Motor Drive output pins   //Mask in output byte
#define L1Lpin D4  // 0x01
#define L1Hpin D5  // 0x02
#define L2Lpin D3  // 0x04
#define L2Hpin D6  // 0x08
#define L3Lpin D9  // 0x10
#define L3Hpin D10 // 0x20

// Mapping from sequential drive states to motor phase outputs
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
// Drive state to output table
const int8_t driveTable[] = {0x12, 0x18, 0x09, 0x21, 0x24, 0x06, 0x00, 0x00};

// Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are
// not valid as a photointerrupter output (which is an index)
const int8_t stateMap[] = {0x07, 0x05, 0x03, 0x04, 0x01, 0x00, 0x02, 0x07};
// const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07};
// //Alternative if phase order of input or drive is reversed

// Phase lead to make motor spin
int8_t lead = 2; // 2 for forwards, -2 for backwards

// Status LED
DigitalOut led1(LED1);

// Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

// Motor Drive outputs
PwmOut L1L(L1Lpin);
PwmOut L2L(L2Lpin);
PwmOut L3L(L3Lpin);

DigitalOut L1H(L1Hpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3H(L3Hpin);

// Define global variables for position contol
volatile int32_t motorPosition;
volatile int32_t oldMotorPosition;

int8_t orState = 0; // Rotor offset at motor state 0

// Initialise the serial port
RawSerial pc(SERIAL_TX, SERIAL_RX);

// Define communications thread with reduce stack size
Thread commOutT(osPriorityNormal, 1024);
Thread commInT(osPriorityNormal, 2048);

// A struct used for sending messages between threads
// Code is message type, data is message content
typedef struct {
  uint8_t code;
  uint32_t data;
} message_t;

// Array of messages to print to console
Mail<message_t, 16> outMessages;

// Enum for giving message types a code
enum messageCode {
  motorState,
  nonceHigh,
  nonceLow,
  hashRate,
  keyChange,
  rotChange,
  velChange,
  positionReport,
  velocityReport,
  rotationCount,
  err = 255
};

void putMessage(uint8_t code, uint32_t data) {
  message_t *pMessage = outMessages.alloc();
  pMessage->code = code;
  pMessage->data = data;
  outMessages.put(pMessage);
}

void commOutFn() {
  while (1) {
    osEvent newEvent = outMessages.get();
    message_t *pMessage = (message_t *)newEvent.value.p;
    switch (pMessage->code) {
    case hashRate:
      pc.printf("Mining with %.2f kHash/s\n\r",
                (int32_t)pMessage->data / 1000.0f);
      break;
    case keyChange:
      pc.printf("Mining key changed.");
      break;
    case positionReport:
      pc.printf("Position: %.1f, ", (int32_t)pMessage->data / 6.0);
      break;
    case velocityReport:
      pc.printf("Velocity: %.1f \n\r", (int32_t)pMessage->data/6.0);
      break;
    case rotChange:
      pc.printf("Changed target position to %.1f \n\r", (float)pMessage->data);
      break;
    case velChange:
      if (pMessage->data <= 0)
        pc.printf("Removed maximum speed cap.\n\r");
      else
        pc.printf("Changed target velocity to %.1f \n\r", (int32_t)pMessage->data/6.0);
      break;

    case motorState:
      pc.printf("Current motorState is %x\n\r", pMessage->data);
      break;
    case nonceHigh:
      pc.printf("Nonce found: 0x%x", pMessage->data);
      break;
    case nonceLow:
      pc.printf("%x\n\r", pMessage->data);
      break;
    case err:
      if (pMessage->data == 1)
        pc.printf("Serial input too long.\n\r");
      else 
        pc.printf("Error code %x.\n\r", pMessage->data);
      break;
    default:
      pc.printf("Unknown error with data %x\n\r", pMessage->data);
      break;
    }
    outMessages.free(pMessage);
  }
}

Queue<void, 8> inCharQ;

void serialISR() {
  uint8_t newChar = pc.getc();
  inCharQ.put((void *)newChar);
}

char newCmd[MAX_CMD_LENGTH];

/*
 * Key is 64 bits so we protect it with a mutex
 * The rest are 32 bits and we take care to access
 * them attomaticaly in both motorISR and motorCtrlFn
 */


volatile uint64_t newKey;
volatile float targetVelocity = INIT_SPEED;
volatile float targetRotation = INIT_ROTATION;
volatile int32_t motorTorque = 800;

Mutex newKey_mutex;

/* Function to parse the input over serial 
 * V0 and R0 make the motor spin at maximum 
 * positve speed to the maximum position
 */

void parseIn() {
  float tmp = 0;
  switch (newCmd[0]) {
  case 'R':
    sscanf(newCmd, "R%f", &tmp);
    oldMotorPosition = -(motorPosition - oldMotorPosition);
    motorPosition = 0;
    if (tmp == 0)
      targetRotation = FLT_MAX;
    else
      targetRotation = tmp;
    putMessage(rotChange, tmp);
    break;
  case 'V':
    sscanf(newCmd, "V%f", &tmp);
    if (tmp <= 0){
      targetVelocity = FLT_MAX;
      putMessage(velChange, 0);
    } else {
      targetVelocity = tmp;
      putMessage(velChange, (int32_t)tmp * 6.0f);
    }
    break;
  case 'K':
    // Protect key with mutex as it is 64 bits and 
    // we can not guarrantee atomic access
    newKey_mutex.lock();
    sscanf(newCmd, "K%x", &newKey);
    newKey_mutex.unlock();
    putMessage(keyChange, 0);
    break;
  }
}

void commInFn() {
  pc.attach(&serialISR);
  static int newCmdPos = 0;
  while (1) {
    osEvent newEvent = inCharQ.get();
    uint8_t newChar = *((uint8_t *)(&newEvent.value.p));
    if (newCmdPos >= MAX_CMD_LENGTH) {
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

// Set a given drive state
void motorOut(int8_t driveState, uint32_t pulseWidth) {

  // Lookup the output byte from the drive state.
  int8_t driveOut = driveTable[driveState & 0x07];

  // Turn off first
  if (~driveOut & 0x01)
    L1L.pulsewidth_us(0);
  if (~driveOut & 0x02)
    L1H = 1;
  if (~driveOut & 0x04)
    L2L.pulsewidth_us(0);
  if (~driveOut & 0x08)
    L2H = 1;
  if (~driveOut & 0x10)
    L3L.pulsewidth_us(0);
  if (~driveOut & 0x20)
    L3H = 1;

  // Then turn on
  if (driveOut & 0x01)
    L1L.pulsewidth_us(pulseWidth);
  if (driveOut & 0x02)
    L1H = 0;
  if (driveOut & 0x04)
    L2L.pulsewidth_us(pulseWidth);
  if (driveOut & 0x08)
    L2H = 0;
  if (driveOut & 0x10)
    L3L.pulsewidth_us(pulseWidth);
  if (driveOut & 0x20)
    L3H = 0;
}

// Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState() { return stateMap[I1 + 2 * I2 + 4 * I3]; }

// Basic synchronisation routine
int8_t motorHome() {
  // Put the motor in drive state 0 and wait for it to stabilise
  motorOut(0, MAX_DUTYCL);
  wait(2.0);

  // Get the rotor state
  return readRotorState();
}


void motorISR() {
  static int8_t oldRotorState;
  int8_t rotorState = readRotorState();

  motorOut((rotorState - orState + lead + 6) % 6,
           motorTorque); //+6 to make sure the remainder is positive
  if (rotorState - oldRotorState == 5)
    motorPosition--;
  else if (rotorState - oldRotorState == -5)
    motorPosition++;
  else
    motorPosition += (rotorState - oldRotorState);
  oldRotorState = rotorState;
}

Thread motorCtrlT(osPriorityHigh, 512);

// Allow motorCntl function to run every tick (10ms in our case)
void motorCtrlTick() { motorCtrlT.signal_set(0x1); }

void motorCtrlFn() {
  Ticker motorCtrlTicker;
  motorCtrlTicker.attach_us(&motorCtrlTick, 100000);
  int32_t velocity = 0;

  float error_s;
  static float error_s_int = 0;

  float error_r;
  //static float error_r_int = 0;
  static float old_error_r;

  int32_t ctorque;

  uint8_t iterations = 0;
  while (1) {
    motorCtrlT.signal_wait(0x1);
    int32_t currPosition = motorPosition;
    velocity = (currPosition - oldMotorPosition) * 10;
    oldMotorPosition = currPosition;
    // Print position and speed every 2 seconds
    iterations = (iterations + 1) % (10 * STATE_INTERVAL);
    if (!iterations) {
      putMessage(positionReport, currPosition);
      putMessage(velocityReport, velocity);
    }

    // Speed proportional control with k_p
    error_s = (targetVelocity * 6.0f - abs(velocity));
    int32_t y_s = (int)(KS_P * error_s + KS_I * error_s_int);
    error_s_int += error_s;

    // Positional PD control with k_p and k_d 
    error_r = targetRotation - currPosition / 6.0f;
    int32_t y_r = (int)(KR_P * error_r + KR_D * (error_r - old_error_r));
    old_error_r = error_r;
    //error_r_int += error_r;

    if (error_r < 0) y_s = -y_s;

    // Prevent motor from beeping if close enough
    if ((error_r > -0.4) && (error_r < 0.4)) y_r = 0;

    if (velocity >= 0){
      // pick min of (y_s and y_r)
      // reset speed integral if we use y_r
      if (y_s < y_r){
        ctorque = y_s;
        //error_r_int = 0;
      } else {
        ctorque = y_r;
        error_s_int = 0;
      }
    } else {
        if (y_s > y_r){
          ctorque = y_s;
          //error_r_int = 0;
        } else {
          ctorque = y_r;
          error_s_int = 0;
        }
    }
   
    if (ctorque < 0) {
      ctorque = -ctorque;
      lead = -2;
    } else {
      lead = 2;
    }

    // Atomic access to motorTorque which is used
    // by the interrupt ISR handler
    if (ctorque > MAX_DUTYCL)
      motorTorque = MAX_DUTYCL;
    else
      motorTorque = ctorque;

    // Spin up motor if velocity = 0 as no interrupts get triggered
    if (velocity == 0)
      motorISR();
  }
}

// Main
int main() {

  L1L.period_us(PWM_PERIOD);
  L2L.period_us(PWM_PERIOD);
  L3L.period_us(PWM_PERIOD);

  pc.printf("Hey, this is group VKPD's BLDC motor controller!\n\r");

  // Start console in and out threads
  commOutT.start(commOutFn);
  commInT.start(commInFn);

  // Run the motor synchronisation
  orState = motorHome();
  putMessage(motorState, orState);
  // orState is subtracted from future rotor state inputs to align rotor and
  // motor states

  // Bitcoin mining

  SHA256 SHA256instance;

  uint8_t sequence[] = {
      0x45, 0x6D, 0x62, 0x65, 0x64, 0x64, 0x65, 0x64, 0x20, 0x53, 0x79,
      0x73, 0x74, 0x65, 0x6D, 0x73, 0x20, 0x61, 0x72, 0x65, 0x20, 0x66,
      0x75, 0x6E, 0x20, 0x61, 0x6E, 0x64, 0x20, 0x64, 0x6F, 0x20, 0x61,
      0x77, 0x65, 0x73, 0x6F, 0x6D, 0x65, 0x20, 0x74, 0x68, 0x69, 0x6E,
      0x67, 0x73, 0x21, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint64_t *key = (uint64_t *)((int)sequence + 48);
  uint64_t *nonce = (uint64_t *)((int)sequence + 56);
  uint8_t hash[32];
  uint32_t lengthOfSequence = 64;

  // Initialise the interrupts
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
  // Poll the rotor state and set the motor outputs accordingly to spin the
  // motor
  while (1) {
    newKey_mutex.lock();
    (*key) = newKey;
    newKey_mutex.unlock();
    SHA256instance.computeHash(hash, sequence, lengthOfSequence);
    hashCount++;
    // Tell us when we find a nonce :)
    if ((hash[0] == 0) && (hash[1] == 0)) {
      putMessage(nonceHigh, (*nonce));
      putMessage(nonceLow, (*nonce+8));
    }
    (*nonce) += 1;
    timePassed = timer.read();
    if (timePassed > HASHRATE_INTERVAL) {
      putMessage(hashRate, hashCount/HASHRATE_INTERVAL);
      hashCount = 0;
      timer.reset();
    }
  }
}
