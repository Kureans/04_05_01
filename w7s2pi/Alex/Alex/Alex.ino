#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

/*
 Darude - Sandstorm

 (c) Jordi Agricola 2014

 Speaker on port speaker

 */

 #define speaker A2
 #define red A3
 #define green A4
 #define blue A5
 
// note durations: 4 = quarter note, eight = eighth note, etc.:
  
 int shortTone = 90; //change back to eighty
 int longTone = 200;
 int standardDelay = 50;
 int delayBetweenBars = 50;
 

#include <serialize.h>
#include <stdarg.h>
#include "packet.h"
#include "constants.h"
#include <math.h>
#include <avr/sleep.h>
/*
 * Alex's configuration constants
 */

typedef enum
{
  STOP=0,
  FORWARD=1,
  BACKWARD=2,
  LEFT=3,
  RIGHT=4
} TDirection;

volatile TDirection dir = STOP;

//Ultrasonic Pins
#define echoPin A0 // attach pin A0 Arduino to pin Echo of HC-SR04
#define trigPin A1 //attach pin A1 Arduino to pin Trig of HC-SR04
#define ECHO_PIN_MASK 0b00000001
#define TRIG_PIN_MASK 0b00000010

//Ultrasonic Variables
double duration; // variable for the duration of sound wave travel
float obs_distance; // variable for the distance measurement
#define OBS_DIST_THRESHOLD 5.0 //threshold for min dist between Alex and obstacle 

//Colour Sensor Pins
#define s0 8        //Module pins wiring
#define s1 9
#define s2 12
#define s3 13
#define out 7

int Red=0, Blue=0, Green=0;  //RGB values 

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      150.0

//#define PI 3.141592654

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.42035

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  11  // Right reverse pin

// Alex's length and breadth in cm 
#define ALEX_LENGTH         24  
#define ALEX_BREADTH        17

// Alex's diagonal; computed and stored once
float alexDiagonal = 0.0;

// Alex's turning cirumference, calculated once
float alexCirc = 0.0;

/*
 *    Alex's State Variables
 */

// Store the forward ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;

//Store the reverse ticks from Alex's left and
//right encoders.
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

//Left and Right ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

//Variables to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

//Variables to keep track of turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

//PRR Register Masks
#define PRR_TWI_MASK 0b10000000
#define PRR_SPI_MASK 0b00000100
#define ADCSRA_ADC_MASK 0b10000000
#define PRR_ADC_MASK 0b00000001
#define PRR_TIMER2_MASK 0b01000000
#define PRR_TIMER0_MASK 0b00100000
#define PRR_TIMER1_MASK 0b00001000
#define SMCR_SLEEP_ENABLE_MASK 0b00000001
#define SMCR_IDLE_MODE_MASK 0b11110001

/* Alex's power saving features.
 * 
 * 
 * 
 */

void WDT_off(void)  //Function to turn off watchdog timer
{
/* Global interrupt should be turned OFF here if not
already done so */
/* Clear WDRF in MCUSR */
MCUSR &= ~(1<<WDRF);
/* Write logical one to WDCE and WDE */
/* Keep old prescaler setting to prevent unintentional
time-out */
WDTCSR |= (1<<WDCE) | (1<<WDE);
/* Turn off WDT */
WDTCSR = 0x00;
/* Global interrupt should be turned ON here if
subsequent operations after calling this function DO
NOT require turning off global interrupt */
}

void setupPowerSaving() 
{
  //Turn off the Watchdog Timer
  WDT_off;
  
  //Modify PRR to shut down TWI
  PRR |= PRR_TWI_MASK;
  
  //Modify PRR to shut down SPI
  PRR |= PRR_SPI_MASK;
  
  // Modify ADCSRA to disable ADC,
  ADCSRA &= ~(ADCSRA_ADC_MASK);
  
// then modify PRR to shut down ADC
   PRR |= PRR_ADC_MASK;

// Set the SMCR to choose the IDLE sleep mode
  SMCR |= SMCR_IDLE_MODE_MASK;
// Do not set the Sleep Enable (SE) bit yet
// Set Port B Pin 5 as output pin, then write a logic LOW
// to it so that the LED tied to Arduino's Pin 13 is OFF.
   DDRB |= 0b00100000;
   PORTB &= 0b11011111;
}

void putArduinoToIdle()
{
  // Modify PRR to shut down TIMER 0, 1, and 2
  PRR |= PRR_TIMER0_MASK;
  PRR |= PRR_TIMER1_MASK;
  PRR |= PRR_TIMER2_MASK;
  
// Modify SE bit in SMCR to enable (i.e., allow) sleep
  SMCR |= SMCR_SLEEP_ENABLE_MASK;
  
// The following function puts ATmega328P’s MCU into sleep;
// it wakes up from sleep when USART serial data arrives
  sleep_cpu();
  
// Modify SE bit in SMCR to disable (i.e., disallow) sleep
  SMCR &= ~(SMCR_SLEEP_ENABLE_MASK);
  
// Modify PRR to power up TIMER 0, 1, and 2
   PRR &= ~(PRR_TIMER0_MASK);
   PRR &= ~(PRR_TIMER1_MASK);
   PRR &= ~(PRR_TIMER2_MASK);
}
 
/*
 * 
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse
  //
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks; 
  statusPacket.params[1] = rightForwardTicks; 
  statusPacket.params[2] = leftReverseTicks; 
  statusPacket.params[3] = rightReverseTicks; 
  statusPacket.params[4] = leftForwardTicksTurns; 
  statusPacket.params[5] = rightForwardTicksTurns; 
  statusPacket.params[6] = leftReverseTicksTurns; 
  statusPacket.params[7] = rightReverseTicksTurns; 
  statusPacket.params[8] = forwardDist; 
  statusPacket.params[9] = reverseDist;  
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf(char *format, ...) 
{
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD &= 0b11110011;
  PORTD |= 0b00001100; 
  
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  switch(dir)
  {
    case FORWARD:
      leftForwardTicks++;
      break;
    case BACKWARD:
      leftReverseTicks++;
    case LEFT:
      leftReverseTicksTurns++;
      break;
    case RIGHT:
      leftForwardTicksTurns++;
      break; 
  }

  if (dir == FORWARD) { 
    forwardDist = (unsigned long) (((leftForwardTicks+6.0)/COUNTS_PER_REV) * WHEEL_CIRC); 
  }
  if (dir == BACKWARD) { 
    reverseDist = (unsigned long) (((leftReverseTicks-3.0)/COUNTS_PER_REV) * WHEEL_CIRC); 
  }


}

void rightISR()
{
  switch(dir) 
  { 
    case FORWARD:
      rightForwardTicks++;
      break;
    case BACKWARD:
      rightReverseTicks++;
    case LEFT:
      rightForwardTicksTurns++;
      break;
    case RIGHT:
      rightReverseTicksTurns++;
      break; 
  }
  
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  cli(); 
  EIMSK = 0b00000011;
  EICRA = 0b00001010;
  sei();
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR(INT0_vect)
{
  leftISR();
}

ISR(INT1_vect)
{
  rightISR();
}



// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
  
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count=0;

  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2In - pIN 11, PB3, OC2A
   */
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex reverse "dist" cm. 
// We decided not to use the "speed" parameter; 
// values fed to motors are fixed from calibration
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed) 
{

  dir = BACKWARD;

  if (dist > 0)
    deltaDist = dist;
  else
    deltaDist = 99999999;

  newDist = reverseDist + deltaDist;

  
  analogWrite(LF, 245);
  analogWrite(RF, 236);
  analogWrite(LR,0);
  analogWrite(RR, 0);
}

// Move Alex forward "dist" cm at speed "speed".
// We decided not to use the "speed" parameter; 
// values fed to motors are fixed from calibration
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void forward(float dist, float speed) 
{

  dir = FORWARD;
  
  int val = pwmVal(speed);

  if (dist > 0)
    deltaDist = dist;
  else
    deltaDist = 99999999;

  newDist = forwardDist + deltaDist;

  // For now we will ignore dist and 
  // reverse indefinitely. We will fix this
  // in Week 9.
  
  analogWrite(LR, 250);
  analogWrite(RR, 240);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
}



unsigned long computeDeltaTicks(float ang)
{
   //Assumption: Angular dist = Linear dist moved in one wheel revolution
   // This is for 360 degrees. For any degrees, it will be (ang * alexCirc) / (360 * WHEEL_CIRC)
   // To convert to ticks, multiply by COUNTS_PER_REV

   unsigned long ticks = (unsigned long) ((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));

   return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed) //supposed to be left
{

  dir = LEFT;
  
  int val = pwmVal(speed);

  if (ang == 0)
    deltaTicks = 99999999;
  else
    deltaTicks = computeDeltaTicks(ang) * 0.7;

  targetTicks = leftReverseTicksTurns + deltaTicks;

  analogWrite(LR, val);
  analogWrite(RF, val);
  analogWrite(LF, 0);
  analogWrite(RR, 0);
}

// Turn Alex right "ang" degrees at rspeed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed) //supposed to be right
{
  
  dir = RIGHT;
  
  int val = pwmVal(speed);

  if (ang == 0)
    deltaTicks = 99999999;
  else
    deltaTicks = computeDeltaTicks(ang) * 0.3;

  targetTicks = rightReverseTicksTurns + deltaTicks;
 
  analogWrite(RR, val);
  analogWrite(LF, val);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{

  dir = STOP;
  analogWrite(LF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  analogWrite(RR, 0);
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;
  leftForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightForwardTicksTurns = 0;
  rightReverseTicksTurns = 0;

  leftRevs = 0;
  rightRevs = 0;
  forwardDist=0;
  reverseDist=0; 
}


// Clears one particular counter
void clearOneCounter(int which)
{
  switch(which)
  {
    case 0:
      clearCounters();
      break;

//    case 1:
//      leftTicks=0;
//      break;
//
//    case 2:
//      rightTicks=0;
//      break;
//
//    case 3:
//      leftRevs=0;
//      break;
//
//    case 4:
//      rightRevs=0;
//      break;
//
//    case 5:
//      forwardDist=0;
//      break;
//
//    case 6:
//      reverseDist=0;
//      break;
  }
}


// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK(); // send from Arduino to RPI 
       // forward((float) command->params[0], (float) command->params[1]); // get distance and speed
        forward((float) command->params[0], (float) 100); // get distance
        break;
   case COMMAND_REVERSE:
        sendOK();
        //reverse((float) command->params[0], (float) command->params[1]);
        reverse((float) command->params[0], (float) 100);
        break;
   case COMMAND_TURN_LEFT:
        sendOK();
        //left((float) command->params[0], (float) command->params[1]);
        left((float) command->params[0], (float) 100);
        break;
   case COMMAND_TURN_RIGHT:
        sendOK();
        //right((float) command->params[0], (float) command->params[1]);
        right((float) command->params[0], (float) 100);
        break;
   case COMMAND_STOP:
        sendOK();
        stop(); 
        break;
   case COMMAND_GET_STATS:
        sendOK();
        sendStatus();
        break;
   case COMMAND_CLEAR_STATS:
        sendOK();
        clearOneCounter(command->params[0]);
        break;
   case COMMAND_GET_COLOUR: 
        sendOK();
        GetDistance();
        GetColours();
        break;
   case COMMAND_MUSIC:
        sendOK();
        GetMusic();
        break;
   
   default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

//Colour Sensing
void GetColours()  
{    
  digitalWrite(s2, LOW);                                           //S2/S3 levels define which set of photodiodes we are using LOW/LOW is for RED LOW/HIGH is for Blue and HIGH/HIGH is for green 
  digitalWrite(s3, LOW);                                           
  Red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);       //here we wait until "out" go LOW, we start measuring the duration and stops when "out" is HIGH again, if you have trouble with this expression check the bottom of the code
  delay(50);
  digitalWrite(s3, HIGH);                                         //Here we select the other color (set of photodiodes) and measure the other colors value using the same techinque
  Blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  delay(50);  
  digitalWrite(s2, HIGH);  
  Green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  delay(50);
  
    if (Green == 0)
    {
      dbprintf("Error");
      dbprintf(Green);
      
    }
    else if (Green >= 600)
    {
      dbprintf("Red");
      dbprintf("%d", (int)Green);
    }
    else if (Green <= 450)
    {
      dbprintf("Green");   
       dbprintf("%d", (int)Green);  
    }   
    else 
    {
      dbprintf("Retake");
       dbprintf("%d", (int)Green);
    }
}

//Get Distance
void GetDistance()
{
    //Ultrasonic  
  // Clears the trigPin condition
/* 
  PORTC &= TRIG_PIN_MASK; //set trigpin low
  delayMicroseconds(2);
  PORTC |= TRIG_PIN_MASK; //set trigpin highs
  delayMicroseconds(10);
  PORTC &= TRIG_PIN_MASK; //set trigpin low */

   // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW); 
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH); 
  obs_distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  delay(500);  
  dbprintf("%d", (int)obs_distance);

 // Serial.println((int)obs_distance);
            
}

//Get Music
void GetMusic() {
  disco(); 
  stopmusic();
  delay(100);
}

//Flash LED
void flash() {
  analogWrite(red, random(0, 255));
  analogWrite(green, random(0, 255));
  analogWrite(blue, random(0, 255));
  delay(10);
}

void stopmusic() {
  noTone(speaker);
  analogWrite(red, 0);
  analogWrite(green, 0);
  analogWrite(blue, 0);
}

void disco() {
   tone(speaker,  NOTE_B3, shortTone);
 delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash();
 tone(speaker,  NOTE_B3, shortTone);
   delay(shortTone);
  noTone(speaker);
  delay(standardDelay); flash();
tone(speaker,  NOTE_B3, shortTone);
  delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash();
 tone(speaker,  NOTE_B3, shortTone);
 delay(shortTone);
 noTone(speaker);
  delay(standardDelay); flash();
tone(speaker,  NOTE_B3, longTone); 
 delay(longTone);
 noTone(speaker);
 delay(delayBetweenBars);
 
    tone(speaker,  NOTE_B3, shortTone);
 delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash(); 
 tone(speaker,NOTE_B3, shortTone);
 delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash();
 tone(speaker,  NOTE_B3, shortTone);
 delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash();
 tone(speaker,  NOTE_B3, shortTone);
   delay(shortTone);
  noTone(speaker);
  delay(standardDelay); flash();
tone(speaker,  NOTE_B3, shortTone);
  delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash();
 tone(speaker,  NOTE_B3, shortTone);
 delay(shortTone);
 noTone(speaker);
  delay(standardDelay); flash();
tone(speaker,  NOTE_B3, longTone); 
 delay(longTone);
 noTone(speaker);
 delay(standardDelay); flash();
 
     tone(speaker,  NOTE_E4, shortTone);
 delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash(); 
 tone(speaker,NOTE_E4, shortTone);
 delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash();
 tone(speaker,  NOTE_E4, shortTone);
 delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash();
 tone(speaker,  NOTE_E4, shortTone);
   delay(shortTone);
  noTone(speaker);
  delay(standardDelay); flash();
tone(speaker,  NOTE_E4, shortTone);
  delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash();
 tone(speaker,  NOTE_E4, shortTone);
 delay(shortTone);
 noTone(speaker);
  delay(standardDelay); flash();
tone(speaker,  NOTE_E4, longTone); 
 delay(longTone);
 noTone(speaker);
 delay(standardDelay); flash();
 
 
     tone(speaker,  NOTE_D4, shortTone);
 delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash(); 
 tone(speaker,NOTE_D4, shortTone);
 delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash();
 tone(speaker,  NOTE_D4, shortTone);
 delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash();
 tone(speaker,  NOTE_D4, shortTone);
   delay(shortTone);
  noTone(speaker);
  delay(standardDelay); flash();
tone(speaker,  NOTE_D4, shortTone);
  delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash();
 tone(speaker,  NOTE_D4, shortTone);
 delay(shortTone);
 noTone(speaker);
  delay(standardDelay); flash();
tone(speaker,  NOTE_D4, longTone); 
 delay(longTone);
 noTone(speaker);
 delay(standardDelay); flash();

tone(speaker, NOTE_A3, longTone);
delay(longTone);
 noTone(speaker);
 delay(standardDelay); flash();

 tone(speaker,  NOTE_B3, shortTone);
 delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash();
 tone(speaker,  NOTE_B3, shortTone);
   delay(shortTone);
  noTone(speaker);
  delay(standardDelay); flash();
tone(speaker,  NOTE_B3, shortTone);
  delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash();
 tone(speaker,  NOTE_B3, shortTone);
 delay(shortTone);
 noTone(speaker);
  delay(standardDelay); flash();
tone(speaker,  NOTE_B3, longTone); 
 delay(longTone);
 noTone(speaker);
 delay(delayBetweenBars);
 
    tone(speaker,  NOTE_B3, shortTone);
 delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash(); 
 tone(speaker,NOTE_B3, shortTone);
 delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash();
 tone(speaker,  NOTE_B3, shortTone);
 delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash();
 tone(speaker,  NOTE_B3, shortTone);
   delay(shortTone);
  noTone(speaker);
  delay(standardDelay); flash();
tone(speaker,  NOTE_B3, shortTone);
  delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash();
 tone(speaker,  NOTE_B3, shortTone);
 delay(shortTone);
 noTone(speaker);
  delay(standardDelay); flash();
tone(speaker,  NOTE_B3, longTone); 
 delay(longTone);
 noTone(speaker);
 delay(standardDelay); flash();
 
 tone(speaker, NOTE_E4, longTone);
 delay(longTone);
 noTone(speaker);
 delay(standardDelay); flash();

 tone(speaker,  NOTE_B3, shortTone);
 delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash();
 tone(speaker,  NOTE_B3, shortTone);
   delay(shortTone);
  noTone(speaker);
  delay(standardDelay); flash();
tone(speaker,  NOTE_B3, shortTone);
  delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash();
 tone(speaker,  NOTE_B3, shortTone);
 delay(shortTone);
 noTone(speaker);
  delay(standardDelay); flash();
tone(speaker,  NOTE_B3, longTone); 
 delay(longTone);
 noTone(speaker);
 delay(delayBetweenBars);
 
    tone(speaker,  NOTE_B3, shortTone);
 delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash(); 
 tone(speaker,NOTE_B3, shortTone);
 delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash();
 tone(speaker,  NOTE_B3, shortTone);
 delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash();
 tone(speaker,  NOTE_B3, shortTone);
   delay(shortTone);
  noTone(speaker);
  delay(standardDelay); flash();
tone(speaker,  NOTE_B3, shortTone);
  delay(shortTone);
 noTone(speaker);
 delay(standardDelay); flash();
 tone(speaker,  NOTE_B3, shortTone);
 delay(shortTone);
 noTone(speaker);
  delay(standardDelay); flash();
tone(speaker,  NOTE_B3, longTone); 
 delay(longTone);
 noTone(speaker);
 delay(standardDelay); flash();

tone(speaker, NOTE_E4, longTone); 
 delay(longTone);
 noTone(speaker);
 delay(standardDelay); flash();
  
}

void setup() {
  // put your setup code here, to run once:

  //Compute the diagonal
  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = PI * alexDiagonal;
   
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  setupPowerSaving();
  sei();

   DDRC |= TRIG_PIN_MASK; //Sets the trigpin as an OUTPUT
   DDRC &= ~(ECHO_PIN_MASK); //Sets the echoPin as an INPUT
   //pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  // pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
   pinMode(s0,OUTPUT);    //pin modes
   pinMode(s1,OUTPUT);
   pinMode(s2,OUTPUT);
   pinMode(s3,OUTPUT);
   pinMode(out,INPUT);
   pinMode(red, OUTPUT);
   pinMode(green, OUTPUT);
   pinMode(blue, OUTPUT);
   digitalWrite(s0,HIGH); //Putting S0/S1 on HIGH/HIGH levels means the output frequency scalling is at 100% (recommended)
   digitalWrite(s1,HIGH); //LOW/LOW is off HIGH/LOW is 20% and LOW/HIGH is  2%
  
  //dbprintf("PI is %3.2f\n", PI);

 //forward(20, 85);
//reverse(20,80);
//right(20,100);
  //GetDistance();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}




void loop() {

  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      }  

      
  if (dir == STOP)
  {
    putArduinoToIdle();
  } 
      
// Check if commanded distance has been reached
// Note: Distance calibrated from middle of the wheel (corresponds approximately to middle of LIDAR)
  if (deltaDist > 0)
  {
    if (dir == FORWARD)
    {
      if (forwardDist >= newDist) 
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == BACKWARD)
    {
      if (reverseDist >= newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
      
    }
    else if (dir == STOP)
    {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  } 

  if (deltaTicks > 0)
  {
    if (dir == LEFT)
    {
      if (leftReverseTicksTurns >= targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == RIGHT)
    {
      if (rightReverseTicksTurns >= targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == STOP)
    {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }


        
}
