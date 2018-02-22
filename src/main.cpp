#include <Arduino.h>
#include <SPI.h> // Includes SPI libary (port 10, 11, 12, 13)
#include "nRF24L01.h"
#include "RF24.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

/* 
SPI: 1) Data from master to slave (Mosi) 
        2) Data from slave to master (Miso)
        3) Clock (synchronises the data,eg. if the clock rises the slave will save the data )
        4) Slave select determines which slave is adressed
        So,
        pin 10 = slave select
        pin 11 = miso
        pin 12 = mosi
        pin 13 = clock
Not SPI
        pin 9  = enables chip or not to allow radio transmission
        pin 7  = determines ping/pong
        pin A0 = Throttle
        
*/

#define PAYLOAD_SIZE 8
#define TIMERCOUNTERVALUE_500HZ 0x83
#define DEBUG__

RF24 radio(9, 10);

volatile uint8_t packetReceived = 0;
volatile uint8_t signalStrength = 0;
uint8_t signalStrengthArray[256] = {0};
uint8_t pos = 0;
uint16_t sum = 0;
uint8_t txBatteryLevel = 0;
uint8_t oldtxBatteryLevel = 0;
uint8_t rxBatteryLevel = 0;
uint8_t oldrxBatteryLevel = 0;
uint8_t oldsignalStrength = 0;

// Set input pins
const int pinThrottle = A3;
const int pinRoll = A0;
const int pinPitch = A1;
const int pinYaw = A2;
const int pinBattery = A7;
//const int pinSwitch1 = 8;
//const int pinSwitch2 = 7;

const uint16_t rollOffset = 0;
const uint16_t pitchOffset = 0;
const uint16_t yawOffset = 0;
const uint16_t throttleOffset = 0;

bool success;

uint16_t payload[PAYLOAD_SIZE];
uint8_t acknowledgePayload[4];

#include <LiquidCrystal_I2C.h>

// Set display pins
#define I2C_ADDR    0x3F // <<----- Add your address here.  Find it from I2C Scanner
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

LiquidCrystal_I2C	lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

void startRadio(void)
{
  radio.begin();
  radio.setRetries(1, 5); // 5 times 5 micro seconds retries

  uint64_t address = 0x007FFFFFFF;
  radio.setChannel(101);
  radio.openWritingPipe(address); // Open pipe 0 with the specified address
  radio.setPALevel(RF24_PA_HIGH); // Set power level to high
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
}

void startScreen(void)
{
  lcd.begin(20,4);  
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.home(); // go home
  lcd.print("Battery tx:     V"); //location 12-15 can be used for 4 voltage digits
  lcd.setCursor ( 0, 1 );         // go to the 2nd line
  lcd.print("Battery QC:     V"); //location 12-15 can be used for 4 voltage digits
  lcd.setCursor ( 0, 2 );         // go to the 3rd line
  lcd.print("T:      R: ");
  lcd.setCursor ( 0, 3 );         // go to the 4th line
  lcd.print("P:      Y: ");

}

void updateScreen(void)
{
  // Display battery level
  double Batterylevel = (double)(analogRead(pinBattery)*(5.0/1024.0)*(119.0/51.0)*1.01); //in mV
  lcd.setCursor(11,0);
  if (Batterylevel<10.0) {
    lcd.print(" " + (String) Batterylevel);
  } else {
    lcd.print((String) Batterylevel);
  }

  // Display TRPY data
  for (int i=0; i<4; i++){
    
    //set cursor position
    if (i==0) lcd.setCursor(3,2);       //throttle position
    else if (i==1) lcd.setCursor(11,2); //roll position
    else if (i==2) lcd.setCursor(3,3);  //pitch position
    else lcd.setCursor(11,3);           //yaw position

    //set value
    if (payload[i]>1000) {
      lcd.print((String) payload[i] + " ");
    } else if(payload[i]>100) {
      lcd.print(" " + (String) payload[i] + " ");
    } else {
      lcd.print("  " + (String) payload[i] + " ");
    }
  }
}

uint8_t movingAvg(uint8_t *ptrArrNumbers, uint16_t *ptrSum, uint8_t pos, uint16_t len, uint8_t nextNum)
{
  //Subtract the oldest number from the prev sum, add the new number
  *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
  //Assign the nextNum to the position in the array
  ptrArrNumbers[pos] = nextNum;
  //return the average
  return *ptrSum / len;
}

void setup(void)
{
#ifdef DEBUG__
  Serial.begin(9600);
#endif
  // Configure the switches
  //pinMode(pinSwitch1, INPUT_PULLUP);
  //pinMode(pinSwitch2, INPUT_PULLUP);

  // Start screen
  startScreen();

  // Configure the transceiver
  startRadio();

  // set up interrupt so that readings are done with 500Hz
#ifndef DEBUG__
  noInterrupts();
  TCCR2B = 0;
  TCCR2B |= 1 << WGM12;
  TCCR2B |= 1 << CS12;
  TCNT2 = TIMERCOUNTERVALUE_500HZ;
  TIMSK2 |= 1 << OCIE2A;

  interrupts();
#endif
}

ISR(TIMER2_COMPA_vect)
{
  TCNT2 = TIMERCOUNTERVALUE_500HZ;

  payload[0] = (uint16_t)analogRead(pinThrottle);
  payload[1] = (uint16_t)analogRead(pinRoll) - rollOffset;
  payload[2] = (uint16_t)analogRead(pinPitch) - pitchOffset;
  payload[3] = (uint16_t)analogRead(pinYaw) - yawOffset;
  //payload[4] = (uint16_t)(!digitalRead(pinSwitch2) & 0x01) << 1 | (!digitalRead(pinSwitch2) & 0x01); // inverted due to pullup resistor

  success = radio.write(&payload, PAYLOAD_SIZE);

  if (radio.isAckPayloadAvailable())
  {
    success = radio.read(&acknowledgePayload, 4);
    rxBatteryLevel = acknowledgePayload[1];
    packetReceived = 100;
  }
  else
  {
    packetReceived = 0;
  }

  signalStrength = movingAvg(signalStrengthArray, &sum, pos, sizeof(signalStrengthArray), packetReceived);
  pos++;
}

void loop(void)
{

#ifdef DEBUG__
  payload[0] = 1024 - ((uint16_t)analogRead(pinThrottle)) - throttleOffset;
  payload[1] = 1024 - (uint16_t)analogRead(pinRoll) - rollOffset;
  payload[2] = 1024 - (uint16_t)analogRead(pinPitch) - pitchOffset;
  payload[3] = 1024 - (uint16_t)analogRead(pinYaw) - yawOffset;
  //payload[4] = (uint16_t)(!digitalRead(pinSwitch2) & 0x01) << 1 | (!digitalRead(pinSwitch2) & 0x01); // inverted due to pullup resistor
  success = radio.write(&payload, PAYLOAD_SIZE);
  if (radio.isAckPayloadAvailable())
  {
    success = radio.read(&acknowledgePayload, 4);
    packetReceived = 100;
    // digitalWrite(pinSwitch1,LOW);
  }
  else
  {
    packetReceived = 0;
  }
  signalStrength = movingAvg(signalStrengthArray, &sum, pos, sizeof(signalStrengthArray), packetReceived);
  pos++;

  if (pos==1) updateScreen(); //only update screen every 256 cycles

  // Serial.println(signalStrength);
  // Serial.print("\t");
  //  Serial.print(analogRead(pinThrottle));
  //  Serial.print("\t");
  //  Serial.print((int16_t)(analogRead(pinRoll) - rollOffset));
  //  Serial.print("\t");
  //  Serial.print((int16_t)(analogRead(pinYaw) - yawOffset));
  //  Serial.print("\t");
  //  Serial.println((int16_t)(analogRead(pinPitch) - pitchOffset)); //acknowledgePayload[1] <<8 | acknowledgePayload[0]);
#endif
}