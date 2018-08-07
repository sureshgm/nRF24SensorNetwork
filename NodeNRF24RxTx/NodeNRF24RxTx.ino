/*
 Copyright (C) 2012 James Coliz, Jr. <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 
 Update 2014 - TMRh20
 */

/**
 * Simplest possible example of using RF24Network,
 *
 * RECEIVER NODE
 * Listens for messages from the transmitter and prints them out.
 */
#include <avr/pgmspace.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <EEPROM.h>         //EEPROM functions

#define SENSOR_DATA 01
//#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))   // clear bit in byte at sfr address
//#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= ~_BV(bit))   // clear bit in byte at sfr address

RF24 radio(9,10);                // nRF24L01(+) radio attached using Getting Started board 

RF24Network network(radio);      // Network uses that radio

// Node 0 is the gateway - USe other file / program for gateway device.
uint16_t GATEWAY = 0;

/*
 * Function prototypes
 */
void ReceivePayload(void);
void formPacket (uint16_t node_ID);
void SendPayload(uint16_t to_node);
void ProcPayload (void);
void printpayload(void);
void SleepNode(byte st); 
void burn8Readings(void);
void getEEPROMValues(void);
bool getAnswer(void);
String getMode() ;
String getSleepInterval(void);

/*
 * Global variables
 */
const uint16_t this_node;    // define this as Node address, Node 1 is always gateway Address of our node in Octal format 
byte mMode;                 //used to track whether in end device mode or router mode
byte sInterval;             //used to set interval between transmitting data
float iREF;                 //Actual measured voltage value of the internal 1.1V reference

const int aout1_pin = 5;    // pins 5 and 6 provide PWM frequency of 980Hz, on 3, 9, 10, 11 490 Hz
const int aout2_pin = 6;

const int ain1_pin = 1;     // mapping the Ain values to analog pins
const int ain2_pin = 2;

const int din1_pin = 8;
const int din2_pin = 4;

const int dout1_pin = 7;
   
struct payload_t {                    // Structure of our payload
  int16_t ain1, ain2, ain3;           // 6 bytes
  int16_t aout1, aout2;               // 4 bytes
  unsigned char din1, din2, din3, din4;        // 4 bytes
  unsigned char dout1, dout2;                  // 2 bytes
  long counter;                       // 4 bytes
};

volatile payload_t payload;

unsigned long last_sent;             // When did we last send?
const unsigned long interval = 10;   // How often to send 'hello world to the other unit

volatile byte portBstatus;
volatile byte eventFlag;

const String enter = ("Enter 'Y' for YES or any other for NO:"); //F() macro tells IDE to store string in flash memory and not SRAM
const String invalid = ("Invalid entry, default to ");
const String would = ("Would you like to update the ");


//This is for sleep mode. It is not really required, as users could just use the number 0 through 10
typedef enum { wdt_16ms = 0, wdt_32ms, wdt_64ms, wdt_128ms, wdt_250ms, wdt_500ms, wdt_1s, wdt_2s, wdt_4s, wdt_8s } wdt_prescalar_e;


/*
 * main program
 */

void setup(void)
{
  analogReference(INTERNAL); //set the ADC reference to internal 1.1V reference
  burn8Readings();
  
  Serial.begin(115200);
  Serial.println("RF24Network Sensor Network");
  
  Serial.print("This Node# ");
  Serial.println(this_node);
  
  pinMode(dout1_pin, OUTPUT);
  pinMode(din1_pin, INPUT);
  digitalWrite(din1_pin, HIGH);
  pinMode(din2_pin, INPUT);
  digitalWrite(din2_pin, HIGH);

  checkNodeAddress();     //Function used for adding and getting module settings from EEPROM
/*  sbi(PCMSK0, PCINT0);    // set the inetrrupt control bit pin PB0
  sbi(PCIFR, PCINT0);       // clear any outstanding interrupt
  sbi(PCICR, PCIE0);        // enable interrupt PCI0
*/
  *digitalPinToPCMSK(din1_pin) |= bit (digitalPinToPCMSKbit(din1_pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(din1_pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(din1_pin)); // enable interrupt for the group  
  
  SPI.begin();
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  network.begin(/*channel*/ 90, /*node address*/ this_node);
  radio.startListening();
/******************************** This is the configuration for sleep mode ***********************/
  network.setup_watchdog(wdt_1s);   //watchdog timer will wake MCU & radio every second to send sleep payload, then go back to sleep  
}

/*
 * Loop funcion
 */
void loop(void) {
  unsigned long now = millis();              // If it's time to send a message, send it!  
  network.update();                           // Check the network regularly

  /* Transmit packet*/
   if ((now - last_sent >= interval) || eventFlag)   {               //  , replace this line for '1' 
      if(eventFlag) {
        eventFlag = 0;
        delay(20);
        *digitalPinToPCMSK(din1_pin) |= bit (digitalPinToPCMSKbit(din1_pin));  // enable pin
      }
      last_sent = now;
      formPacket(SENSOR_DATA);
      if(this_node) {  
        SendPayload(GATEWAY);
      }
      else
        Serial.println("change software to gateway");        
   }
  delay(500);                     // delay to receive any packet for this node
  while( network.available()) {     // Is there anything ready for us?
    ReceivePayload();
    ProcPayload();
    delay(200);       // complete any serial prints pending  
  }

/***************************** CALLING THE NEW SLEEP FUNCTION ************************/  
  digitalWrite(dout1_pin, LOW);
  SleepNode(sInterval);
}
/*
 * SleepNode()
 */
void SleepNode(byte sleeptime)
{
     radio.stopListening();         // Switch to PTX mode. Payloads will be seen as ACK payloads, and the radio will wake up
     radio.powerDown();             // Power down nRF24L01 before going to sleep
     ADCSRA &= ~(1<<ADEN);          // Turn off ADC before going to sleep (set ADEN bit to 0).
     network.sleepNode((int)sleeptime ,0);        // Sleep the node for 10 cycles of 1 second intervals
     ADCSRA |= (1<<ADEN);           // Turn the ADC back on
     radio.powerUp();  
     radio.startListening();
}

/*
 * Function to receive packets
 */
void ReceivePayload(void)
{
    RF24NetworkHeader rheader;        // If so, grab it and print it out
    network.peek(rheader);
    network.read(rheader,&payload,sizeof(payload));
    Serial.print("\n\r Node#:");
    Serial.print(rheader.from_node);    
    printpayload();
}

/*
 * Function to transmit Packets
 */

 void SendPayload(uint16_t to_node)
 {
    bool ok = 0;
    radio.stopListening();                       // start transmission
    RF24NetworkHeader theader(/*to node*/ to_node, /*type*/ 'S' /*Time*/);
    ok = network.write(theader, &payload, sizeof(payload));
    delay(5);
    radio.startListening();                       // start listening
    
    if (ok) {                                   // print transmission status
      Serial.print("Sent to:");
      Serial.println(to_node);
    }
    else  {
      Serial.print("failed to send "); 
      Serial.print(sizeof(payload));
      Serial.println(" bytes to GW");
    } 
 }

/*
 * From payload by reading sensors analod and digital data.
 */
void formPacket (uint16_t data_type)
{
    static unsigned long pktcnt;
    
    payload.ain1 = analogRead(ain1_pin);    // read sensor analog value at ain1_pin
    payload.ain2 = analogRead(ain2_pin);
    payload.ain2 = 0;
    
    payload.din1 = (digitalRead(din1_pin) != HIGH) ? 0 : 1;   // read digital sensor value at din1_pin
    payload.din2 = (digitalRead(din2_pin) != HIGH) ? 0 : 1;

    payload.counter = pktcnt++;  
    Serial.print("Sending: ");
    printpayload();
}
/*
 * Function to print the payload for debug purpose only
 */
void printpayload(void)
{
   
    Serial.print(" ,packet #");
    Serial.print(payload.counter);
    
    Serial.print(" ,Ain1:");
    Serial.print(payload.ain1);
    Serial.print(" ,Ain2:");
    Serial.print(payload.ain2);
    Serial.print(" ,Ain3:");
    Serial.print(payload.ain3);
        
    Serial.print(" ,Aout1:");
    Serial.print(payload.aout1);
    Serial.print(" ,Aout2:");
    Serial.print(payload.aout2);
    
    Serial.print(" ,Din1:");
    Serial.print(payload.din1);
    Serial.print(" ,Din2:");
    Serial.print(payload.din2);
    Serial.print(" ,Din3:");
    Serial.print(payload.din3);
    Serial.print(" ,Din4:");
    Serial.print(payload.din4);
        
    Serial.print(" ,Dout1:");
    Serial.print(payload.dout1); 
    Serial.print(" ,Dout2:");
    Serial.println(payload.dout2);         
}
/*
 * Process the message to set actuators
 */
void ProcPayload (void)
{
    analogWrite(aout1_pin, payload.aout1);
    analogWrite(aout2_pin, payload.aout2);

    if(payload.dout1)
      digitalWrite(dout1_pin, HIGH);
    else
      digitalWrite(dout1_pin, LOW);
}


ISR (PCINT0_vect)     // Interrupt service routine, This code is executed whenever the button on th board is pressed
{
  portBstatus = PINB;   // PINB is the port tyhat contains the momentary status of PIN8
  eventFlag = 1;        // set the event flag for LED ON/OFF
  *digitalPinToPCMSK(din1_pin) &= ~bit (digitalPinToPCMSKbit(din1_pin));  // disable pin
}

// reference to this section: 
// https://github.com/ForceTronics/nRF24L01_Wireless_Sensor_Dev_Board/blob/master/nRF24_Network_Router_End_Device_NoLibrary

//This function makes 8 ADC measurements but does nothing with them
//Since after a reference change the ADC can return bad readings. This function is used to get rid of the first 
//8 readings to ensure next reading is accurate
void burn8Readings(void) {
  for(int i=0; i<8; i++) {
    analogRead(A0);
    analogRead(A1);
  }
}


//The function allows you to view and set the settigns of the wireless sensor module
//Settings include module network address, int. ADC ref value, battery volt cal factor, transmit interval, mode
//Settings are checked and set via serial monitor. when entering settings be sure not to use CR or line ending
//Settings need to be set when the first time the module is used. After that they can be viewed or set again by connecting 
//digital pin 8 to ground before power up or reset
void checkNodeAddress() {
  pinMode(8, INPUT_PULLUP);   // set pin 8 as an input
  int val;                    // variable to store node address
  byte cAddr = 128;           // this variable is used to see if this is the first time the module is being used
  byte tInterval;             // set the transmit time interval
  byte mode;                  // set to router or end device mode
  float ref;                  //set measured internal reference value
  // ********************************************************************************

  //if this is either the first time the module is used or if pin 8 is connecting to ground enter settings mode
  if(EEPROM.get(0,cAddr)!= 128 || !digitalRead(8)) {
    digitalWrite(7,HIGH);     // in settings mode so turn on status LED

    //the following code reads the current settings from EEPROM and puts them in local variables
    Serial.println(F("Current settings"));
    Serial.print(F("Node address: "));
    Serial.println(EEPROM.get(1,val),OCT);
    Serial.print(F("Internal ref: "));
    Serial.println(EEPROM.get(3,ref));
    Serial.print(F("Sleep interval: "));
    Serial.println(getSleepInterval());
    Serial.print(F("Mode: "));
    Serial.println(getMode());
    
    //Update Node address to EEPROM #################################################
    Serial.print(would);
    Serial.print(F("Node Address? "));
    Serial.println(enter);
    if(getAnswer()) {
      Serial.println(F("New Node address"));
      while (!Serial.available()) { }
      val = Serial.parseInt();
      if(val >= 0) {
        EEPROM.put(1, val);
      }
      else {                         //if zero is entered it is invalid since coordinator is zero
        Serial.print(invalid);
        Serial.println("01");
        val = 01;
        EEPROM.put(1, val);
      }
    }
    // update internal refernce voltage ########################################################
    Serial.print(would);
    Serial.print("internal ref voltage? ");
    Serial.println(enter);
    if(getAnswer()) {
      Serial.println(F("New internal ref voltage"));
      while (!Serial.available()) { }
      ref = Serial.parseFloat();
      if(ref >= 1.0 && ref <= 1.2) { //ADC reference value must be between 1.0 and 1.2
        EEPROM.put(3, ref);
      }
      else {
        Serial.print(invalid);
        Serial.println("1.1");
        ref = 1.1;
        EEPROM.put(3, ref);
      }
    }

    // update data transmit interval #############################################
    Serial.print(would);
    Serial.print("Transmit Period? ");
    Serial.println(enter);
    if(getAnswer()) {
      Serial.println(F("Enter data Transmit period in seconds"));
      while (!Serial.available()) { }
      tInterval = Serial.parseInt();
      if(tInterval >= 0 || tInterval < 4) { //check that entry was valid
        EEPROM.put(7, sInterval);
      }
      else {
        Serial.print(invalid);
        tInterval = 3;
        Serial.println(tInterval);
        EEPROM.put(7, sInterval);
      }
    }

    // update data transmit interval #############################################
    Serial.print(would);
    Serial.print(F("mode setting? "));
    Serial.println(enter);
    if(getAnswer()) {
      Serial.println(F("Enter 0 for end device and 1 for router"));
      while (!Serial.available()) { }
      mode = Serial.parseInt();
      if(mode == 0 || mode == 1) { //check that entry was valid
        EEPROM.put(9, mode);
      }
      else {
        Serial.print(invalid);
        Serial.println("0");
        mode = 0;
        EEPROM.put(9, mode);
      }
    }

    // ################################################################
    getEEPROMValues();          //gets settings from EEPROM and stor in global variables
                                //the following code prints out current settings from global variables
    Serial.print("Node address: ");
    Serial.println(this_node, OCT);
    Serial.print("Internal ref voltage: ");
    Serial.println(iREF); 
    Serial.print("Transmit Period: ");
    Serial.println(sInterval);
    Serial.print("Mode setting: ");
    Serial.println(mMode);
    cAddr = 128;                //write '128' to EEPROM to show that settings have been entered at least once
    EEPROM.put(0, cAddr);
  }

  else {
     //not in settings mode so just get settings from EEPROM and store in global variables
    getEEPROMValues();
  }
  //*********************************************************************************
}

//gets transmit interval setting from EEPROM and prints it out to user
String getSleepInterval() {
  byte i;
  String m = " min";
  EEPROM.get(7,i);
  if(i==0) {
    return "1 sec";
  }
  else if (i==1) {
    return ("1"+m);
  }
  else if (i==2) {
    return ("10"+m);
  }
  else {
     EEPROM.put(7,i);
    return ("15"+m);
  }
}

//gets mode setting from EEPROM and prints it out to user
String getMode() {
  byte m;
  EEPROM.get(9,m);
  if(m==1) {
    return "Router";
  }
  else {
    m = 0;
    EEPROM.put(9,m);
    return "End Device";
  }
}

//Checks if user entered a 'Y' into the serial monitor
//if so returns true
bool getAnswer() {
   while (!Serial.available()) { }
   if(Serial.read() == 'Y') return true;
   else return false;
}

//This function gets stored settings from EEPROM and stores in global variables
//The addresses are hard coded and are spread out based on size of stored variable
//each address is a byte in length
void getEEPROMValues() {
  EEPROM.get(1,this_node);
  EEPROM.get(3,iREF);
  EEPROM.get(7,sInterval);
  EEPROM.get(9,mMode);
}


