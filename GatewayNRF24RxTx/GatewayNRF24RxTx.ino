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

RF24 radio(9,10);                // nRF24L01(+) radio attached using Getting Started board 

RF24Network network(radio);      // Network uses that radio


#define GATEWAY   00      // type allowed GATEWAY, ROUTER, SENSOR

const uint16_t this_node = GATEWAY;    // this variable is used only for router / repearter node only
// Node 0 is the gateway.

/*
 * Function prototypes
 */
void ReceivePayload(void);
void formPacket (uint16_t node_ID);
void SendPayload(uint16_t to_node);
void ProcPayload (void);
void printpayload(void);
 
/*
 * Global variables
 */
 
   
struct payload_t {                    // Structure of our payload
  int16_t ain1, ain2, ain3;           // 6 bytes
  int16_t aout1, aout2;               // 4 bytes
  unsigned char din1, din2, din3, din4;        // 4 bytes
  unsigned char dout1, dout2;                  // 2 bytes
  long counter;                       // 4 bytes
};

volatile payload_t payload;

uint16_t Send2Node;

unsigned long last_sent;             // When did we last send?
const unsigned long interval = 1000; //ms  // How often to send 'hello world to the other unit
int LED_pin7 = 7;
/*
 * main program
 */

void setup(void)
{
  Serial.begin(115200);
  Serial.println("RF24Network Sensor Network Gatway");
  Serial.print("This Node# ");
  Serial.println(this_node);

  pinMode(LED_pin7, OUTPUT);
  
  SPI.begin();
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  network.begin(/*channel*/ 90, /*node address*/ this_node);
  radio.startListening();
}

/*
 * Loop funcion
 */
void loop(void) {
  unsigned long now = millis();              // If it's time to send a message, send it!  
  network.update();                           // Check the network regularly
  /* Receive packet - this code for Gateway / Router */
  
  while( network.available()) {     // Is there anything ready for us?
    ReceivePayload();
    ProcPayload();
    SendPayload(Send2Node);
  }
  /* Transmit packet*/
   if (now - last_sent >= interval)  {
      last_sent = now;
   }
}

/*
 * Function to receive packets
 */
void ReceivePayload(void)
{
    RF24NetworkHeader rheader;        // If so, grab it and print it out
    network.peek(rheader);
    network.read(rheader,&payload,sizeof(payload));
    Send2Node = rheader.from_node;
    
    Serial.print("\n\r Node#:");
    Serial.print(Send2Node);

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
    if (ok) {
      Serial.print("Sent to:");
      Serial.println(to_node);
    }
    else  {
      Serial.print("failed to send "); 
      Serial.print(sizeof(payload));
      Serial.println(" bytes");
    } 
 }

void formPacket (uint16_t node_ID)
{

}

void ProcPayload (void)
{
      payload.dout1 = payload.din1;
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
