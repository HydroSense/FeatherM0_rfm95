// rf95_client.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the DQNBasestation/echoServer
// Tested with Adafruit Feather M0 with RFM95

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define RF95_FREQ 915.0

#define VBATPIN A7

PROGMEM static const uint8_t hw_address[] = {0x98,0x76,0xb6,0x5c,0x00,0x01};

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT); // Adafruit Feather M0 with RFM95 

// Need this on Arduino Zero with SerialUSB port (eg RocketScream Mini Ultra Pro)
//#define Serial SerialUSB

void setup() 
{
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  pinMode(VBATPIN, INPUT);
  
  // un-reset the radio
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(57600);
  while (!Serial) ; // Wait for serial port to be available (does not boot headless!)
  if (!rf95.init()){
    Serial.println("init failed");
    
    while (1);
  }
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed"); 
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);


  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
//  driver.setTxPower(23, false);
  // If you are using Modtronix inAir4 or inAir9,or any other module which uses the
  // transmitter RFO pins and not the PA_BOOST pins
  // then you can configure the power transmitter power for -1 to 14 dBm and with useRFO true. 
  // Failure to do that will result in extremely low transmit powers.
//  driver.setTxPower(14, true);

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  if (rf95.setModemConfig(rf95.Bw500Cr45Sf128)){
    Serial.println("rf95 configuration set to BW=500 kHz BW, CR=4/5 CR, SF=7.");
  }else{
    Serial.println("rf95 configuration failed.");
    while (1);
  }

}

int packetnum = 0;
int mode_timer = 0;

enum app_modes{
  app_wait_to_send=0,
  app_sending,
  app_wait_for_reply,  
};

enum app_modes app_mode = app_wait_to_send;

struct rf_message{
  
  /* RF parameters */  
  int8_t rssiUp;  
  int8_t rssiDown;
  uint8_t hw_address[6];
  
  float freq;
  
  uint8_t seqno;
  uint8_t bw;    
  uint8_t sf;
  uint8_t cr;
  
  /* 16 bytes */
  
  uint8_t text[16];
  /* 16 bytes */  
} __attribute__((packed));  // total is 32 bytes

uint8_t* make_packet(struct rf_message *p, uint16_t seqno, const char *text)
{
  int i;
  
  Serial.print("making packet ");
  Serial.println(seqno);
  memcpy(p->hw_address, hw_address, 6);
  p->seqno = seqno;
  p->freq = rf95.getFrequency();
  p->bw = rf95.getBw();
  p->sf = rf95.getSf();
  p->cr = rf95.getCr();
  p->rssiDown = rf95.lastRssi();
  p->rssiUp = 0;

  
  return (uint8_t*)p;
}

void print_packet(struct rf_message *m)
{
  int i;
  for (i=0;i<6;i++){
    Serial.print(m->hw_address[i], HEX);
    Serial.print(":");
  }
  Serial.print(" [");
  Serial.print(m->freq);
  Serial.print("MHz/Bw");
  Serial.print(m->bw);
  Serial.print("kHz/Cr4");
  Serial.print(m->cr+4);
  Serial.print("Sf");
  Serial.print(pow(2,m->sf));
  Serial.print("]: ");
  Serial.print((char*)m->text);
  
}

void rf_state_machine()
{
  enum app_modes _last = app_mode;  
  switch (app_mode){
    case app_wait_to_send:
      if (millis() - mode_timer > 1000){        

        struct rf_message msg;           
        
        Serial.println("\nSending ...");
                
        if (rf95.send(make_packet(&msg, packetnum++, "hello world"), sizeof(msg))){
          digitalWrite(13, HIGH);
          app_mode = app_sending;
        }else{
          Serial.print("ERR: Send failed!");
        }
      }
      break;

    case app_sending:
      if (rf95.mode() != RHGenericDriver::RHModeTx){
        Serial.print("Send complete in ");
        Serial.print(millis() - mode_timer);
        Serial.println(" ms.");
        app_mode = app_wait_for_reply;
        Serial.println("Waiting for reply.");
      } else if (millis() - mode_timer > 5000){
        Serial.println("WARN: Send taking too long, reset radio.");
        app_mode = app_wait_to_send;
      }
      break;

    case app_wait_for_reply:
    {
      // Now wait for a reply
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);

      if (rf95.available())
      {      
         if (rf95.recv(buf, &len))
         {
            Serial.print("got reply [");
            Serial.print(len);
            Serial.print("]: ");

            print_packet((struct rf_message *)buf);
            
            Serial.print("RSSI: ");
            Serial.println(rf95.lastRssi(), DEC);
          }
          else
          {
            Serial.println("ERR: recv failed");
          }
          digitalWrite(13, LOW);
          app_mode = app_wait_to_send;
          
          rf95.setMode(RHGenericDriver::RHModeSleep);
      } else if (millis() - mode_timer > 1000){
        Serial.println("WARN: No response message received, giving up.");
        app_mode = app_wait_to_send;         
        digitalWrite(13, LOW);
        rf95.setMode(RHGenericDriver::RHModeSleep);
      }
    }
    break;
    default:
      Serial.println("Unknown app mode!");
      break;
  }
  if (app_mode != _last){ 
    mode_timer = millis();
  }
}

void loop()
{
  
  rf_state_machine();
    
}


