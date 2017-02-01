#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define RF95_FREQ 915.0

#define VBATPIN A7

PROGMEM static const uint8_t hw_address[] = {0x98,0x76,0xb6,0x5c,0x00,0x02};

RHHardwareSPI zspi = RHHardwareSPI(RHGenericSPI::Frequency8MHz);
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT, zspi); // Adafruit Feather M0 with RFM95 

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
  rf95.setTxPower(5);

  //actual preamble is x + 4.25 symbols.
  rf95.setPreambleLength(6);
  
  if (rf95.setModemConfig(rf95.Bw500Cr48Sf4096)){
    Serial.println("rf95 configuration set to BW=500 kHz, CR=4/8, SF=12.");
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
  //uint8_t unused[32];
  
  /* 16 bytes */  
} __attribute__((packed));  // total is 32 bytes 

uint8_t* make_packet(struct rf_message *p, uint16_t seqno, const char *text)
{
  int i;
  
  Serial.print("making packet ");
  Serial.println(seqno);
  memcpy(p->hw_address, hw_address, 6);
  memcpy(p->text, text, 16);
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
      if (packetnum == 0 || (millis() - mode_timer > 1000)){        

        struct rf_message msg;           
        
        Serial.println("\nSending ...");
             
        //wait for any pending tx's to complete
        rf95.waitPacketSent();

        digitalWrite(13, HIGH); 

        // fill the fifo
        if(rf95.writefifo(make_packet(&msg, packetnum++, "hello world"), sizeof(msg))){
          rf95.setModeTx();
        }else{
          Serial.println("ERR: could not writefifo.");
        }
        
        app_mode = app_sending;
        
      }
      break;

    case app_sending:
      if (rf95.mode() != RHGenericDriver::RHModeTx){
        digitalWrite(13, LOW);
        Serial.print("Send complete in ");
        Serial.print(millis() - mode_timer);
        Serial.println(" ms.");

        struct RH_RF95::perf_counter* p = rf95.getPerf();

        Serial.print("  -Sent ");
        Serial.print(p->sent_bytes);
        Serial.println(" bytes.");

        Serial.print("  -Send overhead: ");
        Serial.print(p->tx_mode - p->send_call);
        Serial.println(" ms.");

        Serial.print("  -Tx time: ");
        Serial.print(p->interrupt - p->tx_mode);
        Serial.println(" ms.");
        
        Serial.print("  -Interrupts: ");
        Serial.println(p->interrupt_count);
                        
        app_mode = app_wait_to_send;
        Serial.println("Waiting to send again.");

      } else if (millis() - mode_timer > 5000){
        Serial.println("WARN: Send taking too long, reset radio.");
        digitalWrite(13, LOW);
        app_mode = app_wait_to_send;
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


