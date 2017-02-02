#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define RF95_FREQ 915.0

#define VBATPIN A7

PROGMEM static const uint8_t hw_address[] = {0x98,0x76,0xb6,0x5c,0x00,0x02};

void on_rx(void);
RHHardwareSPI zspi = RHHardwareSPI(RHGenericSPI::Frequency8MHz);
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT, zspi, &on_rx); // Adafruit Feather M0 with RFM95 

// Need this on Arduino Zero with SerialUSB port (eg RocketScream Mini Ultra Pro)
//#define Serial SerialUSB

// radio callback from handleInterrupt
void on_rx(void){
  // nothing to do.
}

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
  app_sending_tr,
  app_send_data,
  app_sending_data,
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

struct tr_message{
  uint16_t a;
  uint16_t b;
  uint8_t  c;
} __attribute__((packed));  // total is 5 bytes 

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
uint8_t* make_tr_packet(struct tr_message *p, uint16_t seqno)
{
  int i;
  

  // make up some contents
  p->a = seqno & 0xff;
  p->b = seqno;
  p->c = 9; // number of data packets.
  
  Serial.print("making tr packet ");
  Serial.println(p->c);
  return (uint8_t*)p;
}

void rf_state_machine()
{
  enum app_modes _last = app_mode;  
  static uint8_t data_count = 0;
  
  switch (app_mode){
    case app_wait_to_send:
      if (packetnum == 0 || (millis() - mode_timer > 10000)){        

        struct tr_message tr;
        
        rf95.setModemConfig(RH_RF95::Bw500Cr48Sf4096NoHeadNoCrc);
        rf95.setPayloadLength(5);
        
        if (rf95.send(make_tr_packet(&tr, packetnum++), sizeof(tr))){
          app_mode = app_sending_tr;
        }else{
          Serial.println("ERR: could send TR.");
          mode_timer = millis();
        }
        data_count = tr.c;
      }
      break;

    case app_sending_tr:
      if (rf95.mode() != RHGenericDriver::RHModeTx){
        digitalWrite(13, LOW);
        Serial.print("Send TR complete in ");
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
                        
        app_mode = app_send_data;
        
        //switch to high rate mode
        rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);

        delay(15);
                
      }else if (millis() - mode_timer > 5000){
          Serial.println("WARN: Send TR taking too long, reset radio.");
          digitalWrite(13, LOW);
          app_mode = app_wait_to_send;
        }
      break;
    case app_send_data:
    
        struct rf_message msg;       
             
        digitalWrite(13, HIGH); 
                
        if(rf95.send(make_packet(&msg, packetnum++, "hello world"), sizeof(msg))){          
          app_mode = app_sending_data;
        }else{
          Serial.println("ERR: could not writefifo.");
          app_mode = app_sending_data;
        }
        

      break;
    case app_sending_data:
      if (rf95.mode() != RHGenericDriver::RHModeTx){
          digitalWrite(13, LOW);
          Serial.print("Send data complete in ");
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

          if (--data_count > 0){
            app_mode = app_send_data;           
          }else{
            app_mode = app_wait_to_send;
          }          
  
        } else if (millis() - mode_timer > 5000){
          Serial.println("WARN: Send data taking too long, reset radio.");
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


