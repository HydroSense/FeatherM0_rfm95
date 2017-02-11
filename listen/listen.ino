
#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RFM95_FHSS_INT 5

#define RF95_FREQ 915.0

#define VBATPIN A7

PROGMEM static const uint8_t hw_address[] = {0x98,0x76,0xb6,0x5c,0x00,0x01};

void on_rx(void); //forward declaration of rx handler
RHHardwareSPI zspi = RHHardwareSPI(RHGenericSPI::Frequency8MHz);
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT, RFM95_FHSS_INT, zspi, &on_rx); // Adafruit Feather M0 with RFM95 

// Need this on Arduino Zero with SerialUSB port (eg RocketScream Mini Ultra Pro)
//#define Serial SerialUSB


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

struct tr_message{
  uint16_t a;
  uint16_t b;
  uint8_t  c; // c is how many data packets to expect
} __attribute__((packed));  // total is 5 bytes 


volatile static uint8_t have_tr = 0;
static uint8_t data_count= 0;
static uint32_t data_time = 0;

// radio rx callback
void on_rx(void){  
  //not needed
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

  //actual preamble is x + 4.25 symbols.
  rf95.setPreambleLength(6);

  //setup to receive a TR message
  rf95.setModemConfig(RH_RF95::Bw500Cr48Sf4096NoHeadNoCrc);
  rf95.setPayloadLength(5);  

  // confgiure FHSS for < 400 ms dwell time.
  rf95.configureFhss(400);

  Serial.println("Listening for packets...");
}


void print_packet(struct rf_message *m)
{
  
  static const char* bw_lut[] ={
    "7.8",
    "10.4",
    "15.6",
    "20.8",
    
    "31.25",
    "41.7",
    "62.5",
    "125",
    
    "250",
    "500",
    "a",
    "b",
    
    "c",
    "d",
    "e",
    "f",
  };
  int i;
  
  for (i=0;i<6;i++){
    Serial.print(m->hw_address[i], HEX);
    Serial.print(":");
  }
  Serial.print(" [");
  Serial.print(m->freq);
  Serial.print("MHz/Bw");
  Serial.print(bw_lut[m->bw&0xf]);
  Serial.print("kHz/Cr4");
  Serial.print(m->cr+4);
  Serial.print("Sf");
  Serial.print((int)pow(2,m->sf));
  Serial.print("] ");

  Serial.print("(sq=");
  Serial.print(m->seqno);
  Serial.print("): \"");

  //ensure text buffer is null terminated.
  char tbuf[17];
  memcpy(tbuf, m->text, 16);
  tbuf[16] = 0;  
  Serial.print(tbuf);
  Serial.print("\" ");
}

void loop()
{  
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    
//    struct RH_RF95::perf_counter* p = rf95.getPerf();
//    static uint32_t icount = 0;
//
//      Serial.print("CAD count ");
//      Serial.print(p->cad_cnt);      
//      Serial.print(", Rx timeout ");
//      Serial.print(p->rx_timeout);   
//      Serial.print(", Intterupts ");
//      Serial.println(p->interrupt_count);   

    if (have_tr == 0){
      // wait for a TR message
      if (rf95.available())
      {     
         // assume we got a tr, setup for data
         //switch to high rate mode, the data message comes immediately
         rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);    
         rf95.configureFhss(400);
         rf95.setModeRx();
         
         digitalWrite(13, HIGH);
         if (rf95.recv(buf, &len)){
            struct tr_message *tr;
            tr = (struct tr_message*)&buf[0];
            Serial.print("RX [TR]: ");
            Serial.print(tr->a);
            Serial.print(", ");
            Serial.print(tr->b);
            Serial.print(", ");
            Serial.print(tr->c);


            Serial.print("  RSSI: ");
            Serial.println(rf95.lastRssi(), DEC);
            have_tr = 1;
            data_count = tr->c;   
            data_time =millis();
         }else{
            Serial.println("ERR: recv failed");
         }
         digitalWrite(13, LOW); 
      }
          
    }else{
      // wait for Data messages
      if (millis() - data_time > 3000){
        Serial.println("Timeout waiting for data.");
        have_tr= 0;
        rf95.setModemConfig(RH_RF95::Bw500Cr48Sf4096NoHeadNoCrc);
        rf95.configureFhss(400);
        rf95.setPayloadLength(5);       
        rf95.setModeRx();
      }
      if (rf95.available())
      { 
         if(--data_count==0){ 
            //switch back to waiting for tr's before we get the packet.
            have_tr = 0;    
            rf95.setModemConfig(RH_RF95::Bw500Cr48Sf4096NoHeadNoCrc);
            rf95.configureFhss(400);            
            rf95.setPayloadLength(5);       
            rf95.setModeRx();
         }
                     
         digitalWrite(13, HIGH);
         if (rf95.recv(buf, &len))
         {
            Serial.print("RX [");
            Serial.print(len);
            Serial.print("]: ");
  
            print_packet((struct rf_message *)buf);
            
            Serial.print("RSSI: ");
            Serial.println(rf95.lastRssi(), DEC);
  
  //          Serial.print("RF Time: ");
  //          Serial.print(p->rx_done - p->cad_done);
  //          Serial.println(" ms.");        
          }
          else
          {
            Serial.println("ERR: recv failed");
          }

          digitalWrite(13, LOW);       
      }
    }
    
}


