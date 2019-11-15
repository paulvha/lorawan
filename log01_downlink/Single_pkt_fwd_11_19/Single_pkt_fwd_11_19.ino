#include <FileIO.h>
#include <Console.h>
#include <Process.h>
#include <SPI.h>
#include <LoRa.h>

const String Sketch_Ver = "single_pkt_fwd_11_19";

// timeout settings
const long sendpkt_interval = 15000;      // 15 seconds for replay on OTA
const long interval = 30000;//60000;              // 1min for feeddog.

// Lora settings
static float freq, txfreq;              
static int SF, CR, txsf;
static long BW, preLen;

// prototypes
void getRadioConf();                      // Get LoRa Radio Configure from LG01
void setLoRaRadio();                      // Set LoRa Radio
void receivepacket();                     // receive packet
void sendpacket();                        // send join accept payload (NOT TESTED)
void emitpacket();                        // send data down
void writeVersion();                      // write version to linux
void feeddog();                           // keep alive /reset
void mcu_boot();                          // log the succesfull boot

// common global variables
static uint8_t packet[256];               // used during downlink: send_packet (OTA) and emitpacket (data)
static uint8_t message[256];              // used during receive packet
static uint8_t packet1[32];               // tmp storage for create file
static int send_mode = 0;                 // define mode.  default receive mode 
static int DownLinkSize = 0;              // if not zero downlink info is waiting
unsigned long previousMillis = millis();  // last feeddog update
unsigned long RXMillis;                   // RX end time
static char dwdata[32] = {'\0'};          // for data downlink  payload

// create file
const char  dfile[] = "/var/iot/dog";
const char  kfile[] = "/var/iot/KILL";

#define DOG 1
#define KILL 2

/////////////////////////////////////////////////////////////////
// Set Debug 
// 1 to enable Console Output;
// 0 = no messages
////////////////////////////////////////////////////////////////
int SDEBUG = 1;

/////////////////////////////////////////////////////////////////
// enable debug pulse for scope measurement
// GPIO pulse to show pulse for Window-1 and window-2 (gnd is 6- )
// remove comments to enable PULSE
/////////////////////////////////////////////////////////////////
//#define PULSE 1

#if defined(PULSE) 
#define WINDOW1 PD3   // 7A
#define WINDOW2 PD4   // 7B
#endif

void setup(){
    // Setup Bridge
    Bridge.begin(115200);
    
    // Setup File IO
    FileSystem.begin();

    Console.begin();
    if ( SDEBUG > 0 )
    {
      //Print Current Version
      Console.print(F("Sketch Version:"));
      Console.println(Sketch_Ver);
    }
    
#if defined(PULSE)   
    // debugging only
    pinMode(WINDOW1, OUTPUT);
    pinMode(WINDOW2, OUTPUT);
#endif

    //write sketch version to Linux
    writeVersion();

    //Get Radio configure
    getRadioConf();

    if ( SDEBUG > 0 )
    {
        Console.println(F("Start LoRaWAN Single Channel Gateway"));
        Console.print(F("RX Frequency: "));
        Console.println(freq);
        Console.print(F("TX Frequency: "));
        Console.println(txfreq);
        Console.print(F("Spread Factor: SF"));
        Console.println(SF);
        Console.print(F("TX Spread Factor: SF"));
        Console.println(txsf);
        Console.print(F("Coding Rate: 4/"));
        Console.println(CR);
        Console.print(F("Bandwidth: "));
        Console.println(BW);
        Console.print(F("PreambleLength: "));
        Console.println(preLen);
    }

    if (!LoRa.begin(freq)){
         Console.println(F("init LoRa failed"));
         for (;;);
    }
    
    setLoRaRadio();// Set LoRa Radio to Semtech Chip
    delay(1000);

    // tell log we are up and running
    mcu_boot();
}

void loop(){

    if (!send_mode) {             // received message and check for  server downstream
        receivepacket();          
    } else if (send_mode == 1) {  // join request wait.
        sendpacket();
    } else {                      // downlink data file was detected for device who did the most recent uplink
        emitpacket();
    }

    unsigned long currentMillis = millis();
    if ((currentMillis - previousMillis ) >= interval){
      previousMillis = currentMillis;
      feeddog();
    }
}

//Receive LoRa packets from device and forward it to TTN
void receivepacket() {

#if defined(PULSE) 
    digitalWrite(WINDOW2, LOW);     
#endif

    LoRa.receive(0);  // set for Implicite header & continuous reading

    int packetSize, i = 0;
    memset(message, 0, sizeof(message)); /* make sure message is empty */
    
    // check for packet
    packetSize = LoRa.parsePacket();

    if (packetSize) {   // Received a packet
  
      RXMillis = millis();     // save the RXEND time

#if defined(PULSE) 
      digitalWrite(WINDOW2, HIGH);     
#endif            
      
      while (LoRa.available() && i < 256) {// get packet
          message[i] = LoRa.read();
          i++;
      }
      
      // in case previous downlink message is waiting
      // handle that NOW.. as window timing is critical
      if (DownLinkSize > 0) DoDownlink();

      if ( SDEBUG > 0 ) {
          Console.println();
          Console.print(F("Get Packet:"));
          Console.print(packetSize);
          Console.println(F(" Bytes"));

          for(i=0; i < packetSize; i++) {
            Console.print(F("["));
            Console.print(i);
            Console.print(F("]"));
            Console.print(message[i], HEX);
            Console.print(F(" "));
          }
          Console.println("");
      }

      /*cfgdata file will be save rssi and packetsize*/
      File cfgFile = FileSystem.open("/var/iot/cfgdata", FILE_WRITE);
      cfgFile.print("rssi=");
      cfgFile.println(LoRa.packetRssi());
      cfgFile.print("size=");
      cfgFile.println(packetSize);
      cfgFile.close();

      File dataFile = FileSystem.open("/var/iot/data", FILE_WRITE);
      dataFile.write(message, i);
      dataFile.close();

      // OTTA request??
      if ((int)message[0] == 0) {      // Join Request 
        send_mode = 1;  // change the mode and call send_packet
        return;
      }

      /* process to check Data downlink */
      char devaddr[12] = {'\0'};
      sprintf(devaddr, "%x%x%x%x", message[1], message[2], message[3], message[4]);
      
      if (strlen(devaddr) > 8) {
        for (i = 0; i < strlen(devaddr) - 2; i++) {
          devaddr[i] = devaddr[i + 2];
        }
      }
      
      devaddr[i] = '\0';
      snprintf(dwdata, sizeof(dwdata), "/var/iot/%s", devaddr);

      if (SDEBUG > 0) {
        Console.print(F("Devaddr:"));
        Console.println(dwdata);
      }

      /* check for download message received 
       * for this you need the device address, in case of multiple device to this gateway
       * The download message is only handled after the NEXT uplink message
       */

      if (FileSystem.exists(dwdata)) {
        send_mode = 2;        // start emitdata to read the downlink data
        
        if (SDEBUG > 0) {
          Console.print(dwdata);
          Console.println(F(" Exists"));
        }
      }

      Console.print(F("END A PACKET, Mode:"));
      Console.println(send_mode, DEC);
      return; /* exit the receive loop after received data from the node */
    } /* end of if packetsize than 1 */
}


/** downlink datafile was detected
 *  
 *  we now read the content of the downlink message from the device file and 
 *  make it ready to sent to the device after the next uplink.
 *  
 *  This has been done as downlink windows are time critical
 */
void emitpacket()
{
  int i = 0, j = 0;

  // get the packet
  Process rd;
  rd.begin("cat");
  rd.addParameter(dwdata);
  rd.run();
  
  while (true){
   packet[i] = rd.read();
   if (packet[i] == 0xff) break;
   if (++i > 255) break;
  }

  // to small (only header and part of device address)
  if (i < 3)   return;

  /*  to be used in combination with checkdogDOWN script
   *  In the script the content of the KILL-file is check every minute
   *  if the content is older than 300 seconds, the current lg01_pkt_fwd
   *  will be killed (if no uplink is pending) and restart
   *  The downlink thread is not steady implemented in the lg01_pkt_fwd */
  create_file(KILL, 400);

  if ( SDEBUG > 0 ) {
    Console.println(F("Downlink Message queued:"));
    for (j = 0; j < i; j++) {
      Console.print(F("["));
      Console.print(j);
      Console.print(F("]"));
      Console.print(packet[j], HEX);
      Console.print(F(" "));
    }
    Console.println();
  }

  // remove the download link file
  Process rm;
  rm.begin("rm");
  rm.addParameter("-rf");
  rm.addParameter(dwdata);
  rm.run();

  send_mode = 0;     // back to receive mode
  DownLinkSize = i;  // respond to next uplink from device
}

/* sent queued package to the Lora device */
DoDownlink()
{
  // downlink window delay
  unsigned long w1delay = 1000;
  unsigned long w2delay = w1delay + 1000;

  LoRa.beginPacket();
  LoRa.write(packet, DownLinkSize);
 
  // wait for first window (1 second)
  while (millis()- RXMillis  < w1delay );
  
#if defined(PULSE) 
  digitalWrite(WINDOW1, HIGH);
  LoRa.endPacket();
  digitalWrite(WINDOW1, LOW); 
#else
  LoRa.endPacket();
#endif

  // if the first window was received the second will probably be neglected
  LoRa.setFrequency(txfreq);
  LoRa.setSpreadingFactor(txsf);    /* begin send data to the lora node, lmic use the second receive window, and SF default to 9 */
  
  LoRa.beginPacket();
  LoRa.write(packet, DownLinkSize);
  
  // wait for second window 
  while (millis() - RXMillis < w2delay );

#if defined(PULSE) 
  digitalWrite(WINDOW2, HIGH);
  LoRa.endPacket();
  digitalWrite(WINDOW2, LOW); 
#else
  LoRa.endPacket();
#endif
  
  LoRa.setFrequency(freq);
  LoRa.setSpreadingFactor(SF);    /* reset SF to receive message */
  
  DownLinkSize = 0;

  if (SDEBUG > 0) Console.println(F("[transmit] Data Down END"));
}

/* create/update a file with a time stamp 
 * index : to set the filename to update/create
 * to_sub : any time to subtract */
void create_file(uint8_t index, unsigned long to_sub)
{
  unsigned long newtime = get_cursec() - to_sub;
  char *ff;

  if (index == DOG)  ff = dfile;
  else if (index == KILL) ff = kfile;

  File sFile = FileSystem.open(ff, FILE_WRITE);
  sFile.println(newtime);
  sFile.close();
}

/* return the seconds 1970-01-01 00:00:00 UTC as a unsigned long */
unsigned long get_cursec()
{
    int k = 0;
    
    memset(packet1, 0, sizeof(packet1));

    Process p;    // Create a process
    p.begin("date");
    p.addParameter("+%s");      // seconds since 1970-01-01 00:00:00 UTC
    p.run();
    while (p.available() > 0 && k < 32) {
        packet1[k] = p.read();
        k++;
    }

    return(atol(packet1));
}

/* 
 * This is called every 60000ms (or every minute) and creates/updates a file with the current seconds 
 * 
 * The Checkdog script (run from cron in DRAGINO) will check for the file , reads the content and compare
 * against the current date +%s. 
 * 
 * If the difference is larger than 300 seconds  (= 5 min) it will assume the sketch had died 
 * and will call reset-mcu to restart the sketch. 
 */

void feeddog(){
  return(create_file(DOG,0));
}


// Function to write sketch version number into Linux.
void writeVersion(){
  File fw_version = FileSystem.open("/var/avr/fw_version", FILE_WRITE);
  fw_version.print(Sketch_Ver);
  fw_version.close();
}

// do logging that the MCU has booted
void mcu_boot(){
    Process r;
    r.begin("logger");
    r.addParameter("\"mcu_boot\"");
    r.run();
}

// Get LoRa Radio Configure from LG01
void getRadioConf() {
    char tmp[32];

    Process p;    // Create a process

    //Read frequency from uci ####################
    int j = 0;
    memset(tmp, 0, sizeof(tmp));
    p.begin("uci");
    p.addParameter("get");
    p.addParameter("lorawan.radio.rx_frequency");
    p.run();    // Run the process and wait for its termination
    while (p.available() > 0 && j < 9) {
        tmp[j] = p.read();
        j++;
    }
    freq = atof(tmp);

    //Read txfre from uci ####################
    j = 0;
    memset(tmp, 0, sizeof(tmp));
    p.begin("uci");
    p.addParameter("get");
    p.addParameter("lorawan.radio.tx_frequency");
    p.run();    // Run the process and wait for its termination
    while (p.available() > 0 && j < 10) {
        tmp[j] = p.read();
        j++;
    }
    txfreq = atof(tmp);

    //Read Spread Factor ####################
    j = 0;
    memset(tmp, 0, sizeof(tmp));
    p.begin("uci");
    p.addParameter("get");
    p.addParameter("lorawan.radio.SF");
    p.run();    // Run the process and wait for its termination
    while (p.available() > 0 && j < 3) {
        tmp[j] = p.read();
        j++;
    }

    SF = atoi(tmp) > 0 ? atoi(tmp) : 10;  //default SF10

    //Read tx Spread Factor ####################
    j = 0;
    memset(tmp, 0, sizeof(tmp));
    p.begin("uci");
    p.addParameter("get");
    p.addParameter("lorawan.radio.TXSF");
    p.run();    // Run the process and wait for its termination
    while (p.available() > 0 && j < 3) {
        tmp[j] = p.read();
        j++;
    }

    txsf = atoi(tmp) > 0 ? atoi(tmp) : 9;  //Txsf default to sf9

    //Read Coding Rate  ####################
    j = 0;
    memset(tmp, 0, sizeof(tmp));
    p.begin("uci");
    p.addParameter("get");
    p.addParameter("lorawan.radio.coderate");
    p.run();    // Run the process and wait for its termination
    while (p.available() > 0 && j < 2) {
        tmp[j] = p.read();
        j++;
    }
    CR = atoi(tmp);

    //Read PreambleLength
    j = 0;
    memset(tmp, 0, sizeof(tmp));
    p.begin("uci");
    p.addParameter("get");
    p.addParameter("lorawan.radio.preamble");
    p.run();    // Run the process and wait for its termination
    while (p.available() > 0 && j < 5) {
        tmp[j] = p.read();
        j++;
    }
    preLen = atol(tmp);

    //Read BandWidth  #######################

    j = 0;
    memset(tmp, 0, sizeof(tmp));
    p.begin("uci");
    p.addParameter("get");
    p.addParameter("lorawan.radio.BW");
    p.run();    // Run the process and wait for its termination
    while (p.available() > 0 && j < 2) {
        tmp[j] = p.read();
        j++;
    }

    switch (atoi(tmp)) {
        case 0: BW = 7.8E3; break;
        case 1: BW = 10.4E3; break;
        case 2: BW = 15.6E3; break;
        case 3: BW = 20.8E3; break;
        case 4: BW = 31.25E3; break;
        case 5: BW = 41.7E3; break;
        case 6: BW = 62.5E3; break;
        case 7: BW = 125E3; break;
        case 8: BW = 250E3; break;
        case 9: BW = 500E3; break;
        default: BW = 125E3; break;
    }
}

/* initalise LORA based on the retrieved values */
void setLoRaRadio() {
    // enable Lora
    //LoRa.EnableDebug(Console);    // this requires an adjustment to lora.cpp or it will fail
    LoRa.setFrequency(freq);
    LoRa.setSpreadingFactor(SF);
    LoRa.setSignalBandwidth(BW);
    LoRa.setCodingRate4(CR);
    LoRa.setSyncWord(0x34);
    LoRa.setPreambleLength(preLen);
}

/* in case of of OTA */

/* THIS HAS NOT BEEN TESTED OR EVEN TRIED TO TEST. I USE ABP
 * That said, when I tried to compile with the code included, it failed to run.
 * seems like a memory shortage issue.
 */
void sendpacket()
{
    send_mode = 0;    // set back to receive packet
}
