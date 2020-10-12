/*
   RV-C CAN_Bus_Monitor
     by: Michael J. Kidd <linuxkidd@gmail.com>
    Rev: 1.0
   Date: 2015-11-23
*/

#include <SPI.h>

#include "mcp_can.h"

boolean enable_monitor_output  = false;  // Should we output full can-bus feed?
boolean debug                  = true;  // Enable debug output

//const int CAN_SPEED            = CAN_250KBPS;    // For Uno  CAN-Bus Shield    (16MHz)
const int CAN_SPEED            = CAN_8M_250KBPS; // For nano CAN SPI interface  (8MHz)
const int SPI_CS_PIN           =   10;           // Which pin to use for SPI CS with CAN bus interface

boolean enable_pump_control    = true;  // Enable RO pump control ( 1 = enabled )
const int PUMP_PIN             =    9;  // RO pump relay control output pin
unsigned int req_full_count    =   60;  // Required count of full level before action is taken

const int HOTWATER_FLOW_PIN    =    8;  // Pin to detect hot water demand (connect to ground = demand)


/*
 * Variables below this point are used internally and should not be changed.
 */
unsigned int status_every     =    30;  // Request status of DC loads every 30 seconds
unsigned int last_status      =     0;  // millis() value of last status request
unsigned int now_time         =     0;

unsigned int pump_state       =   LOW;  // Default pump state - LOW = off

unsigned char len = 0;
unsigned char buf[8];

INT32U canId = 0x000;
char outbuf[8];
char outbuf2[32];
char hexDGN[6];

unsigned int bin2int(char * digits);
char* int2bin(INT32U d);
bool checkAquaHotStatus();
void checkAquaHotDemand();
void toggleAquaHot();
void parseTank();


MCP_CAN CAN(SPI_CS_PIN);         // Set CS pin

typedef struct {
  boolean state           = false;
  unsigned int full_count = 0;
} tank;

tank grey;
tank fresh;

typedef struct {
  boolean enable    = true;  // Enable control of AquaHot based on flow switch
  boolean prestate  = false;  // What is the normal state of the AquaHot system (without us requesting it be on)
  boolean desired   = false;  // Desired Status
  boolean state     = false;  // Last read state
  boolean firstread = true;   // Is this the first read of the aquahot state
} aquahot;

typedef struct {
  char prio[4];
  char dgnhi[10];
  char dgnlo[9];
  char srcAD[9];
} packetmeta;

aquahot ah;

void setup() {
  Serial.begin(38400);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(HOTWATER_FLOW_PIN, INPUT_PULLUP);
START_INIT:
  if (CAN_OK == CAN.begin(CAN_SPEED))  {
    CAN.init_Mask(0, 1, 0);
    CAN.init_Mask(1, 1, 0);
    for (int i = 0; i < 6; i++) {
      CAN.init_Filt(i, 1, 0);
    }

    if ( debug )
      Serial.println("Debugging Enabled...");
  }
  else
  {
    delay(100);
    goto START_INIT;
  }
}

void loop() {

  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&len, buf);
    canId = CAN.getCanId();
    char *binCanID=int2bin(canId);

    packetmeta p;

    // Separate the overloaded canID
    memcpy(p.prio,  &binCanID[0], 3);
    memcpy(p.dgnhi, &binCanID[4], 9);
    memcpy(p.dgnlo, &binCanID[13],8);
    memcpy(p.srcAD, &binCanID[21],8);

    // Terminate all the char strings
    p.prio[3]='\0';
    p.dgnhi[9]='\0';
    p.dgnlo[8]='\0';
    p.srcAD[8]='\0';

    free(binCanID);

    sprintf(hexDGN, "%03X%02X",                // Construct full hexDGN for other checking (RO pump control)
            bin2int(p.dgnhi),
            bin2int(p.dgnlo));

    if ( enable_monitor_output ) {
      sprintf(outbuf2, "%X,%03X%02X,%02X,%d,",
            bin2int(p.prio),
            bin2int(p.dgnhi),
            bin2int(p.dgnlo),
            bin2int(p.srcAD),
            len);

      Serial.print(millis()); Serial.print(",");
      Serial.print(outbuf2);
      for (int i = 0; i < len; i++) {
        sprintf(outbuf, "%02X", buf[i]);
        Serial.print(outbuf);
      }
      Serial.println();
    }
    if (strncmp(hexDGN,"1FFB7", 5) == 0 && enable_pump_control )
      parseTank();

    if(ah.enable && !ah.firstread && !ah.desired)
      checkAquaHotDemand();

    if (ah.enable && strncmp(hexDGN, "1FEDA", 5) == 0 && buf[0] == 24) { // Only enter here if we have AquaHot Diesel status, and need it
      ah.state = checkAquaHotStatus();
      if ( !ah.desired || ah.firstread ) {
        ah.prestate = ah.state;
        ah.firstread = false;
      }
      if ( debug ) {
        sprintf(outbuf2, "pre: %d, ah_desired: %d, ah_state: %d",ah.prestate, ah.desired, ah.state);
        Serial.print(millis()); Serial.print(","); Serial.println(outbuf2);
      }
    }
    if(ah.enable && ah.desired)
      checkAquaHotDemand();

    if(ah.enable && ah.firstread) {
      if (millis()>4000) {
        ah.firstread = false;
        if ( debug )
          Serial.println("4 seconds elapsed without AquaHot status... Clearing 'firstread'");
      }
    }
  }
}


/*
   unsigned int bin2int(char * digits)
     This function converts a string of binary bits to an int.
*/
unsigned int bin2int(char * digits) {
  unsigned int res = 0;
  int digits_len = strlen(digits);

  for (int i = 0; i < digits_len; i++) {
    if ((digits[digits_len - (i + 1)]) == '1')
      res += (1 << i);
  }
  return res;
}


/*
 * char* int2bin(INT32U d)
 *    This funciton converts an Unsigned 32 bit int into a CHAR string of binary
 */
char* int2bin(INT32U d) {
  const char* mybins = "01";
  int pos = 0;
  // unsigned long int b;
  char b[29];
  char *c = (char*)malloc(29);
  while(d > 0) {
    b[pos]=mybins[d % 2];
    d /= 2;
    pos++;
  }
  while(pos<29) {
    b[pos]=mybins[0];
    pos++;
  }
  b[pos]='\0';
  pos--;
  for(int i=pos;i>=0;i--) {
    c[pos-i]=b[i];
  }
  pos++;
  c[pos]='\0';
  return c;
}

/*
 * parseAquaHotStatus()
 *   This function logs the current status of the AquaHot system and checks flow sensor input state
 */

bool checkAquaHotStatus() {
  bool curstate=false;
  if (buf[2] == 200)
    curstate=true;
  return curstate;
}

void checkAquaHotDemand() {
  if (digitalRead(HOTWATER_FLOW_PIN) == LOW && !ah.desired) {
    ah.desired = true;
    toggleAquaHot();

    if ( debug ) {
      sprintf(outbuf2, "AH: TurnOn: pre: %d, ah_desired: %d, ah_state: %d",ah.prestate, ah.desired, ah.state);
      Serial.print(millis()); Serial.print(","); Serial.println(outbuf2);
    }
  }
  if (digitalRead(HOTWATER_FLOW_PIN) == HIGH && ah.desired ) {
    ah.desired = false;
    toggleAquaHot();
    
    if ( debug ) {
      sprintf(outbuf2, "AH: TurnOff: pre: %d, ah_desired: %d, ah_state: %d",ah.prestate, ah.desired, ah.state);
      Serial.print(millis()); Serial.print(","); Serial.println(outbuf2);
    }
  }
}

void toggleAquaHot() {
  unsigned char stmp[8] = {0x18, 0xFF, 0xC8, 0x02, 0xFF, 0x00, 0xFF, 0xFF};
  if( !ah.desired && !ah.prestate)
    stmp[3]=0x03;
  CAN.sendMsgBuf(0x19FEDB97, 1, 8, stmp);
  if( debug ) {
    Serial.print("Toggle AquaHot: ");
    Serial.println(ah.desired);
  }
}

/*
   parseTank()
    This function handles Reverse Osmosis Pump control.
    The RO Pump is turned ON if:
     - Fresh water tank is < full
     - Grey water tank is < 1 step below full
    Both parameters are used in their native form ( level / resolution ) which would normally be divided for percentage.
*/
void parseTank() {
  if (buf[0] == 0) { // Fresh water tank
    if (buf[1] < buf[2]) { // It's not full
      fresh.state = true;
      fresh.full_count=0;
    } else {
      if(fresh.full_count > req_full_count )
        fresh.state = false;
      else
        fresh.full_count++;
    }
  }
  if (buf[0] == 2) { // Grey water tank
    if (buf[1] < (buf[2] - 1)) { // Grey tank < 1 level below full
      grey.state = true;
      grey.full_count=0;
    } else {
      if(grey.full_count > req_full_count)
        grey.state = false;
      else
        grey.full_count++;
    }
  }
  if (fresh.state == 1 && grey.state == 1) {
    if ( pump_state != HIGH && debug )
      Serial.println("Toggle Pump: on");
    pump_state = HIGH;
  } else {
    if ( pump_state != LOW && debug )
      Serial.println("Toggle Pump: off");
    pump_state = LOW;
  }

  digitalWrite(PUMP_PIN, pump_state);
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
