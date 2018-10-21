/*

Da Vinci EEPROM update Copyright (C) 2014 by Oliver Fueckert <oliver@voltivo.com>
Increment Serial code - contributed by Matt
UNI/O Library Copyright (C) 2011 by Stephen Early <steve@greenend.org.uk>

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.  */

/************************************************************

Pinout looking at the pads on the EEPROM board

-------------------\
|                   \
|  GND   SCIO   +5V  \
|                    | 
----------------------

// ABS White (used)
00: 5A415A0000363548C0D40100C0D40100 ZAZ..65H........
10: D2005A00544847423030353500000000 ..Z.THGB0055....
20: 0000000034000000FFFFFFFFAA55AA55 ....4........U.U
30: 883355AA0D2E00000000001004ED00E0 .3U.............
40: 5A415A0000363548C0D40100C0D40100 ZAZ..65H........
50: D2005A00544847423030353500000000 ..Z.THGB0055....
60: 0000000034000000FFFFFFFFAA55AA55 ....4........U.U
70: 883355AAC0D40100AA55AA559F200B00 .3U......U.U. ..

// ABS Red (used)
00: 5A41520000363548C0D40100C0D40100 ZAR..65H........
10: D2005A00544847423032313300000000 ..Z.THGB0213....
20: 0000000034000000FFFFFFFFAA55AA55 ....4........U.U
30: 883355AAA49301000000001004ED00E0 .3U.............
40: 5A41520000363548C0D40100C0D40100 ZAR..65H........
50: D2005A00544847423032313300000000 ..Z.THGB0213....
60: 0000000034000000FFFFFFFFAA55AA55 ....4........U.U
70: 883355AAC0D40100AA55AA559F200B00 .3U......U.U. ..

// ABS Yellow (new)
00: 5A4159000034384E80A9030080A90300 ZAY..48N........
10: D2005A00544847423031343100000000 ..Z.THGB0141....
20: 0000000034000000FFFFFFFFAA55AA55 ....4........U.U
30: 883355AA80A90300AA55AA55C7B00A00 .3U......U.U....
40: 5A4159000034384E80A9030080A90300 ZAY..48N........
50: D2005A00544847423031343100000000 ..Z.THGB0141....
60: 0000000034000000FFFFFFFFAA55AA55 ....4........U.U
70: 883355AA80A90300AA55AA55C7B00A00 .3U......U.U....

// PLA Red (used)
00: 5A5032000036354C400D0300400D0300 ZP2..65L@...@...
10: BE002D00544847423032333700000000 ..-.THGB0237....
20: 0000000034000000FFFFFFFFAA55AA55 ....4........U.U
30: 883355AA660300000000001004ED00E0 .3U.f...........
40: 5A5032000036354C400D0300400D0300 ZP2..65L@...@...
50: BE002D00544847423032333700000000 ..-.THGB0237....
60: 0000000034000000FFFFFFFFAA55AA55 ....4........U.U
70: 883355AA400D0300AA55AA559F200B00 .3U.@....U.U. ..

// PLA Yellow (used)
00: 5A504F0000353347400D0300400D0300 ZPO..53G@...@...
10: BE002D00544847423031313100000000 ..-.THGB0111....
20: 0000000034000000FFFFFFFFAA55AA55 ....4........U.U
30: 883355AA15B800000000001004ED00E0 .3U.............
40: 5A504F0000353347400D0300400D0300 ZPO..53G@...@...
50: BE002D00544847423031313100000000 ..-.THGB0111....
60: 0000000034000000FFFFFFFFAA55AA55 ....4........U.U
70: 883355AA400D0300AA55AA551F0E0B00 .3U.@....U.U....

*************************************************************/
#ifndef _NANODEUNIO_LIB_H
#define _NANODEUNIO_LIB_H

#if ARDUINO >= 100
  #include <Arduino.h> // Arduino 1.0
#else
  #include <WProgram.h> // Arduino 0022
#endif

#define NANODE_MAC_DEVICE 0xa0
#define NANODE_MAC_ADDRESS 0xfa

#define CODE 0x00 //1 Byte
#define MATERIAL 0x01 //1 Byte
#define COLOR 0x02  //2 Bytes
#define DATE 0x05	//4 Bytes
#define TOTALLEN 0x08 //4 Bytes
#define NEWLEN 0x0C //4 Bytes
#define HEADTEMP 0x10	//2 Bytes
#define BEDTEMP 0x12	//2 Bytes
#define MLOC 0x14	//2 Bytes
#define DLOC 0x16	//2 Bytes
#define SN 0x18		//12 Bytes
#define CRC 0x24	//2 Bytes
#define LEN2 0x34	//4 Bytes


#define CLEAR_RED       0x32

#define PURPLE          0x41
#define BLUE            0x42 
#define NEON_TANGERINE  0x43
#define VIRDITY         0x44
#define OLIVINE         0x45
#define GOLD            0x46
#define GREEN           0x47
#define NEON_GREEN      0x48
#define SNOW_WHITE      0x49
#define NEON_YELLOW     0x4A
#define BLACK           0x4B
#define VIOLET          0x4C
#define GRAPE_PURPLE    0x4D
#define PURPURIN        0x4E 
#define CLEAR_YELLOW    0x4F
#define CLEAR_GREEN     0x50 
#define CLEAR_TANGERINE 0x51
#define RED             0x52
#define CYBER_YELLOW    0x53
#define TANGERINE       0x54 
#define CLEAR_BLUE      0x55
#define CLEAR_PURPLE    0x56
#define WHITE           0x57
#define CLEAR_MAGENTA   0x58
#define YELLOW          0x59
#define NATURE          0x5A


typedef struct {
  int8_t  tCODE;     // 0x00 // 1 Byte
  int8_t  tMATERIAL; // 0x01 // 1 Byte
  int16_t tCOLOR;    // 0x02 // 2 Bytes
  int8_t  tdum1;     // 0x04 // 1 Byte
  int32_t tDATE;     // 0x05 // 4 Bytes
  int32_t tTOTALLEN; // 0x08 // 4 Bytes
  int32_t tNEWLEN;   // 0x0C // 4 Bytes
  int16_t tHEADTEMP; // 0x10 // 2 Bytes
  int16_t tBEDTEMP;  // 0x12 // 2 Bytes
  int16_t tMLOC;     // 0x14 // 2 Bytes
  int16_t tDLOC;     // 0x16 // 2 Bytes
  char tSN[12];      // 0x18 //12 Bytes
  int16_t tCRC;      // 0x24 // 2 Bytes
  byte tdum2[13];    // 0x26 //13 Bytes
  int32_t tLEN2;     // 0x34 // 4 Bytes
  byte tdum3[4];     // 0x38 // 8 Bytes
} cart; // __attribute__((packed));
struct {
  cart low;
  cart high;
}eeprom; // __attribute__((packed));

void IncrementSerial(unsigned char * cArray, long lAddress, long lSize)
{
	unsigned char szTempBuffer[20] = {0};
	memcpy(szTempBuffer,&cArray[lAddress],lSize);
	long lSerial = atol((char *)szTempBuffer);
	lSerial--;
	sprintf((char *)szTempBuffer,"%04d",lSerial);
	memcpy(&cArray[lAddress],szTempBuffer,lSize);
}

class NanodeUNIO {
 private:
  byte addr;
 public:
  NanodeUNIO(byte address);

  boolean read(byte *buffer,word address,word length);
  boolean start_write(const byte *buffer,word address,word length);
  boolean enable_write(void);
  boolean disable_write(void);
  boolean read_status(byte *status);
  boolean write_status(byte status);
  boolean await_write_complete(void);
  boolean simple_write(const byte *buffer,word address,word length);
};

#endif /* _NANODEUNIO_LIB_H */

#define UNIO_STARTHEADER 0x55
#define UNIO_READ        0x03
#define UNIO_CRRD        0x06
#define UNIO_WRITE       0x6c
#define UNIO_WREN        0x96
#define UNIO_WRDI        0x91
#define UNIO_RDSR        0x05
#define UNIO_WRSR        0x6e
#define UNIO_ERAL        0x6d
#define UNIO_SETAL       0x67

#define UNIO_TSTBY 600
#define UNIO_TSS    10
#define UNIO_THDR    5
#define UNIO_QUARTER_BIT 10
#define UNIO_FUDGE_FACTOR 5

#if defined(__AVR__)
  #define UNIO_OUTPUT() do { DDRD |= 0x80; } while (0)
  #define UNIO_INPUT() do { DDRD &= 0x7f; } while (0)
#else
  #define UNIO_PIN  10
  #define UNIO_OUTPUT() pinMode(UNIO_PIN, OUTPUT)
  #define UNIO_INPUT() pinMode(UNIO_PIN, INPUT);

void sei() {
  enableInterrupts();
}
void cli() {
  disableInterrupts();
}
#endif

static void set_bus(boolean state) {
#if defined(__AVR__)
  PORTD=(PORTD&0x7f)|(!!state)<<7;
#else
  digitalWrite(UNIO_PIN, state);
#endif
}

static boolean read_bus(void) {
#if defined(__AVR__)
  return !!(PIND&0x80);
#else
  return digitalRead(UNIO_PIN);
#endif
}
static void unio_inter_command_gap(void) {
  set_bus(1);
  delayMicroseconds(UNIO_TSS+UNIO_FUDGE_FACTOR);
}

static void unio_standby_pulse(void) {
  set_bus(0);
  UNIO_OUTPUT();
  delayMicroseconds(UNIO_TSS+UNIO_FUDGE_FACTOR);
  set_bus(1);
  delayMicroseconds(UNIO_TSTBY+UNIO_FUDGE_FACTOR);
}

static volatile boolean rwbit(boolean w) {
  boolean a,b;
  set_bus(!w);
  delayMicroseconds(UNIO_QUARTER_BIT);
  a=read_bus();
  delayMicroseconds(UNIO_QUARTER_BIT);
  set_bus(w);
  delayMicroseconds(UNIO_QUARTER_BIT);
  b=read_bus();
  delayMicroseconds(UNIO_QUARTER_BIT);
  return b&&!a;
}

static boolean read_bit(void) {
  boolean b;
  UNIO_INPUT();
  b=rwbit(1);
  UNIO_OUTPUT();
  return b;
}

static boolean send_byte(byte b, boolean mak) {
  for (int i=0; i<8; i++) {
    rwbit(b&0x80);
    b<<=1;
  }
  rwbit(mak);
  return read_bit();
}

static boolean read_byte(byte *b, boolean mak) {
  byte data=0;
  UNIO_INPUT();
  for (int i=0; i<8; i++) {
    data = (data << 1) | rwbit(1);
  }
  UNIO_OUTPUT();
  *b=data;
  rwbit(mak);
  return read_bit();
}

static boolean unio_send(const byte *data,word length,boolean end) {
  for (word i=0; i<length; i++) {
    if (!send_byte(data[i],!(((i+1)==length) && end))) return false;
  }
  return true;
}

static boolean unio_read(byte *data,word length)  {
  for (word i=0; i<length; i++) {
    if (!read_byte(data+i,!((i+1)==length))) return false;
  }
  return true;
}

static void unio_start_header(void) {
  set_bus(0);
  delayMicroseconds(UNIO_THDR+UNIO_FUDGE_FACTOR);
  send_byte(UNIO_STARTHEADER,true);
}

NanodeUNIO::NanodeUNIO(byte address) {
  addr=address;
}

#define fail() do { sei(); return false; } while (0)

boolean NanodeUNIO::read(byte *buffer,word address,word length) {
  byte cmd[4];
  cmd[0]=addr;
  cmd[1]=UNIO_READ;
  cmd[2]=(byte)(address>>8);
  cmd[3]=(byte)(address&0xff);
  unio_standby_pulse();
  cli();
  unio_start_header();
  if (!unio_send(cmd,4,false)) fail();
  if (!unio_read(buffer,length)) fail();
  sei();
  return true;
}

boolean NanodeUNIO::start_write(const byte *buffer,word address,word length) {
  byte cmd[4];
  if (((address&0x0f)+length)>16) return false; // would cross page boundary
  cmd[0]=addr;
  cmd[1]=UNIO_WRITE;
  cmd[2]=(byte)(address>>8);
  cmd[3]=(byte)(address&0xff);
  unio_standby_pulse();
  cli();
  unio_start_header();
  if (!unio_send(cmd,4,false)) fail();
  if (!unio_send(buffer,length,true)) fail();
  sei();
  return true;
}

boolean NanodeUNIO::enable_write(void) {
  byte cmd[2];
  cmd[0]=addr;
  cmd[1]=UNIO_WREN;
  unio_standby_pulse();
  cli();
  unio_start_header();
  if (!unio_send(cmd,2,true)) fail();
  sei();
  return true;
}

boolean NanodeUNIO::disable_write(void) {
  byte cmd[2];
  cmd[0]=addr;
  cmd[1]=UNIO_WRDI;
  unio_standby_pulse();
  cli();
  unio_start_header();
  if (!unio_send(cmd,2,true)) fail();
  sei();
  return true;
}

boolean NanodeUNIO::read_status(byte *status) {
  byte cmd[2];
  cmd[0]=addr;
  cmd[1]=UNIO_RDSR;
  unio_standby_pulse();
  cli();
  unio_start_header();
  if (!unio_send(cmd,2,false)) fail();
  if (!unio_read(status,1)) fail();
  sei();
  return true;
}

boolean NanodeUNIO::write_status(byte status) {
  byte cmd[3];
  cmd[0]=addr;
  cmd[1]=UNIO_WRSR;
  cmd[2]=status;
  unio_standby_pulse();
  cli();
  unio_start_header();
  if (!unio_send(cmd,3,true)) fail();
  sei();
  return true;
}

boolean NanodeUNIO::await_write_complete(void) {
  byte cmd[2];
  byte status;
  cmd[0]=addr;
  cmd[1]=UNIO_RDSR;
  unio_standby_pulse();
  do {
    unio_inter_command_gap();
    cli();
    unio_start_header();
    if (!unio_send(cmd,2,false)) fail();
    if (!unio_read(&status,1)) fail();
    sei();
  } while (status&0x01);
  return true;
}

boolean NanodeUNIO::simple_write(const byte *buffer,word address,word length) {
  word wlen;
  while (length>0) {
    wlen=length;
    if (((address&0x0f)+wlen)>16) {
      wlen=16-(address&0x0f);
    }
    if (!enable_write()) return false;
    if (!start_write(buffer,address,wlen)) return false;
    if (!await_write_complete()) return false;
    buffer+=wlen;
    address+=wlen;
    length-=wlen;
  }
  return true;
}

static void status(boolean r)
{
  if (r) Serial.println("(success)");
  else Serial.println("(failure)");
}

uint8_t eeprom_buf[128];
uint8_t&  elCODE     = *(uint8_t *) &eeprom_buf[CODE];
uint8_t&  elMATERIAL = *(uint8_t *) &eeprom_buf[MATERIAL];
uint16_t& elCOLOR    = *(uint16_t *)&eeprom_buf[COLOR];
uint32_t& elDATE     = *(uint32_t *)&eeprom_buf[DATE];
uint32_t& elTOTALLEN = *(uint32_t *)&eeprom_buf[TOTALLEN];
uint32_t& elNEWLEN   = *(uint32_t *)&eeprom_buf[NEWLEN];
uint16_t& elHEADTEMP = *(uint16_t *)&eeprom_buf[HEADTEMP];
uint16_t& elBEDTEMP  = *(uint16_t *)&eeprom_buf[BEDTEMP];
uint16_t& elMLOC     = *(uint16_t *)&eeprom_buf[MLOC];
uint16_t& elDLOC     = *(uint16_t *)&eeprom_buf[DLOC];
//uint8_t& elSN      = *(uint8_t *) &eeprom_buf[SN];
uint16_t& elCRC      = *(uint16_t *)&eeprom_buf[CRC];
uint32_t& elLEN2     = *(uint32_t *)&eeprom_buf[LEN2];

uint8_t&  euCODE     = *(uint8_t *) &eeprom_buf[0x40 + CODE];
uint8_t&  euMATERIAL = *(uint8_t *) &eeprom_buf[0x40 + MATERIAL];
uint16_t& euCOLOR    = *(uint16_t *)&eeprom_buf[0x40 + COLOR];
uint32_t& euDATE     = *(uint32_t *)&eeprom_buf[0x40 + DATE];
uint32_t& euTOTALLEN = *(uint32_t *)&eeprom_buf[0x40 + TOTALLEN];
uint32_t& euNEWLEN   = *(uint32_t *)&eeprom_buf[0x40 + NEWLEN];
uint16_t& euHEADTEMP = *(uint16_t *)&eeprom_buf[0x40 + HEADTEMP];
uint16_t& euBEDTEMP  = *(uint16_t *)&eeprom_buf[0x40 + BEDTEMP];
uint16_t& euMLOC     = *(uint16_t *)&eeprom_buf[0x40 + MLOC];
uint16_t& euDLOC     = *(uint16_t *)&eeprom_buf[0x40 + DLOC];
//uint8_t& euSN      = *(uint8_t *) &eeprom_buf[0x40 + SN];
uint16_t& euCRC      = *(uint16_t *)&eeprom_buf[0x40 + CRC];
uint32_t& euLEN2     = *(uint32_t *)&eeprom_buf[0x40 + LEN2];

static void dump_eeprom(word address,word length)
{
  byte *buf = (byte *)&eeprom; //[128];
  char lbuf[80];
  char *x;
  int i,j;

  NanodeUNIO unio(NANODE_MAC_DEVICE);
  
  memset(eeprom_buf,0,128);
  status(unio.read(eeprom_buf,address,length));
  
  for (i=0; i<128; i+=16) {
    x=lbuf;
    sprintf(x,"%02X: ",i);
    x+=4;
    for (j=0; j<16; j++) {
      sprintf(x,"%02X",eeprom_buf[i+j]);
      x+=2;
    }
    *x=32;
    x+=1;
    for (j=0; j<16; j++) {
      if (eeprom_buf[i+j]>=32 && eeprom_buf[i+j]<127) *x=eeprom_buf[i+j];
      else *x=46;
      x++;
    }
    *x=0;
    Serial.println(lbuf);
  }
}

int led = LED_BUILTIN;

/*
These are the values to be written to the EEPROM
Make sure only one is uncommented.
By default its set for the starter ABS cartdridge with 120m of filament 

Verified with firmware 1.1.I
*/

// Value to write to the EEPROM for remaining filament lenght
// Default Starter Cartdridge is 120m
//char x[] = {0xc0,0xd4,0x01,0x00}; //120m
char x[] = {0x80,0xa9,0x03,0x00}; //240m
//char x[] = {0x80,0x1a,0x06,0x00}; //400m

// extruder temp
//char et[] = {0xd2,0x00}; // 210 C default for ABS
//char et[] = {0xe6,0x00}; // 230 C
//char et[] = {0xf5,0x00}; // 245 C
//char et[] = {0xfa,0x00}; // 250 C
char et[] = {0xbe,0x00}; // 190 C default for PLA

// bed temp
//char bt[] = {0x5a,0x00}; //90C default for ABS
//char bt[] = {0x32,0x00}; //50C
char bt[] = {0x2d,0x00}; //40C default for PLA
//char bt[] = {0x28,0x00}; //40C

//Materials
//char mt[] = {0x41}; //ABS (ASCII A)
char mt[] = {0x50}; //PLA (ASCII P)
//char mt[] = {0x46}; //Flex (ASCII F)

byte serial_number[20];

byte sr;
NanodeUNIO unio(NANODE_MAC_DEVICE);
  
void setup() {
  pinMode(led, OUTPUT);
  Serial.begin(115200);
  while(!Serial);
  delay(250);
  test_DaVinci_EEPROM();
  read_DaVinci_EEPROM();
}

void test_DaVinci_EEPROM() {
  do {
    digitalWrite(led, LOW); 
    Serial.println("Testing connection to Da Vinci EEPROM CHIP\n");    
    delay(100);
    digitalWrite(led, HIGH);
  } while(!unio.read_status(&sr));
  Serial.println("Da Vinci EEPROM found...");
}

void read_DaVinci_EEPROM(){
  Serial.println("Reading the Davinci EEPROM Contents...");
  dump_eeprom(0,128);
  //dump_eeprom(116,4);
	
  //Read the serial number - added by Matt
  memset(serial_number,0,20);
  status(unio.read(serial_number,SN,12));
  //Increment the serial number
  IncrementSerial(&serial_number[0], 0, 12);	
}

void update_DaVinci_EEPROM() {
  Serial.println("Updating EEPROM...");
  status(unio.simple_write((const byte *)x,TOTALLEN,4));
  status(unio.simple_write((const byte *)x,NEWLEN,4));
  status(unio.simple_write((const byte *)et,HEADTEMP,2)); // extruder temp
  status(unio.simple_write((const byte *)bt,BEDTEMP,2)); // bed temp
  status(unio.simple_write((const byte *)mt,MATERIAL,1)); // Material
  //Write the serial number
  status(unio.simple_write((const byte *)serial_number,SN,12)); //Serial Number
  status(unio.simple_write((const byte *)x,LEN2,4));
  
  // same block from offset 0 is offset 64 bytes
  status(unio.simple_write((const byte *)x,64 + TOTALLEN,4));
  status(unio.simple_write((const byte *)x,64 + NEWLEN,4));
  status(unio.simple_write((const byte *)et,64 + HEADTEMP,2)); // extruder temp
  status(unio.simple_write((const byte *)bt,64 + BEDTEMP,2)); // bed temp
  status(unio.simple_write((const byte *)mt,64 + MATERIAL,1)); // Material
   //Write the serial number
  status(unio.simple_write((const byte *)serial_number,64 + SN,12)); //Serial Number
  status(unio.simple_write((const byte *)x,64 + LEN2,4));

  Serial.println("Dumping Content after modification...");
  dump_eeprom(0,128);
}
//#ifdef GONE
void parse_DaVinci_EEPROM() {
  char temp[80];
  
  Serial.println("Parsing EEPROM...");
  sprintf(temp, "%02X     CODE: %02X %02X", CODE,     elCODE,     euCODE);      Serial.println(temp);
  sprintf(temp, "%02X MATERIAL: %02X %02X", MATERIAL, elMATERIAL, euMATERIAL);  Serial.println(temp);
  sprintf(temp, "%02X    COLOR: %04X %04X", COLOR,    elCOLOR,    euCOLOR);     Serial.println(temp);
  sprintf(temp, "%02X     DATE: %08X %08X", DATE,     elDATE,     euDATE);      Serial.println(temp);
  sprintf(temp, "%02X TOTALLEN: %08X %08X", TOTALLEN, elTOTALLEN, euTOTALLEN);  Serial.println(temp);
  sprintf(temp, "%02X   NEWLEN: %08X %08X", NEWLEN,   elNEWLEN,   euNEWLEN);    Serial.println(temp);
  sprintf(temp, "%02X HEADTEMP: %04X %04X", HEADTEMP, elHEADTEMP, euHEADTEMP);  Serial.println(temp);
  sprintf(temp, "%02X  BEDTEMP: %04X %04X", BEDTEMP,  elBEDTEMP,  euBEDTEMP);   Serial.println(temp);
  sprintf(temp, "%02X     MLOC: %04X %04X", MLOC,     elMLOC,     euMLOC);      Serial.println(temp);
  sprintf(temp, "%02X     DLOC: %04X %04X", DLOC,     elDLOC,     euDLOC);      Serial.println(temp);
  //Serial.println(elSN); //12
  sprintf(temp, "%02X      CRC: %04X %04X", CRC,      elCRC,      euCRC);       Serial.println(temp);
  sprintf(temp, "%02X     LEN2: %08X %08X", LEN2,     elLEN2,     euLEN2);      Serial.println(temp);
}
//#endif

char command[0x20];
void loop() {
  static bool enable_write = false;
  if (receive_function(command, sizeof(command)))
  {
    if (strcmp(command, "?") == 0)
    {
      Serial.println("1 Test EEPROM");
      Serial.println("2 Read EEPROM");
      Serial.println("3 Update EEPROM");
      Serial.println("4 Parse EEPROM");
      Serial.println("enable enable_write = 1");
      Serial.println("disable enable_write = 0");
      Serial.println("reset");
      Serial.println("");
      Serial.print("enable = ");
      Serial.println(enable_write);
      
    }
    else if (strcmp(command, "1") == 0) test_DaVinci_EEPROM();
    else if (strcmp(command, "2") == 0) read_DaVinci_EEPROM();
    else if (strcmp(command, "3") == 0) { if(enable_write) update_DaVinci_EEPROM(); }
    else if (strcmp(command, "4") == 0) parse_DaVinci_EEPROM();
    else if (strcmp(command, "enable") == 0) enable_write = true;
    else if (strcmp(command, "disable") == 0) enable_write = false;
    else if (strcmp(command, "reset") == 0) reset();
    else
    {
      Serial.println("Command not found, use ? for help.");
    }
  }
}

