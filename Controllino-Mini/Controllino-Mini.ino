#include <Controllino.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include <stdlib.h>
#include <SoftwareSerial.h>


#define d0 CONTROLLINO_D0
#define d1 CONTROLLINO_D1
#define d2 CONTROLLINO_D2
#define d3 CONTROLLINO_D3
#define d4 CONTROLLINO_D4
#define d5 CONTROLLINO_D5
#define d6 CONTROLLINO_D6
#define d7 CONTROLLINO_D7

#define a0 CONTROLLINO_A0
#define a1 CONTROLLINO_A1
#define a2 CONTROLLINO_A2
#define a3 CONTROLLINO_A3
#define a4 CONTROLLINO_A4
#define a5 CONTROLLINO_A5

#define RX 11
#define TX 10

SoftwareSerial mySerial(RX, TX); // RX, TX

const uint8_t numDigitalOutputs = 8;
const uint8_t numAnalogInputs = 6;
uint8_t digitalOutput[] = {d0, d1, d2, d3, d4, d5, d6, d7};
uint8_t analogInput[] = {a0, a1, a2, a3, a4, a5};

const uint8_t progversion = 6;

uint8_t serialbufferindex = 0;
const uint8_t serialbuffersize = 200;
uint8_t serialbuffer[serialbuffersize];;
bool eepromupdate = false;


struct memorystruct{
  bool meminit;
  bool debug;
  bool persistent;
  bool outPutStates[numDigitalOutputs];
  uint8_t eepromversion;
};

memorystruct memory = {
  false,
  true,
  false,
  {false, false, false, false, false, false, false, false},
  progversion,
};


void(* resetFunc) (void) = 0;

void loadeeprom(int idx = 0, bool setdefault = false){
  EEPROM.get(idx, memory);
  if(setdefault){
    memory.debug = false;
    memory.eepromversion = progversion;
    eepromupdate = true;
  }
}

void saveeeprom(int idx = 0){
  memory.meminit = true;
  EEPROM.put(idx, memory);
  eepromupdate = false;
}

void clsbuffer(){
  for(uint8_t i=0;i<serialbuffersize;i++){
    serialbuffer[i] = '\0';
  }
  serialbufferindex = 0;
}


void setup_io(uint8_t port, bool active = false, bool output = false){
  uint8_t on = LOW;
  if(active)on = HIGH;

  byte iotype = INPUT;
  if(output)iotype = OUTPUT;

  pinMode(port, iotype);
  if(output)digitalWrite(port, on);
}



void init_ouptut(){
  if(memory.debug)Serial.print(F("init output..."));
  for(uint8_t i=0;i<numDigitalOutputs; i++){
    setup_io(digitalOutput[i], false, true);
  }
  if(memory.debug)Serial.println(F("finish"));
}

void init_input(){
  if(memory.debug)Serial.print(F("init input..."));
  for(uint8_t i=0;i<numAnalogInputs; i++){
    setup_io(analogInput[i]);
  }
  if(memory.debug)Serial.println(F("finish"));
}


void switch_io(uint8_t port, bool on = false){
  uint8_t state = LOW;
  if(on)state = HIGH;
  if(memory.debug){
    Serial.print(F("switch_io Port = "));
    Serial.print(port);
    Serial.print(F(" state = "));
    Serial.println(state);
  }
  digitalWrite(port, state);
}


bool is_on(uint8_t index){
  return digitalRead(digitalOutput[index]);
}

void switch_task(uint8_t index, bool on = false){
  if(index > numDigitalOutputs)return;
  if(memory.debug){
    Serial.print(F("output = "));
    Serial.print(index);
    Serial.print(F(" switch "));
    if(on){
      Serial.println(F("on"));
    } else {
      Serial.println(F("off"));
    }
  }
  switch_io(digitalOutput[index], on);
  if(memory.persistent){
    memory.outPutStates[index] = on;
    eepromupdate = true;
  }
}

uint16_t readAnalogInput(uint8_t index){
  return analogRead(analogInput[index]);
}


int readInput(uint8_t index, bool dpin = true){
  bool result = false;
  if(dpin){
    result = digitalRead(digitalOutput[index]);
  } else {
    result = digitalRead(analogInput[index]);
  }
  return result;
}


void setup() {
  loadeeprom(0);
  if((memory.eepromversion > 254) || (memory.eepromversion < progversion)){
    loadeeprom(0, true);
  }
  delay(10);
  Serial.begin(115200);
  delay(1000);
  if(memory.debug && !Serial)memory.debug = false;
  if(memory.debug)Serial.println(F("start setup"));
  // Initialisiere digitale Ausgänge (falls nötig)
  if(memory.debug)Serial.println(F("setup output"));
  init_ouptut();
  // Initialisiere Analoge Eingänge (falls nötig)
  if(memory.debug)Serial.println(F("setup input"));
  init_input();
  if(memory.debug)Serial.println(F("setup watchdog to 8s"));
  wdt_enable(WDTO_8S);
  if(memory.debug)Serial.println(F("setup finish"));
  wdt_reset();
  if(memory.debug)Serial.println(F("setup mySerial"));
  pinMode(RX, OUTPUT);
  pinMode(TX, OUTPUT);
  digitalWrite(TX, HIGH);
  delay(1000);
  digitalWrite(TX, LOW);
  pinMode(RX, INPUT);
  if(memory.debug)Serial.println(F("start mySerial"));
  mySerial.begin(9600);
  if(memory.persistent){
    if(memory.debug)Serial.println(F("restore last Output States"));
    for(uint8_t i = 0; i < numDigitalOutputs; i++){
      if(memory.outPutStates[i])switch_task(i, memory.outPutStates[i]);
    }
  }
  if(memory.debug)Serial.println(F("Setup finish!"));
  wdt_reset();
}


unsigned long currmillis, eepromtask, myserialtask, serialtask = 0;
void loop() {
  currmillis = millis();
  if((unsigned long)(currmillis - eepromtask) >= 5000){
    if(eepromupdate){
      if(memory.debug)Serial.println(F("save settings to eeprom"));
      saveeeprom();
    }
    eepromtask = millis();
  }

  if((unsigned long)(currmillis - myserialtask) >= 10){
    mySerialEvent();
    myserialtask = millis();
  }

  if((Serial) && (unsigned long)(currmillis - serialtask) >= 20){
    SerialEvent();
    serialtask = millis();
  }

  wdt_reset();
  yield();
}

void SerialEvent(){
  bool tosend = false;
  while(Serial.available() > 0){
    tosend = true;
    char c = (char)Serial.read();
    serialbuffer[serialbufferindex++] = c;
    if(c == '\n' || c == '\r')break;
  }
  yield();
  if(tosend)ProcessSerialEvent(serialbufferindex);
}

void mySerialEvent(){
  bool tosend = false;
  while(mySerial.available() > 0){
    tosend = true;
    char c = (char)mySerial.read();
    serialbuffer[serialbufferindex++] = c;
    if(c == '\n' || c == '\r')break;
  }
  yield();
  if(tosend)ProcessSerialEvent(serialbufferindex);
}

void ProcessSerialEvent(uint8_t index) {
  char* cmd = strtok(serialbuffer, ',');
  char* sep = strchr(serialbuffer, ',');
  *sep = 0;
  ++sep;

  if(memory.debug){
    Serial.print(F("Command = "));
    Serial.println(cmd);
    Serial.print(F("seperator = "));
    Serial.println(sep); 
  }

  if(strcmp(cmd,"sw") == 0){
    uint8_t id = atoi(strtok(sep, ','));
    char* on = strchr(sep, ',');
    *on = 0;
    ++on;
    
    bool state = false;
    if(atoi(on)>0)state = true;
    switch_task(id, state);
  }
  else if(strcmp(cmd,"ra") == 0){
    if(atoi(sep) < 6 && atoi(sep) >= 0){
      mySerial.println(readInput(atoi(sep), false));
    }
  }
  else if(strcmp(cmd,"rd") == 0){
    if(atoi(sep) < 8 && atoi(sep) >= 0){
      mySerial.println(readInput(atoi(sep)));
    }
  }
  else if(strcmp(cmd,"rv") == 0){
    if(atoi(sep) < 6 && atoi(sep) >= 0){
      mySerial.println(analogRead(atoi(sep)));
    }
  }
  else if(strcmp(cmd,"readin") == 0){
    for(uint8_t i=0;i<numAnalogInputs; i++){
      mySerial.print(readInput(i, false));
      mySerial.print(F(","));
    }
    mySerial.println();
  }
  else if(strcmp(cmd,"readout") == 0){
    for(uint8_t i=0;i<numDigitalOutputs; i++){
      mySerial.print(readInput(i));
      mySerial.print(F(","));
    }
    mySerial.println();
  }
  else if(strcmp(cmd,"persist") == 0){
    if(atoi(sep)>0){
      memory.persistent = true;
    } else {
      memory.persistent = false;
    }
    eepromupdate = true;
  }
  else if(strcmp(cmd,"debug") == 0){
    if(atoi(sep)>0){
      memory.debug = true;
    } else {
      memory.debug = false;
    }
    eepromupdate = true;
  }
  else if(strcmp(cmd,"reboot") == 0){
    if(atoi(sep) > 0){
      resetFunc();
    }
  }
  clsbuffer();
}