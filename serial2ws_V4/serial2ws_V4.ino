const int ledPin1 = 13;
const int ledPin2 = 18;
const int ledPin3 = 19;
const int ledPin4 = 3;

const int minVal = 0;
const int maxVal = 255;

HardwareSerial *port;
int updatecontrol = 0;

#include <IRremote.h>
int khz = 38; // 38kHz carrier frequency
unsigned int  rawData[211] = {3400, 3400, 450, 1250, 500, 350, 450, 1300, 500, 1200, 500, 350, 450, 450, 400, 1250, 450, 450, 450, 1250,
                              450, 450, 400, 1250, 450, 450, 450, 1250, 450, 1250, 450, 1300, 450, 400, 450, 400, 450, 1250, 450, 450, 400, 450, 450, 1250, 450, 1250, 450, 450,
                              400, 1300, 450, 400, 450, 1250, 450, 450, 400, 1300, 450, 400, 450, 450, 400, 450, 400, 1300, 450, 1250, 450, 1250, 450, 1300, 450, 1250, 450, 450,
                              400, 450, 400, 450, 400, 450, 400, 450, 450, 1250, 450, 1250, 450, 450, 450, 400, 450, 400, 450, 1250, 450, 450, 400, 1250, 500, 400, 450, 1250,
                              450, 450, 400, 450, 450, 1250, 450, 450, 400, 450, 400, 450, 400, 450, 400, 450, 450, 400, 450, 400, 450, 400, 450, 450, 400, 450, 450, 400, 450,
                              400, 450, 450, 400, 400, 450, 450, 400, 450, 400, 400, 500, 400, 450, 450, 400, 400, 450, 450, 400, 450, 400, 450, 450, 400, 450, 450, 400, 400, 450,
                              450, 400, 450, 400, 450, 450, 400, 400, 500, 400, 400, 450, 450, 400, 450, 400, 450, 450, 400, 450, 450, 400, 1250, 450, 450, 400, 450, 450, 400,
                              400, 450, 450, 1250, 450, 450, 400, 450, 450, 1250, 450, 1250, 450, 450, 400, 450, 450, 400, 450
                             };
int rawsize = sizeof(rawData) / sizeof(rawData[0]);


#include <DHT.h>
#define DHTPIN 8
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
int temperature;
int moisture;

//Avoid first error shot
int inByte = 0;
int codeRead = 0;
int codeAct = 0;
int intDataA = 0;
int lastTemp = 0;
int lastHumid = 0;
int lastDC = 0;
int lastRx = 0;
int lastRy = 0;
char coi[10];
int indexcoi = 0;
int posx, posy = 500;

//unsigned long   +4.32*10^9   //50 days
unsigned long timenow = 0;
unsigned long timepress = 0;
unsigned long timepressIR = 0;
unsigned long timecert = 0;
long timespan = 0;

#include <Pixy.h>
Pixy pixy;
int VisionStatus = 0;
bool encenter = 1;

#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)
uint16_t blocks;
int32_t panError, tiltError;

class ServoLoop
{
  public:
    ServoLoop(int32_t pgain, int32_t dgain);

    void updatepos(int32_t error);
    void setgain(int32_t pgain, int32_t dgain);

    int32_t m_pos;
    int32_t m_prevError;
    int32_t m_pgain;
    int32_t m_dgain;
};

ServoLoop::ServoLoop(int32_t pgain, int32_t dgain)
{
  m_pos = PIXY_RCS_CENTER_POS;
  m_pgain = pgain;
  m_dgain = dgain;
  m_prevError = 0x80000000L;
}

ServoLoop panLoop(350, 700);
ServoLoop tiltLoop(550, 700);

void ServoLoop::setgain(int32_t pgain, int32_t dgain)
{
  m_pgain = pgain;
  m_dgain = dgain;
}

void ServoLoop::updatepos(int32_t error)
{
  long int vel;
  if (m_prevError != 0x80000000)
  {
    vel = (error * m_pgain + (error - m_prevError) * m_dgain) >> 10;

    m_pos += vel;
    if (m_pos > PIXY_RCS_MAX_POS)
      m_pos = PIXY_RCS_MAX_POS;
    else if (m_pos < PIXY_RCS_MIN_POS)
      m_pos = PIXY_RCS_MIN_POS;
  }
  m_prevError = error;
}

void setup() {

  // We need to use different serial ports on different Arduinos
  //
  // See:
  //   - Arduino/hardware/tools/avr/avr/include/avr/io.h
  //   - http://electronics4dogs.blogspot.de/2011/01/arduino-predefined-constants.html
  //
#ifdef __AVR_ATmega32U4__
  port = &Serial1; // Arduino Yun
#else
  port = &Serial;  // Arduino Mega and others
#endif

  port->begin(9600);
  Serial.begin(9600);
  dht.begin();

  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin4, OUTPUT);
  pinMode(ledPin3, OUTPUT);

  delay(1000);
  PORTC |=  B10000000;  //L13

  delay(5000);
  PORTC &= ~B10000000;  //L13
  PORTF &= ~B11000000;  //A0, A1
  timenow = millis();

  //Pixy initialize
  pixy.init();
}


void getAnalog(int pin, int id, int *last, int updatecontrol) {

  // read analog value and map/constrain to output range
  int cur = analogRead(pin);
  // if value changed, forward on serial (as ASCII)
  if (cur != *last || updatecontrol == 100) {
    *last = cur;
    port->print(id);
    port->print('\t');
    port->print(*last);
    port->println();
  }
}

void getDigital(int pin, int id, int *last, int updatecontrol) {

  // read Digital value
  int cur = digitalRead(pin);

  // if value changed, forward on serial (ASCII)
  if (cur != *last || updatecontrol == 100 ) {
    *last = cur;
    port->print(id);
    port->print('\t');
    port->print(*last);
    port->println();
  }
}

void getStatus(int Status, int id, int *last, int updatecontrol) {

  // read Digital value
  int cur = Status;

  // if value changed, forward on serial (ASCII)
  if (cur != *last || updatecontrol == 100 ) {
    *last = cur;
    port->print(id);
    port->print('\t');
    port->print(*last);
    port->println();
  }
}

void getTemperature(int id, int *last, int updatecontrol) {

  // read Temperature value
  int cur = dht.readTemperature();

  // if value changed, forward on serial (ASCII)
  if (cur != *last || updatecontrol == 100 ) {
    *last = cur;
    port->print(id);
    port->print('\t');
    port->print(*last);
    port->println();
  }
}

void getHumidity(int id, int *last, int updatecontrol) {

  // read Temperature value
  int cur = dht.readHumidity();

  // if value changed, forward on serial (ASCII)
  if (cur != *last || updatecontrol == 100 ) {
    *last = cur;
    port->print(id);
    port->print('\t');
    port->print(*last);
    port->println();
  }
}

void pantilt() {
  blocks = pixy.getBlocks();
  if (blocks) {
    panError = X_CENTER - pixy.blocks[0].x;
    tiltError = pixy.blocks[0].y - Y_CENTER;

    //double area = pixy.blocks[0].width * pixy.blocks[0].height;
    double width = pixy.blocks[0].width;
    double w =  width * 1.0 / 319 * 255;
    double r = width * 1.2 / 100;
    double d = 1 / r;

    if (d < 1.2) {
      panLoop.setgain(350 * d, 600 * d);
      tiltLoop.setgain(500 * d, 700 * d);
    } else {
      panLoop.setgain(350, 600);
      tiltLoop.setgain(550, 700);
    }

    pixy.setLED(w, w, w);

    panLoop.updatepos(panError);
    tiltLoop.updatepos(tiltError);

    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);

  }//if block end
}//pantilt end


void loop() {

  // control LED via commands read from serial (ASCII)
  if (port->available()) {
    inByte = port->read();
    timenow = millis();
    if (inByte == 88) { //start
      codeRead = 1;
      timecert = millis();
    } else if (codeRead == 1 && inByte == 32) { //EOL
      codeRead = 0;
      char getdata[3] = {coi[2], coi[3], coi[4]};
      if ((coi[2] < 32 || coi[2] > 122) || (coi[3] < 32 || coi[3] > 122) || (coi[4] < 32 || coi[4] > 122)) {
        codeAct = 0;
      } else {
        codeAct = 1;
      }
      intDataA = atoi(getdata);
      indexcoi = 0;
      coi[3] = 'x'; coi[4] = 'x';
    }
    if (codeRead == 1) { //avoid reboot shot
      timespan = timenow - timecert;
      coi[indexcoi] = inByte;
      indexcoi++;
      //restrict max length 8
      if (timespan > 15) {
        codeRead = 0;
      }
    }
  } //if port end

  if (codeAct == 1) {
    Serial.println(coi[1]);
    Serial.println(intDataA);
    if (coi[1] == 'D') {
      analogWrite(ledPin4, intDataA * 25);
      getStatus(intDataA, 30, &lastDC, 100);
    } else if (coi[1] == 'P' && intDataA >= 200) {
      pixy.setServos(intDataA, posy);
      pixy.setServos(intDataA, posy);
      posx = intDataA;
      getStatus(intDataA, 31, &lastRx, 0);
    } else if (coi[1] == 'T' && intDataA >= 200) {
      pixy.setServos(posx, intDataA);
      pixy.setServos(posx, intDataA);
      posy = intDataA;
      getStatus(intDataA, 32, &lastRy, 0);
    } else if (coi[1] == 't') {
      PORTF = (~PORTF & B01000000) | (PORTF & B10111111);
      getDigital(ledPin3, A1, &lastDC, 100);
    } else if (coi[1] == 'u') {
      getDigital(ledPin2, A0, &lastDC, 100);
      getDigital(ledPin3, A1, &lastDC, 100);
      getDigital(ledPin1, 13, &lastDC, 100);
      getStatus(VisionStatus, 20, &lastDC, 100);
    } else if (coi[1] == 'w') {
      VisionStatus = VisionStatus ^ 0x01;
      getStatus(VisionStatus, 20, &lastDC, 100);
      encenter = 1;
    } else if (coi[1] == 'v') {
      timespan = timenow - timepressIR;
      if (timespan > 200) {
        PORTF = (~PORTF & B10000000) | (PORTF & B01111111);
        getDigital(ledPin2, A0, &lastDC, 100); //Data Handshaking
        IRsend irsend;
        irsend.sendRaw(rawData, rawsize, khz);
        irsend.sendRaw(rawData, rawsize, khz);
      }
      timepressIR = millis();
    } else if (coi[1] == 'z') {
      // Buzzer time
      timespan = timenow - timepress;
      if (timespan > 600) {
        PORTC &= ~B10000000;
        getDigital(ledPin1, 13, &lastDC, 100);
        updatecontrol = 0;
        //Buzz when ready
        pinMode(10, OUTPUT);
        PORTB |=  B01000000;  //10
        for (int i = 0; i < 500; i++) {
          PORTB |=  B01000000;  //10
          delayMicroseconds(500);
          PORTB &= ~B01000000;  //10
          delayMicroseconds(200);
        }
        delay(100);
        for (int i = 0; i < 500; i++) {
          PORTB |=  B01000000;  //10
          delayMicroseconds(250);
          PORTB &= ~B01000000;  //10
          delayMicroseconds(100);
        }
        timepress = millis();
      }
    } else if (coi[1] == 'Z') {
      // Buzzer time
      timespan = timenow - timepress;
      if (timespan > 600) {
        PORTC |= B10000000;
        getDigital(ledPin1, 13, &lastDC, 100);
        updatecontrol = 0;
        //Buzz when ready
        pinMode(10, OUTPUT);
        PORTB |=  B01000000;  //10
        for (int i = 0; i < 500; i++) {
          PORTB |=  B01000000;  //10
          delayMicroseconds(250);
          PORTB &= ~B01000000;  //10
          delayMicroseconds(100);
        }
        delay(100);
        for (int i = 0; i < 500; i++) {
          PORTB |=  B01000000;  //10
          delayMicroseconds(500);
          PORTB &= ~B01000000;  //10
          delayMicroseconds(200);
        }
        timepress = millis();
      }
    }
    codeAct = 0;
  }

  //Update every 2 seconds (1 * 2000)
  if (VisionStatus) {
    pantilt();
    if (encenter) {
      pixy.setServos(500, 500);
      encenter = 0;
    }
  } else {
    updatecontrol += 1;
    getTemperature(8, &lastTemp, updatecontrol);
    getHumidity(9, &lastHumid, updatecontrol);
    if (updatecontrol == 2000) {
      updatecontrol = 0;
    }
  }

  delay(1);
} //loop end
