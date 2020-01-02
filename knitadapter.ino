#include <avr/interrupt.h>
#include <avr/io.h>

#define FIRST_STEP false
#define SECOND_STEP true

const unsigned char bufLen = 200;
unsigned char pos;
unsigned char inputBuf[bufLen];
bool cmdComplete = false;
bool initiated = false;
bool started = false;
bool firstLine = true;
bool bStep = FIRST_STEP;
const byte ledPin = 13;
const byte interruptPin = 2;
volatile bool next_line = false;
unsigned char link = 0;

const unsigned char COMMAND_FIND = 0x06;
const unsigned char REQUEST_FIND[] = {COMMAND_FIND, 0xF9, 0x07, 0xF8, 0x00, 0xFF};
const unsigned char REQUEST_FIND_LEN = 6;

const unsigned char RESPONSE_FIND[] = {
  0x01, 0x41, 0x72, 0x64, 0x75, 0x69, 0x6e, 0x6f, 0x20, 0x4b, 
  0x6e, 0x69, 0x74, 0x74, 0x20, 0x4d, 0x61, 0x63, 0x68, 0x69, 
  0x6e, 0x65, 0x20, 0x61, 0x64, 0x61, 0x70, 0x74, 0x65, 0x72, 
  0x0a, 0x47, 0x72, 0x61, 0x79, 0x59, 0x6f, 0x67, 0x69, 0x20, 
  0x28, 0x63, 0x29, 0x20, 0x32, 0x30, 0x31, 0x39, 0x0a, 0x00, 
  0xF7, 0x0D
};
const unsigned char RESPONSE_FIND_LEN = 52;

const unsigned char COMMAND_INIT = 0x07;
const unsigned char REQUEST_INIT[] = {COMMAND_INIT, 0xF8, 0x00, 0xFF};
const unsigned char REQUEST_INIT_LEN = 4;
const unsigned char RESPONSE_INIT_LEN = 8;
const unsigned char RESPONSE_INIT[] = {0x08, 0x0D, 0x00, 0x31, 0x31, 0x94, 0x00, 0xF7};

const unsigned char COMMAND_START = 0x01;
unsigned char signature[2] = {0x00, 0x00};

unsigned char KEEP_ALIVE_1[] =  {0x08, 0x08, 0x00, 0x31, 0x32, 0x95, 0x00, 0xF7};
const unsigned char KEEP_ALIVE_1_LEN = 8;

unsigned char ADD_LINE_1[] =    {0x08, 0x13, 0x00, 0x95, 0x32, 0x95, 0x00, 0xF7};
const unsigned char ADD_LINE_1_LEN = 8;

unsigned char KEEP_ALIVE_2[] =  {0x08, 0x02, 0x00, 0x95, 0x32, 0x95, 0x00, 0xF7};
const unsigned char KEEP_ALIVE_2_LEN = 8;

unsigned char ADD_LINE_2[] =    {0x08, 0x19, 0x00, 0x32, 0x32, 0x95, 0x00, 0xF7};
const unsigned char ADD_LINE_2_LEN = 8;

void setup() {
  pos = 0;
  pinMode(ledPin, OUTPUT);  // pin 13 as output
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), int0_callback, FALLING);

  UBRR0 = 103; // baud rate of 9600bps
  UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
  // Use 8-bit character sizes
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
  sei();
}

void int0_callback() {
  static unsigned long millis_prev;
  if(millis()-250 > millis_prev) next_line = true;   // устранение дребезга
  millis_prev = millis();
}

bool isRequestValid(unsigned char * request, const unsigned char * expect, int len) {
  for (int i = 0; i < len; i++) {
    if (request[i] != expect[i])
      return false;
  }
  return true;
}

bool isRequestInit(unsigned char * request) {
  return isRequestValid(request, REQUEST_INIT, REQUEST_INIT_LEN);
}

bool isRequestFind(unsigned char * request) {
  return isRequestValid(request, REQUEST_FIND, REQUEST_FIND_LEN);
}

void keepAlive() {
  if (bStep == FIRST_STEP) {
    if (firstLine) {
      firstLine = false;
    } else {
      KEEP_ALIVE_1[3] = signature[0];
    }
    KEEP_ALIVE_1[4] = signature[0];
    KEEP_ALIVE_1[5] = signature[1];
    SerialWrite(KEEP_ALIVE_1, KEEP_ALIVE_1_LEN);
  } else {
    KEEP_ALIVE_2[3] = signature[1];
    KEEP_ALIVE_2[4] = signature[0];
    KEEP_ALIVE_2[5] = signature[1];
    SerialWrite(KEEP_ALIVE_2, KEEP_ALIVE_2_LEN);
  }
}

void addLine() {
  if (bStep == FIRST_STEP) {
    ADD_LINE_1[3] = signature[1];
    ADD_LINE_1[4] = signature[0];
    ADD_LINE_1[5] = signature[1];
    SerialWrite(ADD_LINE_1, ADD_LINE_1_LEN);
  } else {
    ADD_LINE_2[3] = signature[0];
    ADD_LINE_2[4] = signature[0];
    ADD_LINE_2[5] = signature[1];
    SerialWrite(ADD_LINE_2, ADD_LINE_2_LEN);
  }
}
void loop() {
  if (cmdComplete) {
    switch (inputBuf[0]) {
      case COMMAND_FIND:
        if (isRequestFind(inputBuf)) {
          SerialWrite(RESPONSE_FIND, RESPONSE_FIND_LEN);
        }
        break;
      case COMMAND_INIT:
        if (isRequestInit(inputBuf)) {
          SerialWrite(RESPONSE_INIT, RESPONSE_INIT_LEN);
          delay(500);
          SerialWrite(RESPONSE_INIT, RESPONSE_INIT_LEN);
          initiated = true;
        }
        break;
      case COMMAND_START:
        if (!started) {
          signature[0] = inputBuf[35];
          signature[1] = inputBuf[36];
          started = true;
        }
        else {
          link = 0;
        }
        break;
      default:
        break;
    }
    pos = 0;
    cmdComplete = false;
  }
  if (started) {
    delay(400);
    if (!next_line) {
      keepAlive();
      // если после новой линии не получили ответ от компьютера, сбрасываем состояние адаптера к ожиданию связи
//      if (link != 0) {
//        initiated = false;
//        started = false;
//      }
      digitalWrite(ledPin, LOW);
    } else {
      addLine();
      link = 1;
      next_line = false;
      digitalWrite(ledPin, HIGH);
    }
  }
}

void SerialWrite(const unsigned char * buf, int len) {
  for (int i = 0; i < len; i++) {
    USARTWriteChar(buf[i]);
  }
}

void USARTWriteChar(char data)
{
  //Wait untill the transmitter is ready
  while ( !( UCSR0A & (1 << UDRE0)) );
  //Now write the data to USART buffer
  UDR0 = data;
}

ISR(USART_RX_vect, ISR_BLOCK)
{
  unsigned char buf = UDR0;// read the received data byte in temp
  inputBuf[pos++] = buf;
  if (buf == 0xFF) {
    cmdComplete = true;
  }
  if (initiated && buf == 0xFD) {
    cmdComplete = true;
  }
  if (started && buf == 0xFE) {
    cmdComplete = true;
  }
}
