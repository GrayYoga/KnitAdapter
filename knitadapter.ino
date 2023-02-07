#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/eeprom.h>

#define FIRST_STEP false
#define SECOND_STEP true

#define BAUD 9600
#define UBRRn (F_CPU/16/BAUD - 1)

volatile unsigned char pos = 0, prevpos = 0;
volatile unsigned char inputBuf[200];
volatile bool cmdComplete = false;
volatile bool initiated = false;
volatile bool started = false;
volatile bool DTR = false;
bool firstLine = true;
bool bStep = FIRST_STEP;
const byte ledPin = 13;
const byte COUNTER_PIN = 3;
const byte DTR_PIN = 2;
const byte DEBUG_PIN = 10; // oscilloscope debug 
volatile bool next_line = false;
unsigned char link = 0;

// const unsigned char FIND_SEQUENCE_END = 0xFF;
const unsigned char FIND_SEQUENCE_END = 0xF9;

// 05 fa 06 f9 07 f8 00 ff
const unsigned char COMMAND_FIND = 0x05;
// const unsigned char REQUEST_FIND[] = {COMMAND_FIND, 0xFA, 0x06, 0xF9, 0x07, 0xF8, 0x00, FIND_SEQUENCE_END};
const unsigned char REQUEST_FIND[] = {COMMAND_FIND, 0xFA, 0x06, FIND_SEQUENCE_END};
// const unsigned char REQUEST_FIND_LEN = 8;
const unsigned char REQUEST_FIND_LEN = 4;

const unsigned char RESPONSE_FIND[] = {
  0x0D, 0x01, 0x41, 0x72, 0x64, 0x75, 0x69, 0x6e, 0x6f, 0x20, 
  0x4b, 0x6e, 0x69, 0x74, 0x74, 0x20, 0x4d, 0x61, 0x63, 0x68,
  0x69, 0x6e, 0x65, 0x20, 0x61, 0x64, 0x61, 0x70, 0x74, 0x65,
  0x72, 0x0a, 0x47, 0x72, 0x61, 0x79, 0x59, 0x6f, 0x67, 0x69,
  0x20, 0x28, 0x63, 0x29, 0x20, 0x32, 0x30, 0x31, 0x39, 0x08,
  0x0d, 0x00, 0x31, 0x31, 0x94, 0x00, 0xf7
};
const unsigned char RESPONSE_FIND_LEN = 57;

const unsigned char COMMAND_ECHO = 0x65;
const unsigned char COMMAND_ECHO_END = 0x6F;

const unsigned char REQUEST_ECHO[] = {COMMAND_ECHO, 0x63, 0x68, COMMAND_ECHO_END};
const unsigned char REQUEST_ECHO_LEN = 4;
const unsigned char RESPONSE_ECHO_LEN = 8;
const unsigned char RESPONSE_ECHO[] = {COMMAND_ECHO, 0x63, 0x68, COMMAND_ECHO_END, ' ', 'O', 'K',  '\n'};

const unsigned char COMMAND_INIT = 0x07;
const unsigned char COMMAND_INIT_END = 0xFF;
const unsigned char REQUEST_INIT[] = {COMMAND_INIT, 0xF8, 0x00, COMMAND_INIT_END};
const unsigned char REQUEST_INIT_LEN = 4;
const unsigned char RESPONSE_INIT_LEN = 8;
const unsigned char RESPONSE_INIT[] = {0x08, 0x0D, 0x00, 0x31, 0x31, 0x94, 0x00, 0xF7};
// 08 8d 40  31 31 94 00 f7
const unsigned char COMMAND_START = 0x01;
const unsigned char COMMAND_START_END = 0xFD;

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
  cli();
  pos = 0;
  for (int i = 0; i < 200; i++) inputBuf[i] = 0;
  pinMode(ledPin, OUTPUT);
  pinMode(COUNTER_PIN, INPUT_PULLUP);
  pinMode(DTR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(COUNTER_PIN), int0_callback, FALLING);
  UCSR0A = 0;
  UCSR0B = 0;
  UCSR0C = 0;
  UBRR0H = (unsigned char)(UBRRn >> 8);
  UBRR0L = (unsigned char)UBRRn;
  UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
  // Use 8-bit character sizes
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
  UCSR0A = (0 << U2X0);
  sei();
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void int0_callback() {
  static unsigned long millis_prev, cur_millis;
  cur_millis = millis();  
  if ((cur_millis - 250) > millis_prev) next_line = true; // устранение дребезга
  millis_prev = cur_millis;
}

bool isRequestValid(unsigned char * request, const unsigned char * expect, unsigned char len) {
  for (unsigned char i = 0; i < len; i++) {
    if (request[i] != expect[i])
      return false;
  }
  return true;
}

unsigned char reversedRequestValid(unsigned char * request, unsigned char real_len, const unsigned char * expect, unsigned char expected_len) {
  unsigned char len = expected_len;
  if (real_len < expected_len) {
    len = real_len;
  }
  unsigned char i = 0;
  for (i = 0; i < len; i--) {
    if (request[--real_len] != expect[--expected_len])
      break;
  }
  return i;
}

bool isRequestInit(unsigned char * request) {
  return isRequestValid(request, REQUEST_INIT, REQUEST_INIT_LEN);
}

bool isRequestFindReverseCheck(unsigned char * request, unsigned char len) {
  byte result = reversedRequestValid(request, len, REQUEST_FIND, REQUEST_FIND_LEN);
  return result > 1;
}

bool isRequestEcho(unsigned char * request) {
  return isRequestValid(request, REQUEST_ECHO, REQUEST_ECHO_LEN);
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
  if (isRequestFindReverseCheck(inputBuf, pos)){
    SerialWrite(RESPONSE_FIND, RESPONSE_FIND_LEN);
    pos = 0;
  } else if (cmdComplete) {
    switch (inputBuf[0]) {
      case COMMAND_ECHO:
        if (isRequestEcho(inputBuf)) {
          SerialWrite(RESPONSE_ECHO, RESPONSE_ECHO_LEN);
        }
        break;
      case COMMAND_INIT:
        if (isRequestInit(inputBuf)) {
          SerialWrite(RESPONSE_INIT, RESPONSE_INIT_LEN);
          // delay(500);
          // SerialWrite(RESPONSE_INIT, RESPONSE_INIT_LEN);
          initiated = true;
        }
        break;
      case COMMAND_START:
        if (!started) {
          signature[0] = inputBuf[35];
          signature[1] = inputBuf[36];
          started = true;
        }
        break;
      default:
        pos = 0;
        cmdComplete = false;
        break;
    }

    pos = 0;
    cmdComplete = false;
  }
  if (started) {
    delay(400);
    if (!next_line) {
      keepAlive();
      digitalWrite(ledPin, LOW);
    } else {
      addLine();
      next_line = !next_line;
      link = 1;
      digitalWrite(ledPin, HIGH);
    }
  } 
  digitalWrite(ledPin, LOW);
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
  unsigned char buf = UDR0;
  inputBuf[pos++] = buf;
  if (buf == COMMAND_INIT_END) {
    cmdComplete = true;
  }
  if (initiated && buf == COMMAND_START_END) {  
    next_line = false;
    cmdComplete = true;
  }
  if (started && buf == 0xFE) {  // cmd stop (?)
    cmdComplete = true;
  }
  if (!started && !initiated && buf == COMMAND_ECHO_END) {
    cmdComplete = true;
  }
}
