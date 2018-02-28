#define __AVR_ATmega328__
#define F_CPU 16000000
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define DEBUG 1
#define VERBOSE 0
#if defined (ARDUINO)
  #define SERIAL_DEBUG DEBUG
#endif

#define BIT(n) (1<<(n))
#define SET_BIT(val, bitIndex) val |= (1 << bitIndex)
#define CLEAR_BIT(val, bitIndex) val &= ~(1 << bitIndex)
#define TOGGLE_BIT(val, bitIndex) val ^= (1 << bitIndex)
#define BIT_IS_SET(val, bitIndex) (val & (1 << bitIndex))

// Pins
#define CS PB2
#define SPIO PB3
#define CLK PB5
#define ENC_INC PB1
#define ENC_DEC PB0
#define IR PD4

// Encoder stuff
#define MAX_ATTENUATION 128
#define ATTENUATION_INCREMENT 2
#define LEFT_CHANNEL 0
#define RIGHT_CHANNEL 1

// IR stuff
#define IR_MIN_PULSE_WIDTH 56 // the duration of a single bit in the sequence, in tens of us
#define IR_N_SEQUENCES 3 // how many different IR codes to listen for
#define IR_N_WORDS 8
#define IR_REPEAT_N_WORDS 2
#define IR_MAX_PULSE_LEN 16 * IR_MIN_PULSE_WIDTH + 100 // longest pulse in bits (16) times length of a bit, plus a buffer
#define IR_N_PULSES 33
#define IR_DELAY_US 10
#define IR_MAX_ERROR_PERCENT 10
#define IR_ATTENUATION_STEP 4
#define IR_REPEAT_N_PULSES 2
#define IR_REPEAT_TIMEOUT_MS 100
#define IR_REPEAT_SEQ_INDEX 0

// Encoder stuff
uint8_t downSeq[4] = { 0x01, 0x03, 0x00, 0x02 };
uint8_t upSeq[4] = { 0x02, 0x00, 0x03, 0x01 };
uint8_t prevSeq[4] = { 0x03, 0x02, 0x01, 0x00 };
volatile uint8_t up = 0;
volatile uint8_t down = 0;
volatile uint8_t *prevEncoderDirection = &down;
volatile uint8_t attenuation = MAX_ATTENUATION;
volatile uint8_t encoderState;

// IR stuff
uint16_t downIRCode[IR_N_WORDS] = { 0xFFFF, 0x00A2, 0xAAA2, 0x8888, 0x8A28, 0x8A8A, 0x8A88, 0xA000 };
uint16_t upIRCode[IR_N_WORDS] = { 0xFFFF, 0x00A2, 0xAAA2, 0x8888, 0x8AA8, 0xA8A2, 0x2288, 0xA000 };
uint16_t repeatIRCode[IR_REPEAT_N_WORDS] = { 0xFFFF, 0x0800 };
int16_t *validSequences[IR_N_SEQUENCES] = { repeatIRCode, downIRCode, upIRCode };
volatile uint8_t incomingIR = 0;
volatile uint8_t *prevIRdirection = &down;

void setupInterrupts();
void setAttenuation(uint8_t attenuation, uint8_t channelAddress);
void writeSPI(uint8_t data);
uint8_t getEncoderState();
uint8_t collectIRSequence(int16_t pulseLengths[IR_N_PULSES * 2]);
int8_t matchIRSequence(int16_t pulseLengths[IR_N_PULSES * 2], uint16_t *validSequences[IR_N_SEQUENCES]);

ISR(PCINT0_vect) {
  uint8_t nextState = getEncoderState();

  up += ATTENUATION_INCREMENT * (upSeq[encoderState] == nextState);
  if (!up) {
    down += ATTENUATION_INCREMENT * (downSeq[encoderState] == nextState);
    if (!down) {
      *prevEncoderDirection += ATTENUATION_INCREMENT * (prevSeq[encoderState] == nextState);
    }
  }
  
  encoderState = nextState;
}

ISR(PCINT2_vect) {
  incomingIR = 1;
}

int main(void) {
  int16_t pulseLengths[IR_N_PULSES * 2];
  // set CS, SPIO, and CLK to outputs, all others inputs
  DDRB = BIT(CS) | BIT(SPIO) | BIT(CLK);
  CLEAR_BIT(DDRD, IR);
  // enable pull-up on pins 3 & 4
  SET_BIT(PORTB, ENC_INC);
  SET_BIT(PORTB, ENC_DEC);
  // Enable SPI in Master mode
  SPCR = BIT(SPE) | BIT(MSTR);
  setupInterrupts();
  // read initial encoder state
  encoderState = getEncoderState();
  // sync initial volume
  setAttenuation(attenuation, LEFT_CHANNEL);
  setAttenuation(attenuation, RIGHT_CHANNEL);

  #if SERIAL_DEBUG
    Serial.begin(9600);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
  #endif

  while (1) {
    // Handle encoder changes
    if (up) {
      attenuation += up;
      attenuation = attenuation > MAX_ATTENUATION ? MAX_ATTENUATION : attenuation;
      setAttenuation(attenuation, LEFT_CHANNEL);
      setAttenuation(attenuation, RIGHT_CHANNEL);
      up = 0;
      #if SERIAL_DEBUG
        Serial.println("Volume down");
      #endif
    } else if (down) {
      if (down > attenuation) attenuation = 0;
      else attenuation -= down; // Doing this subtraction would cause an overflow if down > volume
      setAttenuation(attenuation, LEFT_CHANNEL);
      setAttenuation(attenuation, RIGHT_CHANNEL);
      down = 0;
      #if SERIAL_DEBUG
        Serial.println("Volume up");
      #endif
    }
 
    // Handle IR
    if (incomingIR) {
      incomingIR = 0;
      CLEAR_BIT(PCMSK2, IR); // Disable IR interrupts while we’re polling for IR changes
      if (collectIRSequence(pulseLengths)) { // 
        #if SERIAL_DEBUG
          Serial.println("Collected potentially valid IR sequence");
        #endif
        #if SERIAL_DEBUG && VERBOSE
          for (int i = 0; i < IR_N_PULSES * 2; i++) {
            Serial.print(pulseLengths[i]);
            if (i % 2 == 0) Serial.print("\t");
            else Serial.println();
          }
          Serial.println();
        #endif
        int8_t seqIndex = matchIRSequence(pulseLengths, validSequences);
        if (seqIndex > -1) {
          #if SERIAL_DEBUG
            Serial.print("Matched sequence ");
            Serial.println(seqIndex);
          #endif
          if (validSequences[seqIndex] == downIRCode) {
            up += IR_ATTENUATION_STEP;
            prevIRdirection = &up;
          } else if (validSequences[seqIndex] == upIRCode) {
            down += IR_ATTENUATION_STEP;
            prevIRdirection = &down;
          } else if (validSequences[seqIndex] == repeatIRCode) {
            *prevIRdirection += IR_ATTENUATION_STEP;
          }
        }
      }

      SET_BIT(PCMSK2, IR); // Reenable IR interrupts
    }
  }

  return 1;
}

void setAttenuation(uint8_t attenuation, uint8_t channelAddress) {
  CLEAR_BIT(PORTB, CS);
  writeSPI(channelAddress);
  writeSPI(attenuation);
  SET_BIT(PORTB, CS);
}

void writeSPI(uint8_t data) {
  SPDR = data;
  while (!(SPSR & (1<<SPIF)));
}

inline void setupInterrupts() {
  // enable pin change interrupt
  SET_BIT(PCICR, PCIE0);
  SET_BIT(PCICR, PCIE2);
  // enable pin change interrupt on encoder pins
  SET_BIT(PCMSK0, ENC_INC);
  SET_BIT(PCMSK0, ENC_DEC);
  // enable pin change interrupt on IR pin
  SET_BIT(PCMSK2, IR);
  sei();
}

uint8_t getEncoderState() {
  return (BIT_IS_SET(PINB, ENC_INC) ? 1 : 0) << 1 | (BIT_IS_SET(PINB, ENC_DEC) ? 1 : 0);
}

uint8_t collectIRSequence(int16_t pulseLengths[IR_N_PULSES * 2]) {
  uint8_t pulseIndex = 0;

  for (uint8_t pulseIndex = 0; pulseIndex < IR_N_PULSES * 2; pulseIndex += 2) {
    uint16_t timeHigh = 0;
    uint16_t timeLow = 0;
    while (!BIT_IS_SET(PIND, IR)) {
      _delay_us(IR_DELAY_US);
      timeLow++;
      // Time out after max pulse length
      if (timeLow >= IR_MAX_PULSE_LEN) {
        #if SERIAL_DEBUG
          Serial.print("Sequence timed out at ");
          Serial.print(timeLow);
          Serial.println(" low");
          for (int i = 0; i < pulseIndex; i++) {
            Serial.print(pulseLengths[i]);
            if (i % 2 == 0) Serial.print("\t");
            else Serial.println();
          }
          Serial.println();
        #endif
        return 0;
      };
    }
    
    pulseLengths[pulseIndex] = timeLow;

    while (BIT_IS_SET(PIND, IR)) {
      _delay_us(IR_DELAY_US);
      timeHigh++;
      // Time out after max pulse length
      if (timeHigh >= IR_MAX_PULSE_LEN) {
        #if SERIAL_DEBUG
          Serial.print("Sequence timed out at ");
          Serial.print(timeHigh);
          Serial.println(" high");
          for (int i = 0; i < pulseIndex; i++) {
            Serial.print(pulseLengths[i]);
            if (i % 2 == 0) Serial.print("\t");
            else Serial.println();
          }
          Serial.println();
        #endif
        return pulseIndex + 1 == IR_REPEAT_N_PULSES * 2 - 1; // Return 1 only if the collected code might be the repeat code
      };
    }
    
    pulseLengths[pulseIndex + 1] = timeHigh;
  }

  return 1;
}

int8_t matchIRSequence(int16_t pulseLengths[IR_N_PULSES * 2], uint16_t *validSequences[IR_N_SEQUENCES]) {
  for (int i = 0; i < IR_N_SEQUENCES; i++) {
    uint8_t pulseDurationBits = 0;
    uint8_t high = BIT_IS_SET(validSequences[i][0], 15) ? 1 : 0;
    uint8_t pulseIndex = 0;
    uint8_t breakout = 0;
    for (int word = 0; word < IR_N_WORDS; word++) {
      #if SERIAL_DEBUG && VERBOSE
        Serial.print("Word ");
        Serial.print(word);
        Serial.print(": ");
        Serial.println(validSequences[i][word]);
      #endif
      for (int bit = 15; bit >= 0; bit--) {
        pulseDurationBits++;
        #if SERIAL_DEBUG && VERBOSE
          Serial.print("Bit ");
          Serial.print(bit);
          Serial.print(" (");
          Serial.print(BIT_IS_SET(validSequences[i][word], bit));
          Serial.print(", ");
          Serial.print(high);
          Serial.println(")");
        #endif
        if (!(high && BIT_IS_SET(validSequences[i][word], bit) || !high && !BIT_IS_SET(validSequences[i][word], bit))) {
          // level change in expected sequence; compare with received sequence
          uint16_t expectedDuration = pulseDurationBits * IR_MIN_PULSE_WIDTH;
          uint16_t actualDuration = pulseLengths[pulseIndex];
          uint8_t delta = expectedDuration > actualDuration ? expectedDuration - actualDuration : actualDuration - expectedDuration;
          uint8_t error = delta * 100 / expectedDuration;
          #if SERIAL_DEBUG && VERBOSE
            Serial.print("Pulse bits: ");
            Serial.print(pulseDurationBits);
            Serial.print(" ");
            Serial.println(high ? "high" : "low");
            Serial.print("Actual duration: ");
            Serial.println(actualDuration);
            Serial.print("Error: ");
            Serial.println(error);
            Serial.println();
          #endif
          if (error > IR_MAX_ERROR_PERCENT) {
            breakout = 1;
            break;
          }

          pulseDurationBits = 0;
          pulseIndex++;
          high = !high;
        }
      }

      if (breakout) break;

      // Early return path for repeat sequence since it’s fewer words
      if (i == IR_REPEAT_SEQ_INDEX && word == IR_REPEAT_N_WORDS - 1) {
        return i;
      }
    }

    if (breakout) continue;
    return i; // If we made it here, the sequence matches
  }

  return -1;
}
