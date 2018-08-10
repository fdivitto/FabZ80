#include <avr/io.h>
#include <avr/pgmspace.h>


/*
 *  PA0 = A0 (Bus Address) - Input
 *  PA1 = A1 (Bus Address) - Input
 *  PA2 = A2 (Bus Address) - Input
 *
 *  PB0 = Z80_RESET        - Output
 *  PB1 = CODE_INJECT      - Output
 *  PB2 = READY            - Output
 *  PB3 = Z80_INT          - Output
 *  PB4 = SD_SPI_CS        - Output
 *  PB5 = SPI_MOSI         - Output
 *  PB6 = SPI_MISO         - Input
 *  PB7 = SPI_SCK          - Output
 *
 *  PC0 = D0 (Bus Data)    - Input / Output
 *  PC1 = D1 (Bus Data)    - Input / Output
 *  PC2 = D2 (Bus Data)    - Input / Output
 *  PC3 = D3 (Bus Data)    - Input / Output
 *  PC4 = D4 (Bus Data)    - Input / Output
 *  PC5 = D5 (Bus Data)    - Input / Output
 *  PC6 = D6 (Bus Data)    - Input / Output
 *  PC7 = D7 (Bus Data)    - Input / Output
 *
 *  PD0 = UART_RX          - Input
 *  PD1 = UART_TX          - Output
 *  PD2 = Z80_IORQ         - Input
 *  PD3 = Z80_MREQ         - Input
 *  PD4 = Z80_WR           - Input
 *  PD5 = Z80_RD           - Input
 *  PD7 = Z80_CLOCK        - Output
 */


// Z80 boot code
#include "bootcode.h"



// Define UART bauds
#define UART_BAUDS 2000000


// Define Z80 clock frequency in Hertz
// Maximum value is: 8000000 Hertz (with F_CPU = 16000000)
// Minimum value is: 31250 hertz with (F_CPU = 16000000)
#define F_Z80 8000000


// Wait for one Z80 clycle
// This should be changed when F_CPU or F_Z80 is changed
// Each "nop" is 62.5ns
// These tests are done:
//   F_Z80 = 8000000 : 0-2 nop
//   F_Z80 = 6000000 : 0-2 nop
//   F_Z80 = 5000000 : 0-2 nops
//   F_Z80 = 4000000 : 2-4 nops
//   F_Z80 = 3000000 : 2-4 nops
//   F_Z80 = 2000000 : >=6 nops
//   F_Z80 = 1000000 : >=14 nops
//#define WAIT1C __asm__("nop\n\tnop\n\t");
#define WAIT1C


// injecting states
int8_t injectState = 0;
uint16_t injectCodePos = 0;


void setup()
{

  /*** Configure ports direction ***/

  DDRA = 0x00;  // address bus - all input
  DDRB = _BV(PB0) | _BV(PB1) | _BV(PB2) | _BV(PB3) | _BV(PB4) | _BV(PB5) | _BV(PB7);
  DDRC = 0x00;  // data bus - all input
  DDRD = _BV(PD1) | _BV(PD7);


  /*** Pull-ups ***/

  // none


  /*** Initial state ***/

  // Z80_RESET   = 0 (Z80 reset ENABLED)
  // CODE_INJECT = 0 (code injecting ENABLED)
  // READY       = 1 (ready DISABLED)
  // Z80_INT     = 1 (INT DISABLED)
  PORTB = _BV(PB2) | _BV(PB3);


  /*** start Z80 clock ***/

  // Setup timer2 (Z80 clock) - Clear Timer on Compare Match (CTC) Mode on PD7 (OC2) pin

  // WGM21 = CTC mode, COM21 = toggle OC2 on compare match, CS20 = no prescaling
  TCCR2 = _BV(WGM21) | _BV(COM20) | _BV(CS20);

  // For example 4Mhz is (CPU_F / (2 * 4000000) - 1 = 1) at CPU_F = 16000000
  OCR2 = F_CPU / (2 * F_Z80) - 1;


  /*** Setup UART ***/

  Serial.begin(UART_BAUDS);
  _delay_ms(500); // time for UART, Z80 reset and clock stabilization

}


// return true if Z80_MREQ = 0
bool isMREQEnabled()
{
  return (PIND & _BV(PD3)) == 0;
}


// return true if Z80_IOREQ = 0
bool isIOREQEnabled()
{
  return (PIND & _BV(PD2)) == 0;
}


// return true if Z80_RD = 0
bool isRDEnabled()
{
  return (PIND & _BV(PD5)) == 0;
}


// return true if Z80_WR = 0
bool isWREnabled()
{
  return (PIND & _BV(PD4)) == 0;
}


// wait for Z80_MREQ or Z80_IOREQ, then wait for Z80_RD or Z80_WR
void waitForReq()
{
  while ((!isMREQEnabled() && !isIOREQEnabled()) || (!isRDEnabled() && !isWREnabled()))
    ;
}


// 1. set databus as output
// 2. set databus value
// 3. enable READY
// 4. wait for Z80_RD = 1
// 5. set databus as input
// 6. disable READY
void putDataBus(uint8_t value)
{
  noInterrupts();

  // databus as output
  DDRC = 0xFF;

  // set databus value
  PORTC = value;

  // READY = 0 (ready)
  PORTB &= ~_BV(PB2);

  // wait 1 Z80 cycle
  WAIT1C;

  // set databus as input
  DDRC = 0x00;

  // READY = 1 (not ready)
  PORTB |= _BV(PB2);

  interrupts();
}


// 1. read databus value
// 2. enable READY
// 3. wait for Z80_WR = 1
// 4. disable READY
uint8_t getDataBus()
{
  noInterrupts();

  // get databus value
  uint8_t d = PINC;

  // READY = 0 (ready)
  PORTB &= ~_BV(PB2);

  // wait 1 Z80 cycle
  WAIT1C;

  // READY = 1 (not ready)
  PORTB |= _BV(PB2);

  interrupts();

  return d;
}


/*
 * IO Registers:
 *
 * 0x00: UART Status (read only)
 *    bit 0..5 : number of bytes available in RX buffer
 *    bit 6    : 1 = TX buffer has space, 0 = TX buffer has no more space (do not send bytes!)
 *    bit 7    : unused
 *
 * 0x01: UART Send/Recv byte (read/write)
 *    A write sends a byte, a read receives a byte.
 *    Reading when RX buffer is empty returns 0xFF
 *    Writing when TX buffer is full locks atmega. Actually Z80 doesn't lock so
 *    next time an IO is requested atmega is still un-ready, locking also Z80,
 *    until UART will send data out.
 */
void handleIOReq()
{
  switch (PINA & 0b111)
  {
    // UART Status
    case 0x00:
      if (isRDEnabled())
      {
        uint8_t status = Serial.available() | (Serial.availableForWrite() > 0 ? (1 << 6) : 0);
        putDataBus(status);
      }
      break;

    // UART Send/Recv byte
    case 0x01:
      if (isRDEnabled())
        putDataBus(Serial.read());
      else if (isWREnabled())
        Serial.write(getDataBus());
      break;
  }
}


// test: send continuous message to serial, never exit from inject state!
/*
uint32_t v = 0;
String str = "\n";
char const * msgptr = str.c_str();
bool handleCodeInjectState_test()
{
  // handle memory read (code inject)
  switch (injectState)
  {
    // ld A, ...
    case 0:
      putDataBus(0x3E);
      ++injectState;
      break;
    case 1:
      putDataBus(*msgptr);
      ++msgptr;
      if (*msgptr == 0)
      {
        str = String(String(++v) + " hellos from Z80!\n");
        msgptr = str.c_str();
      }
      ++injectState;
      break;

    // out (01H), A
    case 2:
      putDataBus(0xD3);
      ++injectState;
      break;
    case 3:
      putDataBus(0x01);
      injectState = 0;  // restart state
      break;
  }
  return false;
}
*/


bool handleCodeInjectState()
{
  // handle memory read (code inject)
  switch (injectState)
  {
    /** LD DE, Z80DESTADDR **/
    // 0x11 -> LD DE
    case 0:
      putDataBus(0x11);
      ++injectState;
      break;
    // LOW(Z80DESTADDR)
    case 1:
      putDataBus(Z80DESTADDR & 0xFF);
      ++injectState;
      break;
    // HIGH(Z80DESTADDR)
    case 2:
      putDataBus(Z80DESTADDR >> 8 & 0xFF);
      ++injectState;
      break;

    /** LDI **/
    // 0xED -> LDI opcode prefix
    case 3:
      putDataBus(0xED);
      ++injectState;
      break;
    // 0xA0 -> LDI opcode
    case 4:
      putDataBus(0xA0);
      ++injectState;
      break;
    // read memory: put next byte of the boot code
    case 5:
      putDataBus(pgm_read_byte(&Z80BOOTCODE[injectCodePos++]));
      if (injectCodePos == sizeof(Z80BOOTCODE))
        injectState = 6;  // end of boot code
      else
        injectState = 3;  // load next
      break;

    /** JP Z80DESTADDR **/
    // 0xC2 -> JP
    case 6:
      putDataBus(0xC3);
      ++injectState;
      break;
    // LOW(Z80DESTADDR)
    case 7:
      putDataBus(Z80DESTADDR & 0xFF);
      ++injectState;
      break;
    // HIGH(Z80DESTADDR)
    case 8:
      putDataBus(Z80DESTADDR >> 8 & 0xFF);
      ++injectState;
      return true;  // terminate inject

  }
  return false;
}


void loop()
{
  /*** Code injecting, handling memory reads only ***/

  // RESET = 1 (Z80 not reset)
  PORTB |= _BV(PB0);

  while (true)
  {
    // Wait for MREQ or IOREQ, then for RD or WR
    waitForReq();

    if (isMREQEnabled() && isRDEnabled())
    {
      // handle memory read in injecting state
      if (handleCodeInjectState())
        break;  // terminate inject
    } else if (isIOREQEnabled())
    {
      // handle IO request
      handleIOReq();
    }
  } // end "state" loop

  // disable injecting mode
  PORTB |= _BV(PB1);

  /*** Running, handle IO requests only ***/
  while (true)
  {
    // Wait for MREQ or IOREQ, then for RD or WR
    waitForReq();

    if (isIOREQEnabled())
      handleIOReq();
  }

}
