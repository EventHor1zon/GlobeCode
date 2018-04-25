/*
*   Program:  Persistence of Vision Globe
*   Version:  0.1
*   Date:     21/3/18
*   Author:   R Miller
*   Language: C++
*   Compiler: avrdude via Arduino IDE
*   IDE:      Atom IDE, Arduino IDE
*   Platform: ATMega328p @ 16mHz
*
*   Graded Unit 2 Project - City of Glasgow College
*   HND Electronic Engineering
*/

/*  For Psuedo-code, see Logbook OR /Project/Source Code/Psuedo
*   programming hints - DONT use delays (time sensitive operation)
*                      DO disable interrupts whilst adjusting 16-bit timer1 registers
*
*   Will use 16-bit Timer1 for timing events
*     Registers -  TCNT1, OCR1A, OCR1B, ICR1
*
*  Note - will be using low-level C where possible, as familiarity with AVR code is a project objective
*  Note - millis() check the timer it uses!
*/

#define MAX 0xFFFF                // define max for comparisson


unsigned int T_NOW;               // unsigned int works with the 16 bit timers
unsigned int T_LAST;              // t_now, t_last used for working with rotation speed tracking
unsigned int FRAME_T=0;           // frame_t is time between writing frames of the image
unsigned long int TEMP;           // temp is used to store OCR1A before assigning. It may be bigger than 16 bit, hence long.
unsigned int FRAME_C=0, FRAME_NO=400; //

ISR(INT0_vect){                     //  Interrupt Vector on PORTD[2] - nano pin D2, PCB pin 20
    // if can't turn off interrupts during an interrupt, use flags instead.
    unsigned char sreg = SREG;      // store interrupt settings
    CLI();                          // clear interrupts
    T_NOW = TCNT1;                  // get time now - rem, triggers twice per spin!
    if(T_LAST > T_NOW){             // if timer overflown, must deal with differently
      T_ROT=T_NOW+(MAX-T_LAST);     // delta_T = t_now + (difference in t_last and max)
    } else { T_ROT = T_NOW - T_LAST; } // otherwise t_rot = t_now - t_last
    T_LAST=T_NOW;                   // set t_last for next use
    FRAME_T = T_ROT/(FRAME_NO/2);   // get frame_t - triggers twice so divide frame_no by 2
    SREG=sreg;                      // restore interrupt settings
  }                                 // end interrupt

ISR(TIM1_COMPA_vect){                     // interrupt on timer match; i.e FRAME_T reached
  // is this too much code for an interrupt? Could use flags instead.
  FRAME_C++;                              // advance the frame counter
  if (FRAME_C > FRAME_NO) FRAME_C = 0;    // if frames completed, reset frame_c
  SPI_W=1;                                // set SPI write flag - don't write here, too CPU intensive
  unsigned char sreg=SREG;                // store interrupt settings
  CLI();                                  // clear interrupts
  TEMP=TCNT1+FRAME_T;                     // find value for next interrupt. In case of overflow, use temp buffer
  if(TEMP > MAX){                         // if higher than counter max value
    OCR1A = (uint16_t)(TEMP-MAX);         // next frame is t - max temp variable? May need to typecast from a larger val)
  } else { OCR1A = (uint16_t)TEMP; }      // otherwise set next interrupt to temp - may need to typecast?
  SREG=sreg;                              // restore interrupts
}

setISR(){                                 // this function initialises the interrupts
  attachInterrupt(INTRRUPT1, INT0_Vect, RISING);   // use more manual software method

}

void setup(){
  // set-up - set pin3 clk, pin4 data, pin5 enable(out), pin6 ISR(in,rot), pin7 ISR(in,fault), pin8 out(statusled)
  // pin9 input(button1), pin10 input(button2)
  setISR();
  if(DEBUG) Serial.begin(9600);            // if in debug mode, start serial port

}


void loop(){
  // loop - do initialisation then set flag; do check speed; do set led flag @ threshold; do led timer set; //
  if( vel > THRESHOLD)
  {
      if(SPI_W)
      {
        writeSPI(frame_no, numleds, etc);   // write data out SPI
              }

  }

}