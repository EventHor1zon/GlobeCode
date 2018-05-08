/*
*   Program:  Persistence of Vision Globe
*   Version:  0.4
*   Date:     8/5/18
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
*
*  Note - concerns about img storage size!
*  Note - will be using low-level C where possible, as familiarity with AVR code is a project objective
*  Note - millis() check the timer it uses!
*/

#define MAX 0xFFFF                // define max value for timer1 for comparisson
#define START   0x00              // start frame 32*0 bits
#define END     0xFF              // end frame 32 * 1 bits
#define GLOBAL  0xE8              // global setting for LEDs (111[00000-11111])
#define NUMLEDS 60                 // Number of LEDs to write to

unsigned int T_NOW;               // unsigned int works with the 16 bit timers
unsigned int T_LAST;              // t_now, t_last used for working with rotation speed tracking
unsigned int FRAME_T=0;           // frame_t is time between writing frames of the image
unsigned long int TEMP;           // temp is used to store OCR1A before assigning. It may be bigger than 16 bit, hence long.
unsigned int FRAME_C=0, FRAME_LEN=0, FRAME_NO=400; //


/***************   INTERRUPT FUNCTIONS *********************/

ISR(INT0_vect){                           //  External Interrupt Vector on PORTD[2] - nano pin D2, PCB pin 20
    // (if can't turn off interrupts during an interrupt, use flags instead...)
    unsigned char sreg = SREG;            // store interrupt settings
    CLI();                                // clear interrupts
    T_NOW = TCNT1;                        // get time now - rem, triggers twice per spin!
    if(T_LAST > T_NOW){                   // if timer overflown, must deal with differently
      T_ROT=T_NOW+(MAX-T_LAST);           // delta_T = t_now + (difference in t_last and max)
    } else { T_ROT = T_NOW - T_LAST; }    // otherwise t_rot = t_now - t_last
    T_LAST=T_NOW;                         // set t_last for next use
    FRAME_T = T_ROT/(FRAME_NO/2);         // get frame_t - triggers twice so divide frame_no by 2
    SREG=sreg;                            // restore interrupt settings
  }                                       // end interrupt

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

/***********    SPI FUNCTIONS ********************/

void spi_write(uint8_t x){                // SPI write function ( uses uint8_t as 8-byte val)
    SPDR = x;                             // pass x into the 8 bit spi data reg
    while ((SPSR & (1<<SPIF)) == 0);      // wait until SPI transaction concluded (compare SPI reg with SPI done flag)
}


void spi_start(){               // start frame 32 * 0 bits
  for(int i=0; i<4; i++){       // send 4 lots of START
    spi_write(START);           // (start is 0)
  }
}

void spi_end(){                 // end frame 32 * 1 bits
  for(int i=0; i<4; i++){       // send 4 lots of END - try more - see note
    spi_write(END);             // end = 0
  }
}

void write_frame(uint32_t *data, int no_leds){    // takes pointer to data structure & num leds
  spi_start();                                    // write startframe
  for(int i=0; i<NUMLEDS; i++){                   // for each LED
    spi_write(GLOBAL);                            // write brightness bit & 111
    spi_write((uint8_t)data[i]);                     // first 8 bits for red
    spi_write((uint8_t)(data[i] >> 8));                // next 8 bits for green
    spi_write((uint8_t)(data[i] >> 16));                // next 8 for blue
  }
  spi_end();                                      // send end frame
}
/********************* IMAGE FUNCTIONS *************/

void loadIMG(){                           // use this function to select an image based on
  switch (IMG_NO) {                       // img_no - increment with button
    case 1:
  }

}

/****************** SETUP FUNCTIONS ****************/

setISR(){                                          // this function initialises the interrupt INT0
  //attachInterrupt(INTRRUPT1, INT0_Vect, RISING); - use more manual software method
  DDRD &= ~(1 << DDD2);                            // Set PortD2 as input
  EIMSK = (1 << INT0);                             // Enable int0 in ext. int. mask reg.
  EICRA = (1 << ISC01) | (1 << ISC00);             // set up external interrupt 0 on RISING edge
  sei();                                           // arduino macro to enable global interrupts
}

void initSPI(){                           // sets up SPI interface
  PRR = (0 << PRSPI);                     // turns SPI on in Power Reduction registers
  DDRB = (1<<DDB2)|(1<<DDB3)|(1<<DDB5);     // init PortB pins as output - SS, MOSI, SCK
  SPCR = (1<<SPE)|(1<<MSTR)|(0<<SPR1)|(0<<SPR0);  // spi_enable | spi_master mode | spi speed setting
  SPSR = (1<<SPI2X);                      // set double speed on spi
  PORTB &= ~(1<<PB2);                     // set SS pin low - LEDs are dumb (technical term) but still need SS low in order to write out
}

void setup(){
  // setup function runs once.
  // set-up - setup SPI (pins done in function) ISR(rotary,"" ), ISR(fault ISR), LED (statusled)
  //          setup ISR(button1) ISR(button2)
  // pin9 input(button1), pin10 input(button2)
  setISR();
  initSPI();
  if(DEBUG) Serial.begin(9600);            // if in debug mode, start serial port
}

/******************* MAIN LOOP **************/

void loop(){
  // in arduino code, this runs like a continuous while loop
  // loop - keep most code here short checks, absolutely no delays
  // mostly interrupt driven code anyways

  if(THRESHOLD)                // if going threshold speed...
  {
      if(SPI_W)                //   and SPI write flag is on
      {
        writeFrame(frameNo);   //     write data out SPI
              }
      if(ERR_CHK)             //   and error check flag is on
      {
        errorChk();           // check error pin
              }
      if(ERROR && !SHUTDOWN)  // if error flag is active but device is not shutting down
      {
        shutDown();           // shut down device.
              }
      if(BUTTON_1)            // maybe do button functionality in the interrupt?
      {
        ;
              }
      if(BUTTON_2)
      {
        ;
              }
  }
  else                                    // if not going threshold speed
  { if(!SHUTDOWN){                        // and not in process of shutting down...
        getSpeed(); }                     // check speed
    }
}
