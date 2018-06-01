/*
*   Test code v1.1
*   This is the test code used to test the POV globe mechanics
*   code, and functionality.
*
*
*
*
*/
/*
*   Program:  Globe Test Code
*   Version:  0.5
*   Date:     10/5/18
*   Author:   R Miller
*   Language: C++
*   Compiler: avrdude via Arduino IDE
*   IDE:      Atom IDE, Arduino IDE
*   Platform: ATMega328p @ 16mHz
*
*   Graded Unit 2 Project - City of Glasgow College
*   HND Electronic Engineering
*
*   Note - Using timer 1 interrupts disables delay() function
*   Next steps: Get Pinchange interrupts and PORTB debouncing working
*               Get OCR interrupt SPI writes working (dummy frame/rotation time)
*               Test on PCB
*/


#include <avr/io.h>

#define START   0x00    // start frame 32*0 bits
#define END     0xFF    // end frame 32 * 1 bits
#define GLOBAL  0xE3    // low brightness
#define NUMLEDS 8
#define MAX     0xFFFF
#define NOP __asm__ __volatile__ ("nop\n\t")

//unsigned int t;

uint32_t test_clr[8]={
  0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000
};    // blank frame for clearing LEDs //

uint32_t test[8]={   // test image of 8 LEDs
  0xFFFFFF,0xFF0000,0xFF0000,0xFF00FF,0xFFFF00,0xFF0000,0xFF0000,0x00FF00
};  // test R, G, B, RG, GB, RB, RGB, 0.5*RGB //

bool LED_WRITE=0;
bool SWISH_MODE=0;
bool TIMER_TEST=1;
bool OCR_FLAG=0;
bool BUTTON_1, BUTTON_2;
bool STATUS_LED=0;
bool STATUS_M =0;

ISR(INT0_vect){                           //  External Interrupt Vector on PORTD[2] - nano pin D2, PCB pin 20
    LED_WRITE=1;
    //if(TIMER_TEST) TIMER_FLAG=1;
  }                                       // end interrupt

ISR(TIMER1_COMPA_vect){                   // for Vectron! //
    OCR_FLAG=1;
  }

void enableBrake(){     // writes brake enable pin high
  PORTD=(1<<PORTD6);
}                     // tested, works

void disableBrake(){    // writes brake enable pin low
  PORTD=(0<<PORTD6);
}                     // tested, works

void enableMotor(){     // writes motor enable pin high
  PORTD=(1<<PORTD5);
  }                     // tested, works

void disableMotor(){    // writes motor enable pin low
  PORTD=(0<<PORTD5);
  }                     // tested, works

void statusLEDon(){     // writes brake enable pin high
  PORTD=(1<<PORTD7);
}                     // tested, works

void statusLEDoff(){    // writes brake enable pin low
  PORTD=(0<<PORTD7);
}                     // tested, works

void no_op(long int CYCLES){      // delays for x cycles (instead of delay()) //
  for(long int i=0; i<CYCLES; i++){
  NOP;
    }
  }                     // tested, works.

unsigned int overflow_timer_test(){       // simple test for operation of OCR1A
      unsigned int t_now;
      unsigned long temp;
      unsigned char sreg=SREG;
      cli();
      t_now=TCNT1;
      temp=t_now+1000;                  // sets OCR1A slightly ahead of timer
      if(temp > MAX) {temp-=MAX;}
      OCR1A=(unsigned int)temp;
      SREG=sreg;
      return t_now;
  }

unsigned int timer_test(){        // simple test for timer 1
    unsigned int t_now;             // returns the current value of TCNT1
    unsigned char sreg=SREG;
    cli();
    t_now=TCNT1;
    SREG=sreg;
    return t_now;
  }

void spi_start(){               // start frame 32 * 0 bits
  for(int i=0; i<4; i++){       // send 4 lots of START
    spi_write(START);           // (start is 0)
  }
  //blinky(1, 80);
}

void spi_end(){                 // end frame 32 * 1 bits
  for(int i=0; i<4; i++){       // send 4 lots of END - try more - see note
    spi_write(END);             // end = 0
  }
  //blinky(1, 80);
}

void spi_frame(uint32_t *data, int no_leds){    // takes pointer to data structure & num leds
  spi_start();                                 // write startframe
  for(int i=0; i<no_leds; i++){             // for each set of leds
    spi_write(GLOBAL);                      // write brightness bit & 111
    spi_write((uint8_t)data[i]);                     // first 8 bits for red
    spi_write((uint8_t)(data[i] >> 8));                // next 8 bits for green
    spi_write((uint8_t)(data[i] >> 16));                // next 8 for blue
  }                                         // increment is 32 bits per LED
  spi_end();                                // send end frame
}

void spi_write(uint8_t x){                // SPI write function ( uses uint8_t as 8-byte val)
       /* Start transmission */
   SPDR = x;
   /* Wait for transmission complete */
   while(!(SPSR & (1<<SPIF)));      // wait until SPI transaction concluded
}

void initSPI(){                           // sets up SPI interface
  PRR = (0 << PRSPI);                     // turns SPI on in Power Reduction registers
  DDRB = (1<<DDB2)|(1<<DDB3)|(1<<DDB5);     // init PortB pins as output - SS, MOSI, SCK
  SPCR = (1<<SPE)|(1<<MSTR)|(0<<SPR1)|(0<<SPR0);  // spi_enable | spi_master mode | spi speed setting
  SPSR = (1<<SPI2X);                      // set double speed on spi for funsies
  PORTB &= ~(1<<PB2);                     // set SS pin low - LEDs are dumb (technical term) but still need SS low in order to write out

  //Serial.println("Init spi complete");
}

void initISR(){
  DDRD &= ~(1 << DDD2);                            // Set PortD2 as input
  EIMSK = (1 << INT0);                             // Enable int0 in ext. int. mask reg.
  EICRA = (1 << ISC01) | (1 << ISC00);             // set up external interrupt 0 on RISING edge

  DDRC = (0 << DDC0)|(0<<DDC1);                    // set portc 0,1 as inputs
  PCICR = (1<< PCIE1);                            // enable interrupts on pinchange [8:14]
  PCMSK1 = (1<< PCINT0)|(1<<PCINT1);            // enables pinchange on individual pins PORTC 0,1
  sei();
}

void initTimer(){                              // sets up timer1
  PRR = (0 << PRTIM1);                    //  turn on timer 1 in power register
  TCCR1A = (0<<WGM11)|(0<<WGM10);         //  set timer mode 'normal'
  TCCR1B = (0<<WGM12)|(0<<WGM13)|(0<<CS12)|(1<<CS11)|(1<<CS10);  // as above, plus clock prescaler of 64 - see logbook table
  TIMSK1 = (1<<OCIE1A);//|(1<<OCIE1B);      // turn on timer 1 OCR1A match compare interrupt
}                                                 // AND OCR1B

void initPins(){
  // set-up - setup SPI (pins done in function)
  //          LED (statusled)   PORTD7
  //          pin9 output (brake)     PORTD6
  //          pin10 output (enable)       PORTD5
  DDRD=(1<<DDD7)|(1<<DDD6)|(1<<DDD5); // set up pins as outputs
  PORTD=(0<<PORTD7)|(0<<PORTD6)|(0<<PORTD5);  // set all outputs low to begin with
  }

void setup(){
  Serial.begin(9600);
  initSPI();
  initISR();
  initTimer();
  initPins();
  Serial.println("All devices succesfully initiated!");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 0);
}

void loop(){

    /*for(int i=0;i<5;i++){
      spi_frame(test, 8);
      delay(1000);
      spi_frame(test_clr, 8);
      delay(1000);
    }
    blinky(10, 50);
    // code to test rotary hall interrupt //
    if(LED_WRITE){           // code to test hall sensor interrupt
        if(count % 2 == 1){ spi_frame(test, 8); }
        else              { spi_frame(test_clr, 8); }
        LED_WRITE=0;
        count+=1;
    }
    blinky(10, 50);
    // code to test OCR1A timer
    Serial.println("Swipe the magnet past the sensor twice as fast as you can!");*/


}
