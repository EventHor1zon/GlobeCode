/*
*   Test code v1.1
*   This is the test code used to test the POV globe mechanics
*   code, and functionality.
*
*
*
*
*/
#include <avr/io.h>

#define START   0x00    // start frame 32*0 bits
#define END     0xFF    // end frame 32 * 1 bits
#define GLOBAL  0xFF    //
#define NUMLEDS 8


uint32_t test_clr[8]={
  0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000,0x000000
};    // blank frame for clearing LEDs //

uint32_t test[8]={   // test image of 8 LEDs
  0xFF0000,0x00FF00,0x0000FF,0xFFFF00,0x00FFFF,0xFF00FF,0xFFFFFF,0x999999
};  // test R, G, B, RG, GB, RB, RGB, 0.5*RGB //

void blinky(int x, int t){
  for(int i=0; i<x; i++){
    digitalWrite(LED_BUILTIN, 1);
    delay(t);
    digitalWrite(LED_BUILTIN, 0);
    delay(t);
  }
}

void spi_start(){               // start frame 32 * 0 bits
  for(int i=0; i<4; i++){       // send 4 lots of START
    spi_write(START);           // (start is 0)
  }
  blinky(1, 80);
}

void spi_end(){                 // end frame 32 * 1 bits
  for(int i=0; i<4; i++){       // send 4 lots of END - try more - see note
    spi_write(END);             // end = 0
  }
  blinky(1, 80);
}

void spi_frame(uint32_t *data, int no_leds){    // takes pointer to data structure & num leds
  spi_start();                                 // write startframe
  Serial.println("Sent start");
  for(int i=0; i<no_leds; i++){             // for each set of leds
    Serial.print("Data: ");
    Serial.print(data[i], HEX);
    spi_write(GLOBAL);                      // write brightness bit & 111
    spi_write(data[i]);        // 8-bit int typecast data+(increment); rem data here is a pointer!
    spi_write(data[i] >> 8);
    spi_write(data[i] >> 16);
    Serial.print("  R: ");
    Serial.print(r, HEX);
    Serial.print("  G: ");
    Serial.print(g, HEX);
    Serial.print("  B: ");
    Serial.println(b, HEX);
  }                                         // increment is 32 bits per LED
  Serial.println("Sent data");
  spi_end();                                // send end frame
  Serial.println("Sent end");
}

void spi_write(uint8_t x){                // SPI write function ( uses uint8_t as 8-byte val)
       /* Start transmission */
   SPDR = x;
   /* Wait for transmission complete */
   while(!(SPSR & (1<<SPIF)))
   ;      // wait until SPI transaction concluded

}

void initSPI(){                           // sets up SPI interface
  PRR = (0 << PRSPI);                     // turns SPI on in Power Reduction registers
  DDRB = (1<<DDB2)|(1<<DDB3)|(1<<DDB5);     // init PortB pins as output - SS, MOSI, SCK
  SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
  PORTB &= ~(1<<PB2);                     // set SS pin low - LEDs are dumb (technical term) but still need SS low in order to write out
  Serial.println("Init spi complete");
}

void setup(){
  Serial.begin(9600);
  blinky(10,200);
  initSPI();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 0);
}

void loop(){
  blinky(10, 50);
  while(1){
    spi_frame(test, 8);
    delay(10000);
    spi_frame(test_clr, 8);
    delay(10000);
    //while(1){
    //  blinky(5, 700);
    //}
}
}
