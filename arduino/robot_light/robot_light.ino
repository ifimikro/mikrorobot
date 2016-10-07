#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN 7

Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, PIN, NEO_GRB + NEO_KHZ800);

int incomingByte;

void setup() {
  Serial.begin(9600);


  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
          if (Serial.available() > 0) {
                // read the incoming byte:
               incomingByte = Serial.read();
                if (incomingByte == 'w'){
                  //white
                 colorWipe(strip.Color(255, 255, 255), 50 );
                                 
                }
                if (incomingByte == 'c'){
                  //rainbowCycle
               rainbowCycle(20);
               }
               if (incomingByte == 'g'){
                //green
                colorWipe(strip.Color(0, 255, 0), 50); 
               }
               if (incomingByte == 'r'){
                //red
                colorWipe(strip.Color(255, 0, 0), 50); 
               }
               if (incomingByte == 'b'){
                //blue
                colorWipe(strip.Color(0, 0, 255), 50); 
               }
                
          }
}


void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
