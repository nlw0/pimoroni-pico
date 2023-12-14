/*
  Pico Unicorn Plasma Example
  Written by Tim Kerby https://github.com/tkerby

  Hardware: Raspberry Pi Pico with Pimoroni Unicorn Pico Display
  Printf over USB 9600 8N1 (see makefile)

  Based on Adafruit Arduino Example for 32x32 panel
  Originally written by Limor Fried/Ladyada & Phil Burgess/PaintYourDragon for Adafruit Industries.
  https://github.com/adafruit/RGB-matrix-Panel/blob/master/examples/plasma_32x32/plasma_32x32.ino

  BSD License
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include <time.h>

#include "pico_unicorn.hpp"

using namespace pimoroni;

// PicoUnicorn<uint32_t(12), uint32_t(6), uint16_t(4)> pico_unicorn;
// PicoUnicorn<12, 1, 4> pico_unicorn;
// PicoUnicorn<12, 6, 4> pico_unicorn;
// PicoUnicorn<14, 1, 0> pico_unicorn;
PicoUnicorn<12,6,4, uint16_t, pimoroni::GAMMA_12BIT> pico_unicorn;

template<typename T>
T cb(T x){return x*x*x;}

int main() {
  stdio_init_all();
  // pico_unicorn.init();
  pico_unicorn.clear();

  sleep_ms(1e3);
  printf("\nHi\n");

  // const float Tperiod=8;
  // const float Tperiod=100;
  const float Tperiod=10;
  // const float Dperiod=24*60*60;
  // const float Dperiod=12*60*60;
  const float Dperiod=120;
  // const float Tperiod=1;
  // const float Tperiod=2;
  // const float Tperiod=4;

  const int period=16;
  // const int period=8;

  uint ms = to_us_since_boot(get_absolute_time());
  float looptime = Tperiod/period;
  uint mycount=0;
  float periodfilter=0.0;
  float alpha=0.1;

  for(;;) {
    unsigned char x, y;

    uint mytime = to_us_since_boot(get_absolute_time());

    // state = (state + 1) % period;
    // state = int(std::round(mytime*1e-6 / Tperiod)) % period;
    float state = std::fmod(mytime*1e-6 / Tperiod*period, period);
    float state15 = std::fmod(mytime*1e-6 / 15*period, period);
    float Dstate = std::fmod(mytime*1e-6 / Dperiod*period, period);

    uint16_t xx = std::min(state,period-state); // triangle wave from 0 to period/2
    
    periodfilter += alpha*(mytime-ms-periodfilter);
    if (mycount==100) {
      printf("t=%f xx=%d mytime=%d looptime=%f time=%f\n", state, xx, mytime, looptime, 1e-6*periodfilter);
      mycount=0;
    }
    mycount++;
    ms=mytime;
    
    for(y=0; y<7; y++) {
      for(x=0; x<16; x++) {
	// uint16_t ar = (y+state+x)%16; // triangle wave from 0 to period/2
	// uint16_t br = std::min(ar,uint16_t(16-ar)); // triangle wave from 0 to period/2-1
        // pico_unicorn.set_pixel(x, y, std::round(1+254*float(br)/float(period-1)), uint8_t(0), uint8_t(0));

	float ar = std::fmod(y+state+x,16); // triangle wave from 0 to period/2
	float br = std::min(ar,16-ar); // triangle wave from 0 to period/2-1

	float ag = std::fmod(7-y+Dstate+x,16); // triangle wave from 0 to period/2
	float bg = std::min(ag,16-ag); // triangle wave from 0 to period/2-1
	
        // pico_unicorn.set_pixel(x, y, std::round(2+253*br/float(period-1)), uint8_t(std::round(10*bg/float(period-1))), uint8_t(0));
	float gwave = std::max(std::min(state15-2, 16-state15-2),0.0f)/14.0f;
	// float myr = 2+253*br/float(period-1);
	// float myg = 150*std::max(std::min(Dstate-2, 16-Dstate-2),0.0f)/14*bg/float(period-1);
	// float myg2 = std::min(myr/2, myg);
        // pico_unicorn.set_pixel(x, y, myr, myg2, uint8_t(0));
	float myr = br/float(period-1);
	float myr2 = 2+253*myr;
	float myg = 150*gwave*bg/float(period-1);
	float myg2 = (myr*0.9+0.1) * myg;
        pico_unicorn.set_pixel(x, y, myr2, myg2, 0);
        // pico_unicorn.set_pixel(x, y, 0, myg2, 0);
      }
    }
    // sleep_ms(1000*looptime);
    // sleep_ms(1);
  }

  return 0;
}
