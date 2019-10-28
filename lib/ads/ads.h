#ifndef _ARGOS_LIB_ADS_H_
#define _ARGOS_LIB_ADS_H_

#include <stdint.h>

typedef struct {
  int miso;
  int mosi;
  int csn;
  int rst;
  int drdy;
  int start;
  int clksel;
} ads_pins_t;

class ADS {

 public:
  ADS(const ads_pins_t& ads_pins);

  bool init();

  int32_t get_data();

 private:
  ads_pins_t pins_;
  
};

#endif
