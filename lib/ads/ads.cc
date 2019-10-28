#include <stdint.h>

#include "lib/ads/ads.h"

ADS::ADS(const ads_pins_t& ads_pins){
  pins_ = ads_pins;
}

bool ADS::init(){

  return true;
}

int32_t ADS::get_data(){
  return 0;
}
