#ifndef LPFILTER_H_
#define LPFILTER_H_

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 250 Hz

fixed point precision: 16 bits

* 0 Hz - 40 Hz
  gain = 1
  desired ripple = 1 dB
  actual ripple = n/a

* 55 Hz - 125 Hz
  gain = 0
  desired attenuation = -100 dB
  actual attenuation = n/a

*/

#define LPFILTER_TAP_NUM 57

typedef struct {
  int history[LPFILTER_TAP_NUM];
  unsigned int last_index;
} LPFilter;

void LPFilter_init(LPFilter* f);
void LPFilter_put(LPFilter* f, int input);
int LPFilter_get(LPFilter* f);

#endif