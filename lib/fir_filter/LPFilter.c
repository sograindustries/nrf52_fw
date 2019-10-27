#include <stdint.h>
#include "LPFilter.h"

static int filter_taps[LPFILTER_TAP_NUM] = {
  4,
  15,
  26,
  9,
  -70,
  -201,
  -302,
  -257,
  -40,
  200,
  232,
  -21,
  -326,
  -320,
  88,
  513,
  419,
  -243,
  -813,
  -523,
  562,
  1321,
  613,
  -1290,
  -2426,
  -675,
  4090,
  9338,
  11620,
  9338,
  4090,
  -675,
  -2426,
  -1290,
  613,
  1321,
  562,
  -523,
  -813,
  -243,
  419,
  513,
  88,
  -320,
  -326,
  -21,
  232,
  200,
  -40,
  -257,
  -302,
  -201,
  -70,
  9,
  26,
  15,
  4
};

void LPFilter_init(LPFilter* f) {
  int i;
  for(i = 0; i < LPFILTER_TAP_NUM; ++i)
    f->history[i] = 0;
  f->last_index = 0;
}

void LPFilter_put(LPFilter* f, int input) {
  f->history[f->last_index++] = input;
  if(f->last_index == LPFILTER_TAP_NUM)
    f->last_index = 0;
}

int LPFilter_get(LPFilter* f) {
  int64_t acc = 0;
  int index = f->last_index, i;
  for(i = 0; i < LPFILTER_TAP_NUM; ++i) {
    index = index != 0 ? index-1 : LPFILTER_TAP_NUM-1;
    acc += (int64_t)f->history[index] * filter_taps[i];
  };
  return acc >> 16;
}
