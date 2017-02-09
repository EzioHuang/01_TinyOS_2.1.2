// $Id: BlinkToRadio.h,v 1.4 2006-12-12 18:22:52 vlahan Exp $

#ifndef BLINKTORADIO_H
#define BLINKTORADIO_H

enum {
  AM_BLINKTORADIOMSG = 6,   //BLINKTORADIOMSG is consistent with struct name and the configuration file.
  //AM_BLINKTORADIOMSG = 0x89,
  //default TIMER_PERIOD_MILLI = 250
  TIMER_PERIOD_MILLI = 5000

};

typedef nx_struct BlinkToRadioMsg {
  nx_uint16_t nodeid;
  nx_uint16_t counter;
} BlinkToRadioMsg;

#endif
