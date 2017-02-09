/*                                                                      tab:2
 *
 * Copyright (c) 2000-2007 The Regents of the University of
 * California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holders nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * Implementation of the <code>PacketParrot</code> application.
 *
 * @author Prabal Dutta
 * @date   Apr 6, 2007
 * @author Janos Sallai
 * @date   Jan 25, 2012
 */

// #include "printf.h"   //01 added by Ezio 24/08/2016 to enable printf function
module PacketParrotP {
  uses {
    interface Boot;
    interface Leds;
    interface Packet;
    interface AMPacket;
    interface AMSend[uint8_t id];
    interface Receive[uint8_t id];
    interface Receive as Snoop[uint8_t id];
    interface SplitControl as AMControl;
    interface LogRead;
    interface LogWrite;
    interface Timer<TMilli> as Timer0;
  }
}
implementation {

  enum {
    INTER_PACKET_INTERVAL = 25
  };

  typedef nx_struct logentry_t {
    nx_uint8_t len;
    message_t msg;
  } logentry_t;

  bool m_busy = TRUE;
  logentry_t m_entry;

  event void Boot.booted() {
    call AMControl.start();
    printf("Boot.booted and AMControl.satrt m_entry.len =\n");     //01 added by Ezio 24/08/2016 to enable printf function
    printf("Boot.booted and AMControl.satrt m_entry.msg =\n");     //01 added by Ezio 24/08/2016 to enable printf function
    printf("Boot.booted and AMControl.satrt\n");     //01 added by Ezio 24/08/2016 to enable printf function
    printfflush();
  }


  event void AMControl.startDone(error_t err) {
    if (err==SUCCESS)
    {
    printf("AMControl.startDone err SUCCESS =%d\n",err);     //01 added by Ezio 24/08/2016 to enable printf function
    printfflush();
  }
    else
    {
    printf("AMControl.startDone err FALSE =%d\n",err);     //01 added by Ezio 24/08/2016 to enable printf function
    printfflush();
    }
    if (err == SUCCESS) {
      printf("call LogRead.read\n");     //01 added by Ezio 24/08/2016 to enable printf function
      printfflush();
      if (call LogRead.read(&m_entry, sizeof(logentry_t)) != SUCCESS) {
	// Handle error.
      printf("call LogRead.read FAIL\n");     //01 added by Ezio 24/08/2016 to enable printf function
      printfflush();
      }
    }
    else {
      printf("AMControl.start FAIL\n");     //01 added by Ezio 24/08/2016 to enable printf function
      printfflush();
      call AMControl.start();
    }
  }


  event void AMControl.stopDone(error_t err) {
    printf("AMControl.stopDone\n");     //01 added by Ezio 24/08/2016 to enable printf function
    printfflush();
  }


  event void LogRead.readDone(void* buf, storage_len_t len, error_t err) {
    printf("event LogRead.readDone buf =%d\n",&buf);     //01 added by Ezio 24/08/2016 to enable printf function
    printf("event LogRead.readDone len =%d\n",len);     //01 added by Ezio 24/08/2016 to enable printf function
    printfflush();
    if (err==SUCCESS)
    {
    printf("AMControl.startDone err SUCCESS =%d\n",err);     //01 added by Ezio 24/08/2016 to enable printf function
    printfflush();
  }
    else
    {
    printf("AMControl.startDone err FALSE =%d\n",err);     //01 added by Ezio 24/08/2016 to enable printf function
    printfflush();
    }
    if ( (len == sizeof(logentry_t)) && (buf == &m_entry) ) {
      call AMSend.send[call AMPacket.type(&m_entry.msg)](call AMPacket.destination(&m_entry.msg), &m_entry.msg, m_entry.len);
      call Leds.led1On();
      printf("AMSend.send\n");     //01 added by Ezio 24/08/2016 to enable printf function
      printfflush();
    }
    else {
      printf("call LogWrite.erase\n");     //01 added by Ezio 24/08/2016 to enable printf function
      printfflush();
      if (call LogWrite.erase() != SUCCESS) {
      printf("call LogWrite.erase FAIL\n");     //01 added by Ezio 24/08/2016 to enable printf function
      printfflush();
	// Handle error.
      }
      call Leds.led0On();
    }
  }


  event void AMSend.sendDone[uint8_t id](message_t* msg, error_t err) {
    printf("event AMSend.sendDone\n");     //01 added by Ezio 24/08/2016 to enable printf function
    printfflush();
    call Leds.led1Off();
    if ( (err == SUCCESS) && (msg == &m_entry.msg) ) {
      printf("call Packet.clear\n");     //01 added by Ezio 24/08/2016 to enable printf function
      printfflush();
      call Packet.clear(&m_entry.msg);
      printf("call LogRead.read\n");     //01 added by Ezio 24/08/2016 to enable printf function
      printfflush();
      if (call LogRead.read(&m_entry, sizeof(logentry_t)) != SUCCESS) {
	// Handle error.
      }
    }
    else {
      printf("call Timer0.startOneShot\n");     //01 added by Ezio 24/08/2016 to enable printf function
      printfflush();
      call Timer0.startOneShot(INTER_PACKET_INTERVAL);
    }
  }


  event void Timer0.fired() {
    printf("event void Timer0.fired\n");     //01 added by Ezio 24/08/2016 to enable printf function
    printf("call AMSend.send\n");     //01 added by Ezio 24/08/2016 to enable printf function
    printfflush();
    call AMSend.send[call AMPacket.type(&m_entry.msg)](call AMPacket.destination(&m_entry.msg), &m_entry.msg, m_entry.len);
  }


  event void LogWrite.eraseDone(error_t err) {
    if (err==SUCCESS)
    {
    printf("AMControl.startDone err SUCCESS =%d\n",err);     //01 added by Ezio 24/08/2016 to enable printf function
    printfflush();
  }
    else
    {
    printf("AMControl.startDone err FALSE =%d\n",err);     //01 added by Ezio 24/08/2016 to enable printf function
    printfflush();
    }
    printf("call AMSend.send\n");     //01 added by Ezio 24/08/2016 to enable printf function
    printfflush();
    if (err == SUCCESS) {
      printf("call AMSend.send m_busy=%d\n",m_busy);     //01 added by Ezio 24/08/2016 to enable printf function
    printfflush();
      m_busy = FALSE;
    printf("call AMSend.send m_busy=%d\n",m_busy);     //01 added by Ezio 24/08/2016 to enable printf function
    printfflush();
    }
    else {
      // Handle error.
    }
    call Leds.led0Off();
  }

  event message_t* Receive.receive[uint8_t id](message_t* msg, void* payload, uint8_t len) {
   call Leds.led2On();
   printf("event message_t* Receive.receive\n");     //01 added by Ezio 24/08/2016 to enable printf function
    printfflush();
    if (!m_busy) {
      m_busy = TRUE;
      m_entry.len = len;
      m_entry.msg = *msg;
      printf("call LogWrite.append\n");     //01 added by Ezio 24/08/2016 to enable printf function
      printfflush();
      if (call LogWrite.append(&m_entry, sizeof(logentry_t)) != SUCCESS) {
	m_busy = FALSE;
      }
    }
    return msg;
  }

  event message_t* Snoop.receive[uint8_t id](message_t* msg, void* payload, uint8_t len) {
    printf("event message_t* Snoop.receive\n");     //01 added by Ezio 24/08/2016 to enable printf function
    printfflush();
    return signal Receive.receive[id](msg, payload, len);
  }
  
  event void LogWrite.appendDone(void* buf, storage_len_t len, 
                                 bool recordsLost, error_t err) {
    printf("event void LogWrite.appendDone\n");     //01 added by Ezio 24/08/2016 to enable printf function
    printfflush();
    m_busy = FALSE;
    call Leds.led2Off();
  }

  event void LogRead.seekDone(error_t err) {
    printf("event void LogRead.seekDone\n");     //01 added by Ezio 24/08/2016 to enable printf function
    printfflush();
  }

  event void LogWrite.syncDone(error_t err) {
    printf("event void LogWrite.syncDone\n");     //01 added by Ezio 24/08/2016 to enable printf function
    printfflush();
  }

}
