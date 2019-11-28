
#ifndef USB_SERIAL_h
#define USB_SERIAL_h

#include "usbd_cdc_if.h"
#include "string.h"

#define SERIAL_TX_BUFFER_SIZE   128
#define SERIAL_RX_BUFFER_SIZE   128

/*
*  This is a USB CDC Virtual Com Port wrapper
*  Author: maisonsmd
*  Email: maisondmd@gmail.com
*/

class UsbSerial {
private:
public:
  uint16_t _availableBytes = 0;
  uint8_t * _buffer;
  void begin();
  // return number of bytes last received
  uint16_t available();
  // copy received buffer to <buffer>
  void readBytes(uint8_t * buffer, uint16_t length);
  void writeBytes(uint8_t * buffer, uint16_t length);
  void print(const char * message);
  void println(const char * message);
};

extern UsbSerial Serial;

#endif

