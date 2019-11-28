#include "UsbSerial.h"

void UsbSerial::begin(){
  Serial._buffer = UserRxBufferFS;
}
// return number of bytes last received
uint16_t UsbSerial::available(){
  return _availableBytes;
}
// copy received buffer to <buffer>
void UsbSerial::readBytes(uint8_t * buffer, uint16_t length){
  memset(buffer, 0, length);

  if(length > _availableBytes) length = _availableBytes;

  memcpy(buffer, _buffer, length);
  _availableBytes = 0;
}

void UsbSerial::writeBytes(uint8_t * buffer, uint16_t length){
  memcpy(UserTxBufferFS, buffer, length);
  CDC_Transmit_FS(UserTxBufferFS, length);
}

void UsbSerial::print(const char * message){
  uint16_t len = strlen(message);
  memcpy(UserTxBufferFS, message, len);
  CDC_Transmit_FS(UserTxBufferFS, len);
}

void UsbSerial::println(const char * message){
  uint16_t len = strlen(message);
  strncpy((char*)UserTxBufferFS, (char*)message, len + 1);
  strcat((char*)UserTxBufferFS, "\r\n");
  CDC_Transmit_FS(UserTxBufferFS, len + 2);
}

UsbSerial Serial;

