#include <stddef.h>
#include <stdint.h>

// from https://lxp32.github.io/docs/a-simple-example-crc32-calculation/
uint32_t crc32(const uint8_t* buff,size_t n) {
  uint32_t crc=0xFFFFFFFF;
  
  for(size_t i=0;i<n;i++) {
    uint8_t ch = buff[i];
    for(size_t j=0;j<8;j++) {
      uint32_t b=(ch^crc)&1;
      crc>>=1;
      if(b) crc=crc^0xEDB88320;
      ch>>=1;
    }
  }
  
  return ~crc;
}
