#include "sbox.h"
#include <stdint.h>
#include <stdio.h>

uint8_t mult_aes_256(uint8_t, uint8_t);
void build_tbox(uint32_t*);
void build_i_tbox(uint32_t*);
void prettyprint(const uint8_t*, const int, char*);

int main(int argc, char *argv[])
{
  uint32_t tbox[256];
  uint32_t i_tbox[256];
  char t_buffer[9];
  char s_buffer[3];
  char i_s_buffer[3];
  char i_t_buffer[9];
  int i;

  build_tbox(tbox);
  build_i_tbox(i_tbox);

  // print the tbox
  for(i = 0; i < 256; i++)
  {
    prettyprint((uint8_t*)&tbox[i], 4, t_buffer);
    prettyprint(&sbox[i], 1, s_buffer);

    prettyprint((uint8_t*)&i_tbox[i], 4, i_t_buffer);
    prettyprint(&i_sbox[i], 1, i_s_buffer);
    printf("assign sbox_tbox[%i] = 80'h%s%s%s%s;\n", i, s_buffer, t_buffer, i_s_buffer, i_t_buffer);
  }
}

// AES polynomial is m(x) = x^8 + x^4 + x^3 + x + 1
// this is: 00011011 = 0x1B
uint8_t mult_aes_256(uint8_t A, uint8_t B)
{
  uint8_t Ax[8];
  uint8_t result, mask;
  int i;

  Ax[0] = A;

  // I could have merged these two loops, but this code is not meant to perform
  // since it is only used to generate tboxes
  for(i = 1; i < 7; i++)
  {
    Ax[i] = Ax[i-1] << 1; // multiply by x
    if(Ax[i-1] >> 7) // if necessary, reduce
      Ax[i] ^= 0x1B;
  }

  for(i = 0, result = 0x00, mask = 0x01; i < 8; i++, mask = mask << 1)
    if(mask & B)
      result ^= Ax[i];

  return result;
}

void build_tbox(uint32_t *tbox)
{
  uint8_t byte, byte02, byte03;
  int i;

  // clear tboxes...
  for(i = 0; i < 256; i++)
    tbox[i] = 0x0;

  // fill t-tables
  for(i = 0; i < 256; i++)
  {
    byte = sbox[i];
    byte02 = mult_aes_256(byte, 0x02);
    byte03 = mult_aes_256(byte, 0x03);

    //tbox[i] = (byte02 << 24) | (byte << 16) | (byte << 8) | byte03;
    tbox[i] = (byte03 << 24) | (byte << 16) | (byte << 8) | byte02;
  }
}

void build_i_tbox(uint32_t *i_tbox)
{
  uint8_t byte;
  uint8_t byte0e, byte0b, byte0d, byte09;
  int i;

  // clear tboxes...
  for(i = 0; i < 256; i++)
    i_tbox[i] = 0x0;

  // fill t-tables
  for(i = 0; i < 256; i++)
  {
    byte = i_sbox[i];

    byte0e = mult_aes_256(byte, 0x0e);
    byte0b = mult_aes_256(byte, 0x0b);
    byte0d = mult_aes_256(byte, 0x0d);
    byte09 = mult_aes_256(byte, 0x09);

    i_tbox[i] = (byte0b << 24) | (byte0d << 16) | (byte09 << 8) | byte0e;
  }
}

void prettyprint(const uint8_t *bytes, const int numbytes, char *buffer)
{
  static const char hex[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
  int i;

  /*for(i = 0; i < 8; i++)
    buffer[i] = hex[(word << (i*4)) >> 28];*/
  for(i = numbytes-1; i >= 0; i--)
  {
    buffer[2*i] = hex[bytes[i] >> 4];
    buffer[2*i+1] = hex[bytes[i] & 15];
  }

  buffer[2*numbytes] = '\0';
}
