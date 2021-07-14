#include <stdint.h>
#include "ad9833.h"

void ad9833_reset(volatile uint16_t *spi_out, volatile uint16_t *spi_sr) {
  // Reset the internal registers
  *spi_out = (1 << AD9833_B28) | (1 << AD9833_RESET);
  // Wait till the data has been output
  while (!((*spi_sr) & (1 << 1)));
  // Set to  1MHz for both frequency registers
  uint32_t OneMHzMagicNum = 10737418;
  *spi_out = (1 << 15) | (uint16_t)(OneMHzMagicNum & 0x3fff );
  while (!((*spi_sr) & (1 << 1)));
  *spi_out = (1 << 15) | (uint16_t)(( OneMHzMagicNum >> 14 ) & 0x3fff);
  while (!((*spi_sr) & (1 << 1)));
  *spi_out = (1 << 14) | (uint16_t)(OneMHzMagicNum & 0x3fff );
  while (!((*spi_sr) & (1 << 1)));
  *spi_out = (1 << 14) | (uint16_t)(( OneMHzMagicNum >> 14 ) & 0x3fff);
  while (!((*spi_sr) & (1 << 1)));
  // Set to 0 for both phase registers
  *spi_out = (0b11 << 14);
  while (!((*spi_sr) & (1 << 1)));
  *spi_out = (0b11 << 14) | (1 << 13);
  while (!((*spi_sr) & (1 << 1)));
  // Unreset the AD9833
  *spi_out = (1 << AD9833_B28);
  while (!((*spi_sr) & (1 << 1)));
}

void ad9833_set_frequency(volatile uint16_t* spi_dr, volatile uint16_t* spi_sr,
			  uint8_t freq_reg, uint32_t freq) {
  //double freq_data = (268435456)/AD9833_MCLK * freq;
  //uint32_t freq_data_rounded = (uint32_t)( (int32_t)(freq_data) );
  uint32_t freq_data_rounded = AD9833_FREQ_COEFF_X100 * freq;
  freq_data_rounded /= 100;
  uint16_t freq_reg_data = (((freq_reg ==0)?(0b01):(0b10)) << 14);
  *spi_dr =  freq_reg_data | (uint16_t)(freq_data_rounded & 0x3fff);
  while (!((*spi_sr) & (1 << 1)));
  *spi_dr =  freq_reg_data | (uint16_t)(( freq_data_rounded >> 14) & 0x3fff);
  while (!((*spi_sr) & (1 << 1)));
}


void ad9833_change_mode(volatile uint16_t* spi_dr, volatile uint16_t* spi_sr, uint8_t mode) {
  switch(mode) {
  case AD9833_MODE_SINE:
    *spi_dr = (1 << AD9833_B28);
    while (!((*spi_sr) & (1 << 1)));
    break;
  case AD9833_MODE_SQUARE:
    *spi_dr = (1 << AD9833_B28) | (1 << AD9833_OPBITEN) | (1 << AD9833_DIV2);
    while (!((*spi_sr) & (1 << 1)));
    break;
  case AD9833_MODE_TRI:
    *spi_dr = (1 << AD9833_B28) | (1 << AD9833_MODE);
    while (!((*spi_sr) & (1 << 1)));
    break;
  }
}
