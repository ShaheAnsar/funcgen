#ifndef AD9833_H
#define AD9833_H

#define AD9833_B28 13
#define AD9833_HLB 12
#define AD9833_FSELECT 11
#define AD9833_PSELECT 10
#define AD9833_RESET 8
#define AD9833_SLEEP1 7
#define AD9833_SLEEP12 6
#define AD9833_OPBITEN 5
#define AD9833_DIV2 3
#define AD9833_MODE 1

#define AD9833_MCLK 25000000UL
#define AD9833_FREQ_COEFF_X100 1074

#define AD9833_MODE_SINE 0
#define AD9833_MODE_TRI 1
#define AD9833_MODE_SQUARE 2


extern void ad9833_reset(volatile uint16_t* spi_out, volatile uint16_t* spi_sr);
extern void ad9833_set_frequency(volatile uint16_t* spi_dr, volatile uint16_t* spi_sr,
				 uint8_t freq_reg, uint32_t freq);
extern void ad9833_change_mode(volatile uint16_t* spi_dr, volatile uint16_t* spi_sr,
			       uint8_t mode);
#endif
