#ifndef TWI_H
#define TWI_H

#include <stdbool.h>
#include <stdint.h>

void twi_init(void);
bool twi_write_packet(uint8_t address, uint8_t control,
	const uint8_t *data, uint8_t length);
bool twi_read_register_word(uint8_t address, uint8_t command, uint16_t *value);
bool twi_read_register_byte(uint8_t address, uint8_t command, uint8_t *value);
bool twi_write_register_word(uint8_t address, uint8_t command, uint16_t value);
bool twi_write_register_byte(uint8_t address, uint8_t command, uint8_t value);
bool twi_send_command(uint8_t address, uint8_t command);
bool twi_read_block(uint8_t address, uint8_t command, uint8_t *data, uint8_t max_length, uint8_t *actual_length);

#endif
