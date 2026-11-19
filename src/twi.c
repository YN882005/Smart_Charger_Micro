#include "twi.h"

#include <avr/io.h>

#include "config.h"

#define TWI_TIMEOUT_LOOPS 60000u

enum {
	TWI_STATUS_START = 0x08u,
	TWI_STATUS_REPEATED_START = 0x10u,
	TWI_STATUS_SLA_W_ACK = 0x18u,
	TWI_STATUS_DATA_W_ACK = 0x28u,
	TWI_STATUS_SLA_R_ACK = 0x40u,
	TWI_STATUS_DATA_R_ACK = 0x50u,
	TWI_STATUS_DATA_R_NACK = 0x58u
};

static bool twi_wait_complete(void)
{
	uint16_t timeout = TWI_TIMEOUT_LOOPS;

	while ((TWCR & _BV(TWINT)) == 0u && timeout > 0u) {
		--timeout;
	}

	return (TWCR & _BV(TWINT)) != 0u;
}

static uint8_t twi_status(void)
{
	return (uint8_t)(TWSR & 0xF8u);
}

static bool twi_start(void)
{
	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
	return twi_wait_complete()
		&& (twi_status() == TWI_STATUS_START
			|| twi_status() == TWI_STATUS_REPEATED_START);
}

static bool twi_write(uint8_t value, uint8_t expected_status)
{
	TWDR = value;
	TWCR = _BV(TWINT) | _BV(TWEN);
	return twi_wait_complete() && twi_status() == expected_status;
}

static bool twi_read(uint8_t *value, bool acknowledge, uint8_t expected_status)
{
	TWCR = _BV(TWINT) | _BV(TWEN) | (acknowledge ? _BV(TWEA) : 0u);
	if (!twi_wait_complete() || twi_status() != expected_status) {
		return false;
	}

	*value = TWDR;
	return true;
}

static bool twi_stop(void)
{
	uint16_t timeout = TWI_TIMEOUT_LOOPS;

	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
	while ((TWCR & _BV(TWSTO)) != 0u && timeout > 0u) {
		--timeout;
	}

	return (TWCR & _BV(TWSTO)) == 0u;
}

bool twi_write_packet(uint8_t address, uint8_t control,
	const uint8_t *data, uint8_t length)
{
	bool ok = false;
	uint8_t index;

	if (address > 0x7Fu || (length > 0u && data == 0)) {
		return false;
	}

	if (!twi_start()
		|| !twi_write((uint8_t)(address << 1), TWI_STATUS_SLA_W_ACK)
		|| !twi_write(control, TWI_STATUS_DATA_W_ACK)) {
		goto cleanup;
	}

	for (index = 0u; index < length; ++index) {
		if (!twi_write(data[index], TWI_STATUS_DATA_W_ACK)) {
			goto cleanup;
		}
	}

	ok = true;

cleanup:
	if (!twi_stop()) {
		ok = false;
	}
	return ok;
}

void twi_init(void)
{
	TWSR = 0;
	TWBR = (uint8_t)(((F_CPU / TWI_CLOCK_HZ) - 16UL) / 2UL);
	TWCR = _BV(TWEN);
}

bool twi_read_register_word(uint8_t address, uint8_t command, uint16_t *value)
{
	uint8_t low;
	uint8_t high;
	bool ok = false;

	if (value == 0 || address > 0x7Fu) {
		return false;
	}

	if (!twi_start()
		|| !twi_write((uint8_t)(address << 1), TWI_STATUS_SLA_W_ACK)
		|| !twi_write(command, TWI_STATUS_DATA_W_ACK)
		|| !twi_start()
		|| !twi_write((uint8_t)((address << 1) | 1u), TWI_STATUS_SLA_R_ACK)
		|| !twi_read(&low, true, TWI_STATUS_DATA_R_ACK)
		|| !twi_read(&high, false, TWI_STATUS_DATA_R_NACK)) {
		goto cleanup;
	}

	*value = (uint16_t)low | ((uint16_t)high << 8);
	ok = true;

cleanup:
	if (!twi_stop()) {
		ok = false;
	}
	return ok;
}

bool twi_read_register_byte(uint8_t address, uint8_t command, uint8_t *value)
{
	bool ok = false;

	if (value == 0 || address > 0x7Fu) {
		return false;
	}

	if (!twi_start()
		|| !twi_write((uint8_t)(address << 1), TWI_STATUS_SLA_W_ACK)
		|| !twi_write(command, TWI_STATUS_DATA_W_ACK)
		|| !twi_start()
		|| !twi_write((uint8_t)((address << 1) | 1u), TWI_STATUS_SLA_R_ACK)
		|| !twi_read(value, false, TWI_STATUS_DATA_R_NACK)) {
		goto cleanup;
	}

	ok = true;

cleanup:
	if (!twi_stop()) {
		ok = false;
	}
	return ok;
}

bool twi_write_register_word(uint8_t address, uint8_t command, uint16_t value)
{
	bool ok = false;

	if (address > 0x7Fu) {
		return false;
	}

	if (!twi_start()
		|| !twi_write((uint8_t)(address << 1), TWI_STATUS_SLA_W_ACK)
		|| !twi_write(command, TWI_STATUS_DATA_W_ACK)
		|| !twi_write((uint8_t)(value & 0xFFu), TWI_STATUS_DATA_W_ACK)
		|| !twi_write((uint8_t)((value >> 8) & 0xFFu), TWI_STATUS_DATA_W_ACK)) {
		goto cleanup;
	}

	ok = true;

cleanup:
	if (!twi_stop()) {
		ok = false;
	}
	return ok;
}

bool twi_write_register_byte(uint8_t address, uint8_t command, uint8_t value)
{
	bool ok = false;

	if (address > 0x7Fu) {
		return false;
	}

	if (!twi_start()
		|| !twi_write((uint8_t)(address << 1), TWI_STATUS_SLA_W_ACK)
		|| !twi_write(command, TWI_STATUS_DATA_W_ACK)
		|| !twi_write(value, TWI_STATUS_DATA_W_ACK)) {
		goto cleanup;
	}

	ok = true;

cleanup:
	if (!twi_stop()) {
		ok = false;
	}
	return ok;
}

bool twi_send_command(uint8_t address, uint8_t command)
{
	bool ok = false;

	if (address > 0x7Fu) {
		return false;
	}

	if (!twi_start()
		|| !twi_write((uint8_t)(address << 1), TWI_STATUS_SLA_W_ACK)
		|| !twi_write(command, TWI_STATUS_DATA_W_ACK)) {
		goto cleanup;
	}

	ok = true;

cleanup:
	if (!twi_stop()) {
		ok = false;
	}
	return ok;
}

bool twi_read_block(uint8_t address, uint8_t command, uint8_t *data, uint8_t max_length, uint8_t *actual_length)
{
	uint8_t block_size = 0u;
	uint8_t i;
	bool ok = false;

	if (data == 0 || address > 0x7Fu || max_length == 0u) {
		return false;
	}

	if (!twi_start()
		|| !twi_write((uint8_t)(address << 1), TWI_STATUS_SLA_W_ACK)
		|| !twi_write(command, TWI_STATUS_DATA_W_ACK)
		|| !twi_start()
		|| !twi_write((uint8_t)((address << 1) | 1u), TWI_STATUS_SLA_R_ACK)) {
		goto cleanup;
	}

	if (!twi_read(&block_size, true, TWI_STATUS_DATA_R_ACK)) {
		goto cleanup;
	}

	if (actual_length != 0) {
		*actual_length = block_size;
	}

	for (i = 0u; i < block_size; ++i) {
		bool ack = (i + 1u < block_size);
		uint8_t b = 0u;
		if (!twi_read(&b, ack, ack ? TWI_STATUS_DATA_R_ACK : TWI_STATUS_DATA_R_NACK)) {
			goto cleanup;
		}
		if (i < max_length) {
			data[i] = b;
		}
	}

	ok = true;

cleanup:
	if (!twi_stop()) {
		ok = false;
	}
	return ok;
}
