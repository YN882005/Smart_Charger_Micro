#include "ina233.h"

#include "twi.h"

bool ina233_device_init(ina233_device_t *device, uint8_t address,
	float r_shunt, float i_max)
{
	(void)r_shunt;
	(void)i_max;

	if (device == 0) {
		return false;
	}
	device->initialized = false;

	if (address > 0x7Fu) {
		return false;
	}

	/* Spec: Shunt = 0.0333 Ohm, Max current = 1.5 A -> Current_LSB = 50 uA/bit (0.00005 A/bit)
	 * MFR_CALIBRATION (0xD4) = 0x0C03 (trunc(0.00512 / (0.00005 * 0.0333)) = 3075) */
	if (!twi_write_register_word(address, INA233_REG_CALIBRATION, 0x0C03u)) {
		return false;
	}

	device->address = address;
	device->current_lsb = 0.00005f;
	device->initialized = true;
	return true;
}

static bool ina233_read_register(const ina233_device_t *device, uint8_t command,
	uint16_t *value)
{
	if (device == 0 || !device->initialized || value == 0) {
		return false;
	}

	return twi_read_register_word(device->address, command, value);
}

bool ina233_read_sample(const ina233_device_t *device,
	ina233_raw_sample_t *sample)
{
	ina233_raw_sample_t next = {0};

	if (device == 0 || !device->initialized || sample == 0) {
		return false;
	}

	/* READ_VBUS (0x88) and READ_IIN (0x89) */
	if (!ina233_read_register(device, INA233_REG_BUS_VOLTAGE, &next.bus_voltage_raw)
		|| !ina233_read_register(device, INA233_REG_CURRENT, &next.current_raw)) {
		return false;
	}

	/* READ_VBUS: 1.25 mV/bit */
	next.bus_voltage_v = (float)next.bus_voltage_raw * 0.00125f;

	/* READ_IIN: signed 16-bit * 0.05 mA/bit */
	next.current_ma = (float)((int16_t)next.current_raw) * 0.05f;

	*sample = next;
	return true;
}

bool ina233_read_ein(const ina233_device_t *device, ina233_energy_t *energy)
{
	uint8_t block[6];
	uint8_t actual_length = 0u;

	if (device == 0 || !device->initialized || energy == 0) {
		return false;
	}

	/* READ_EIN (0x86): 6-byte PMBus block read */
	if (!twi_read_block(device->address, INA233_REG_READ_EIN,
		block, 6u, &actual_length) || actual_length < 6u) {
		return false;
	}

	/* Byte 0-1: Accumulator (16-bit, unsigned, LSB first)
	 * Byte 2:   Power accumulator rollover count (8-bit)
	 * Byte 3-5: Sample count (24-bit, LSB first) */
	uint16_t acc = (uint16_t)block[0] | ((uint16_t)block[1] << 8);
	uint32_t roll = (uint32_t)block[2];
	uint32_t sc = (uint32_t)block[3]
		| ((uint32_t)block[4] << 8)
		| ((uint32_t)block[5] << 16);

	energy->accumulator_16bit = acc;
	energy->rollover_count = (uint8_t)roll;
	energy->sample_count = sc;
	energy->raw_power_sum = (roll * 65536UL) + (uint32_t)acc;

	if (sc > 0u) {
		/* Power_LSB = 25 * Current_LSB = 1.25 mW/bit = 0.00125 W/bit */
		energy->avg_power_w = ((float)energy->raw_power_sum / (float)sc) * 0.00125f;
	} else {
		energy->avg_power_w = 0.0f;
	}

	energy->energy_wh = 0.0f;
	energy->v_avg = 0.0f;
	energy->capacity_mah = 0u;

	return true;
}

bool ina233_reset_ein(const ina233_device_t *device)
{
	ina233_energy_t dummy;
	/* Reading READ_EIN automatically clears accumulator and sample count in INA233 */
	return ina233_read_ein(device, &dummy);
}
