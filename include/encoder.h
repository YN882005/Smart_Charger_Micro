#ifndef ENCODER_H
#define ENCODER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	ENCODER_EVENT_NONE = 0,
	ENCODER_EVENT_CW,
	ENCODER_EVENT_CCW,
	ENCODER_EVENT_PRESS,
	ENCODER_EVENT_DOUBLE_PRESS
} encoder_event_t;

void encoder_init(void);

/* Poll for button single press or double press */
encoder_event_t encoder_poll(void);

/* Dual-sensitivity step retrieval:
 * is_value_picking == false: 2 physical detents (+-2 ticks) in same direction.
 *                            Resets accumulator if direction reverses.
 * is_value_picking == true:  1 physical detent (+-1 tick) for immediate response.
 * Returns +1 (CW), -1 (CCW), or 0.
 */
int8_t Encoder_GetStep(bool is_value_picking);
int8_t encoder_get_step(bool is_value_picking);

/* Reset accumulator on screen changes */
void Encoder_ResetAccumulator(void);

/* Helper to simulate raw encoder detents */
void Encoder_SimulateDetent(int8_t detents);

#endif /* ENCODER_H */
