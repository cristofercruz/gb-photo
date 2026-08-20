#ifndef __CALIBRATION_H_INCLUDE__
#define __CALIBRATION_H_INCLUDE__

#include <stdint.h>
#include <stdbool.h>

#include "compat.h"

/** Per-unit sensor calibration.

    There are no useful constants for the output bias voltage (CNTR4's V field) or the
    output reference voltage (CNTR5). They have to be measured, once per cartridge, by
    capturing real frames and evaluating them -- because they depend on the individual
    M64282FP and drift with temperature. Values good for one sensor are not good for
    another, so a table of fixed numbers cannot do this job.

    Five bands, matching the five states of the exposure gain ladder.
*/
#define N_CALIBRATION_BANDS 5

typedef struct camera_calibration_t {
    uint8_t gains;                              // gain_lo | (gain_hi << 4); zero == never calibrated
    uint8_t voltage_ref[N_CALIBRATION_BANDS];   // CNTR4 V field, 0..7
    uint8_t voltage_out[N_CALIBRATION_BANDS];   // CNTR5 with bit 7 cleared
    /* Hash of the twelve threshold references the search ran against. The measurement is
       only meaningful for those references, so if they change -- the factory procedure
       having regenerated them, or a previously unreadable set becoming readable -- the
       stored result is stale and has to be measured again. */
    uint8_t reference_hash;
} camera_calibration_t;

extern camera_calibration_t camera_calibration;

#define CALIBRATION_GAIN_LO() (camera_calibration.gains & 0x0F)
#define CALIBRATION_GAIN_HI() (camera_calibration.gains >> 4)

inline bool camera_is_calibrated(void) {
    return (camera_calibration.gains != 0);
}

/** CNTR5 output reference voltage -> millivolts, the inverse of TO_VOLTAGE_OUT().
    Bit 5 is the sign (set == positive), bits 4:0 the magnitude, one step per 32 mV.
*/
inline int16_t calibration_voltage_out_mv(uint8_t cntr5) {
    int16_t magnitude = (int16_t)(cntr5 & 0x1F) * VOLTAGE_OUT_STEP;
    return (cntr5 & 0x20) ? magnitude : (int16_t)(0 - magnitude);
}

/** Run the full sensor calibration. Takes roughly 70 captures at minimum exposure,
    about two seconds. Leaves the camera registers mapped.
*/
void camera_calibrate(void) BANKED;

/** True when the stored calibration was measured against the references now in place. */
bool camera_calibration_is_current(void) BANKED;

#endif
