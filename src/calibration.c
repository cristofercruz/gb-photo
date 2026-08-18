#pragma bank 255

#include <gbdk/platform.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "compat.h"
#include "gbcamera.h"
#include "systemhelpers.h"
#include "state_camera.h"
#include "calibration.h"

camera_calibration_t camera_calibration;

/** Successive-approximation tables for the output reference voltage.

    Each maps the current six-bit CNTR5 value to the next one, and six lookups walk the
    full +-992 mV range in halving steps of 512, 256, 128, 64 and 32 mV. Which table to
    consult comes from how the last captured frame evaluated, so this is a binary search
    on the sensor's dark level, run against live frames.
*/
static const uint8_t voltage_search_darker[64] = {
#include "calib_darker.inc"
};
static const uint8_t voltage_search_brighter[64] = {
#include "calib_brighter.inc"
};

/** Threshold levels the dither matrix is flattened to for each measurement.

    These twelve bytes are the *input* to the measurement below, not its output: each
    band's search runs against a flat dither at its own level. They are read from the save
    when a valid set is stored there, and only fall back to the constants below when it
    fails its checksum.

    The stored set is per camera. It is written by the factory procedure that a save full
    of $AA triggers -- the one that shows "STORE PLEASE WAIT" and has to be run in
    complete darkness -- and it lives at $AFF2 in SRAM bank 2 with an echo at $BFF2 in
    bank 8 (offsets 0x04FF2 and 0x11FF2 in a .sav). A camera that has been through that
    procedure carries better numbers than any constant, so reading them is preferable.
*/
#define CALIB_N_REFERENCES     12
#define CALIB_STORED_BANK      2
#define CALIB_STORED_ADDR      0xAFF2
#define CALIB_STORED_ECHO_BANK 8
#define CALIB_STORED_ECHO_ADDR 0xBFF2
#define CALIB_STORED_SUM_SEED  0x0D
#define CALIB_STORED_XOR_SEED  0x23

// last-resort fallbacks, used only when no valid set is stored in the save
static const uint8_t calib_reference_defaults[CALIB_N_REFERENCES] = {
    0x7E, 0x7F, 0x7F, 0x7F,     // band 0 trial, indexed by gain_lo
    0x7E, 0x7D, 0x7E, 0x7E,     // band 1 trial, indexed by gain_lo
    0x7D, 0x7E,                 // band 2 trial, indexed by gain_lo > 1
    0x7D,                       // band 3 trial
    0x6A                        // band 4 trial
};
static uint8_t calib_reference[CALIB_N_REFERENCES];

/** Read one copy of the stored references and verify it: a running sum plus $0D and a
    running xor plus $23, in the two bytes that follow.
*/
static bool calib_load_stored(uint8_t bank, uint16_t address) {
    CAMERA_SWITCH_RAM(bank);
    const uint8_t * stored = (const uint8_t *)address;
    uint8_t sum = 0, parity = 0;
    for (uint8_t i = 0; i != CALIB_N_REFERENCES; i++) {
        uint8_t value = *stored++;
        calib_reference[i] = value;
        sum += value;
        parity ^= value;
    }
    if ((uint8_t)(sum + CALIB_STORED_SUM_SEED) != *stored++) return false;
    return ((uint8_t)(parity + CALIB_STORED_XOR_SEED) == *stored);
}

static void calib_load_references(void) {
    if (calib_load_stored(CALIB_STORED_BANK, CALIB_STORED_ADDR)) return;
    if (calib_load_stored(CALIB_STORED_ECHO_BANK, CALIB_STORED_ECHO_ADDR)) return;
    memcpy(calib_reference, calib_reference_defaults, sizeof(calib_reference));
}

#define CALIB_TRIAL_BAND0(g) calib_reference[0 + (g)]
#define CALIB_TRIAL_BAND1(g) calib_reference[4 + (g)]
#define CALIB_TRIAL_BAND2(g) calib_reference[8 + (((g) <= 1) ? 0 : 1)]
#define CALIB_TRIAL_BAND3    calib_reference[10]
#define CALIB_TRIAL_BAND4    calib_reference[11]

// CNTR5 rails the V sweep runs against, and the midpoint the O search starts from
#define CALIB_VOUT_POSITIVE_RAIL 0xBF
#define CALIB_VOUT_NEGATIVE_RAIL 0x9F
#define CALIB_VOUT_MIDPOINT      0xA0
#define CALIB_VOUT_SEARCH_STEPS  6

// CNTR4 seeds: both start at V=3, differing only in edge ratio (50% and 100%)
#define CALIB_CNTR4_EDGE_050 0x03
#define CALIB_CNTR4_EDGE_100 0x23

/** Where the captured frame is probed. Four bytes at stride two, from $A002.

    This sits below $A100, where the live image is read from -- gbcamera.h treats
    $A000..$A0FF as unused, but the sensor writes its first rows there and they carry
    the reference behaviour this measurement needs. If calibration comes out wrong on a
    real camera this is the first thing to try moving (to $A102).
*/
#define CALIB_PROBE_ADDR  0xA002
#define CALIB_PROBE_COUNT 4
#define CALIB_PROBE_LEVEL 9

/** Count set bits in the sampled bytes and report whether the frame reads dark.

    Only bits 6, 5 and 4 are counted, giving a maximum of 12 against the threshold of 9.
    Bit 3 is deliberately left out: the stored reference levels this measurement runs
    against were derived from a three-bit count, so including it would shift the
    operating point away from what those references assume. Set CALIBRATION_COUNT_BIT3
    to 1 to include it and see the difference.
*/
#ifndef CALIBRATION_COUNT_BIT3
#define CALIBRATION_COUNT_BIT3 0
#endif

static bool frame_reads_dark(void) {
    CAMERA_SWITCH_RAM(CAMERA_BANK_LAST_SEEN);
    const uint8_t * probe = (const uint8_t *)CALIB_PROBE_ADDR;
    uint8_t count = 0;
    for (uint8_t i = CALIB_PROBE_COUNT; i != 0; i--, probe += 2) {
        uint8_t sample = *probe;
        if (sample & 0x40) count++;
        if (sample & 0x20) count++;
        if (sample & 0x10) count++;
#if (CALIBRATION_COUNT_BIT3 == 1)
        if (sample & 0x08) count++;
#endif
    }
    CAMERA_SWITCH_RAM(CAMERA_BANK_REGISTERS);
    return (count >= CALIB_PROBE_LEVEL);
}

static void calib_capture(void) {
    CAMERA_SWITCH_RAM(CAMERA_BANK_REGISTERS);
#if defined(CAMERA_EMULATE_CAPTURE)
    CAM_REG_CAPTURE = SHADOW.CAM_REG_CAPTURE & ~CAM00F_CAPTURING;
#else
    CAM_REG_CAPTURE = SHADOW.CAM_REG_CAPTURE | CAM00F_CAPTURING;
    while (CAM_REG_CAPTURE & CAM00F_CAPTURING);
#endif
}

static void calib_flatten_dither(uint8_t level) {
    CAMERA_SWITCH_RAM(CAMERA_BANK_REGISTERS);
    for (uint8_t i = 0; i != sizeof(CAM_DITHERPATTERN); i++) CAM_DITHERPATTERN[i] = level;
}

static void calib_set_vout(uint8_t value) {
    CAMERA_SWITCH_RAM(CAMERA_BANK_REGISTERS);
    CAM_REG_ZEROVOUT = SHADOW.CAM_REG_ZEROVOUT = value;
}
static void calib_set_vref(uint8_t v) {
    CAMERA_SWITCH_RAM(CAMERA_BANK_REGISTERS);
    CAM_REG_EDRAINVVREF = SHADOW.CAM_REG_EDRAINVVREF = (SHADOW.CAM_REG_EDRAINVVREF & 0xF8) | (v & 0x07);
}

/** The per-band search. Three phases, all driven by live captures.

    1. At the positive rail, walk the bias voltage V up while the frame reads dark,
       stopping at V=7.
    2. Only if that never fired: at the negative rail, walk V down while the frame
       reads bright, stopping at V=1.
    3. From the midpoint, binary-search the output reference voltage in six steps.

    Leaves the result in the CNTR4 and CNTR5 shadows for the caller to store.
*/
static void calib_search(void) {
    bool moved = false;

    calib_set_vout(CALIB_VOUT_POSITIVE_RAIL);
    while (1) {
        calib_capture();
        if (!frame_reads_dark()) break;
        moved = true;
        uint8_t v = (SHADOW.CAM_REG_EDRAINVVREF & 0x07) + 1;
        calib_set_vref(v);
        if (v >= 0x07) goto refine;
    }
    if (!moved) {
        calib_set_vout(CALIB_VOUT_NEGATIVE_RAIL);
        while (1) {
            calib_capture();
            if (frame_reads_dark()) break;
            uint8_t v = SHADOW.CAM_REG_EDRAINVVREF & 0x07;
            if (v <= 1) break;              // the seed of 3 is what normally stops this
            calib_set_vref(v - 1);
            if ((v - 1) == 1) break;
        }
    }

refine:
    calib_set_vout(CALIB_VOUT_MIDPOINT);
    for (uint8_t step = CALIB_VOUT_SEARCH_STEPS; step != 0; step--) {
        calib_capture();
        const uint8_t * table = (frame_reads_dark()) ? voltage_search_darker : voltage_search_brighter;
        calib_set_vout(table[SHADOW.CAM_REG_ZEROVOUT & 0x3F]);
    }
}

// one measurement: flatten the dither, program the band's gain and edge ratio, search
static void calib_band(uint8_t band, uint8_t trial_level, uint8_t cntr1, uint8_t cntr4_seed) {
    calib_flatten_dither(trial_level);
    CAMERA_SWITCH_RAM(CAMERA_BANK_REGISTERS);
    CAM_REG_EDEXOPGAIN  = SHADOW.CAM_REG_EDEXOPGAIN  = cntr1;
    CAM_REG_EDRAINVVREF = SHADOW.CAM_REG_EDRAINVVREF = cntr4_seed;
    calib_search();
    camera_calibration.voltage_ref[band] = SHADOW.CAM_REG_EDRAINVVREF & 0x07;
    camera_calibration.voltage_out[band] = SHADOW.CAM_REG_ZEROVOUT & 0x7F;
}

void camera_calibrate(void) BANKED {
#if defined(CAMERA_EMULATE_CAPTURE)
    return;                                 // no sensor to measure
#else
    // prefer this camera's own stored references over the built-in fallbacks
    calib_load_references();

    CAMERA_SWITCH_RAM(CAMERA_BANK_REGISTERS);
    SHADOW.CAM_REG_CAPTURE = CAM_REG_CAPTURE = CAM00F_POSITIVE;
    CAM_REG_EXPTIME = SHADOW.CAM_REG_EXPTIME = swap_bytes(CAM02_MIN_VALUE);

    /* Find the lowest gain at which this sensor reads dark at minimum exposure -- a
       direct measurement of how sensitive this particular part is. Stops at 3. */
    uint8_t gain_lo = 0;
    while (1) {
        CAMERA_SWITCH_RAM(CAMERA_BANK_REGISTERS);
        CAM_REG_EDEXOPGAIN  = SHADOW.CAM_REG_EDEXOPGAIN  = gain_lo;
        CAM_REG_EDRAINVVREF = SHADOW.CAM_REG_EDRAINVVREF = 0x05;
        calib_set_vout(0x80);
        calib_flatten_dither(0xD5);
        calib_search();
        calib_set_vout(SHADOW.CAM_REG_ZEROVOUT & 0x7F);
        calib_flatten_dither(0x80);
        calib_capture();
        if (frame_reads_dark()) break;
        if (++gain_lo == 3) break;
    }
    uint8_t gain_hi = (gain_lo <= 1) ? 0x04 : 0x05;

    /* The five bands, each with its own gain, edge ratio and trial level. The trial
       level indexes shift with gain_lo because a more sensitive part needs a different
       threshold to read the same way. */
    calib_band(0, CALIB_TRIAL_BAND0(gain_lo), gain_lo | 0x20, CALIB_CNTR4_EDGE_050);
    calib_band(1, CALIB_TRIAL_BAND1(gain_lo), gain_lo | 0x20, CALIB_CNTR4_EDGE_100);
    calib_band(2, CALIB_TRIAL_BAND2(gain_lo), gain_hi | 0x20, CALIB_CNTR4_EDGE_100);
    calib_band(3, CALIB_TRIAL_BAND3,          0x08 | 0x20,    CALIB_CNTR4_EDGE_100);
    calib_band(4, CALIB_TRIAL_BAND4,          0x0A,           CALIB_CNTR4_EDGE_050);

    camera_calibration.gains = gain_lo | (gain_hi << 4);
#endif
}
