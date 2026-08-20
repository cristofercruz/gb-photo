#pragma bank 255

#include <gbdk/platform.h>
#include <stdint.h>

#include "compat.h"
#include "gbcamera.h"
#include "systemhelpers.h"
#include "state_camera.h"

/** Spot metering areas.

    A deliberately biased alternative to the broad "Overall" area below. They are sampled
    the same dense way rather than from a handful of scattered points, because a sparse
    sample is what made the servo wander: with only a few points, one dark object landing
    on one of them swings the whole measurement.

    Each spot is 6x4 tiles read every other tile row, which is 6*4*4 == 96 rows at up to
    24 each, so it comes out on the same 0..HISTOGRAM_MAX_VALUE scale as the overall
    walk and the brightness setpoint means the same thing in every mode.
*/
#define SPOT_TILES_W 6
#define SPOT_TILES_H 4
#define TILE_BYTES   16
#define TILE_ROW_BYTES (16 * TILE_BYTES)

typedef struct meter_spot_t { uint8_t x, y; } meter_spot_t;

static const meter_spot_t meter_spots[N_AUTOEXP_AREAS] = {
    [area_center]  = { 5, 5 },
    [area_top]     = { 5, 1 },
    [area_right]   = { 9, 5 },
    [area_bottom]  = { 5, 9 },
    [area_left]    = { 1, 5 },
    [area_overall] = { 0, 0 }       // unused; the overall area uses screen_meter()
};


/* The metering setpoint, as a fraction of full scale.

   The servo drives the measured/target ratio to a setpoint of 36 against a target byte
   of 84, so it settles at 84*36 = 3024 out of a full scale of 6912 -- exactly 7/16.
   Expressing it as a fraction rather than a raw number is what keeps it correct if the
   number of sampled points ever changes. */
void AT((SPOT_TILES_W * SPOT_TILES_H * 4 * 24 * 7) / 16) __histogram_target_value;
void AT(SPOT_TILES_W * SPOT_TILES_H) __histogram_points_count;
void AT(SPOT_TILES_W * SPOT_TILES_H * 4 * 24) __histogram_max_value;


extern const uint8_t bit_count_table[];     // aligned to 256 byte boundary

static uint8_t histogram_counter;


/** The broad metering walk.

    Two interleaved sweeps over the frame, reading ONE tile row at a time -- two bytes,
    one bit-plane each -- and stepping by a stride between reads. The first sweep's
    running total is doubled before the second is added, which is where the weighting
    comes from.

    The two sweeps are not concentric rings. They cover largely the same broad block,
    columns 2-14 and rows 2-11, and between them they touch 107 of the frame's 224 tiles
    with 288 weighted samples. That breadth is the point: metering a wide area is what
    stops a dark subject in the middle from dragging the exposure up and blowing out
    everything behind it.

    The total comes to 288 * 24 = 6912 at full darkness. Dividing by three puts it on the
    same 0..HISTOGRAM_MAX_VALUE scale the rest of this file uses, and lands the setpoint
    (84 * 36 = 3024) exactly on HISTOGRAM_TARGET_VALUE.
*/
#define METER_P1_START   ((const uint8_t *)0xA320)
#define METER_P2_START   ((const uint8_t *)0xA540)
#define METER_P1_PASSES  10
#define METER_P2_PASSES  6

static const uint8_t meter_strides_p1[] = { 0x14,0x14,0x04,0x14,0x14,0x04,0x14,0x04,0x14,0x14,0x04,0x4F };
static const uint8_t meter_strides_p2[] = { 0x14,0x14,0x04,0x14,0x14,0x04,0x14,0x84 };

/* The sweeps run from hand-written assembly on the Game Boy targets.

   Metering is the one routine that runs in full on every viewfinder frame -- 168 tile
   rows for the overall area -- so the per-row cost sets the frame rate. Written in C it
   is a function call, a 16-bit accumulate and a 16-bit pointer add per row; in assembly
   it is ten instructions plus the stride, with the two bit-planes accumulated in D and E
   as bytes and folded into the 16-bit total once per pass. A pass is at most 24 rows, so
   neither plane can exceed 192 and the byte accumulators cannot overflow.

   The walks are the same addresses in either path -- the assembly only changes how fast
   they are read, never which rows are read or how they are weighted. */
uint16_t meter_total;
uint8_t  meter_passes;
uint16_t meter_spot_ptr;

#ifdef NINTENDO

void meter_sweep_p1(void) NAKED {
    __asm
        ld hl, #0xA320
        ld a, #10
        ld (#_meter_passes), a
    1$:
        ld b, #0x3E             ; bit_count_table page
        ld de, #0x0000          ; D = high-plane sum, E = low-plane sum
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        ld a, #0x14
        add l
        ld l, a
        adc h
        sub l
        ld h, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        ld a, #0x14
        add l
        ld l, a
        adc h
        sub l
        ld h, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        ld a, #0x04
        add l
        ld l, a
        adc h
        sub l
        ld h, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        ld a, #0x14
        add l
        ld l, a
        adc h
        sub l
        ld h, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        ld a, #0x14
        add l
        ld l, a
        adc h
        sub l
        ld h, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        ld a, #0x04
        add l
        ld l, a
        adc h
        sub l
        ld h, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        ld a, #0x14
        add l
        ld l, a
        adc h
        sub l
        ld h, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        ld a, #0x04
        add l
        ld l, a
        adc h
        sub l
        ld h, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        ld a, #0x14
        add l
        ld l, a
        adc h
        sub l
        ld h, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        ld a, #0x14
        add l
        ld l, a
        adc h
        sub l
        ld h, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        ld a, #0x04
        add l
        ld l, a
        adc h
        sub l
        ld h, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        ld a, #0x4F
        add l
        ld l, a
        adc h
        sub l
        ld h, a
        push hl
        ld a, d
        add a          ; 2*D, may carry
        ld c, a
        ld a, #0x00
        rla
        ld b, a
        ld a, c
        add e          ; + E, may carry again
        ld c, a
        jr nc, 2$
        inc b
    2$:
        ld hl, #_meter_total
        ld a, (hl)
        add c
        ld (hl+), a
        ld a, (hl)
        adc b
        ld (hl), a
        pop hl
        ld a, (#_meter_passes)
        dec a
        ld (#_meter_passes), a
        jp nz, 1$
        ret
    __endasm;
}

void meter_sweep_p2(void) NAKED {
    __asm
        ld hl, #0xA540
        ld a, #6
        ld (#_meter_passes), a
    1$:
        ld b, #0x3E             ; bit_count_table page
        ld de, #0x0000          ; D = high-plane sum, E = low-plane sum
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        ld a, #0x14
        add l
        ld l, a
        adc h
        sub l
        ld h, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        ld a, #0x14
        add l
        ld l, a
        adc h
        sub l
        ld h, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        ld a, #0x04
        add l
        ld l, a
        adc h
        sub l
        ld h, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        ld a, #0x14
        add l
        ld l, a
        adc h
        sub l
        ld h, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        ld a, #0x14
        add l
        ld l, a
        adc h
        sub l
        ld h, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        ld a, #0x04
        add l
        ld l, a
        adc h
        sub l
        ld h, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        ld a, #0x14
        add l
        ld l, a
        adc h
        sub l
        ld h, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        ld a, #0x84
        add l
        ld l, a
        adc h
        sub l
        ld h, a
        push hl
        ld a, d
        add a          ; 2*D, may carry
        ld c, a
        ld a, #0x00
        rla
        ld b, a
        ld a, c
        add e          ; + E, may carry again
        ld c, a
        jr nc, 2$
        inc b
    2$:
        ld hl, #_meter_total
        ld a, (hl)
        add c
        ld (hl+), a
        ld a, (hl)
        adc b
        ld (hl), a
        pop hl
        ld a, (#_meter_passes)
        dec a
        ld (#_meter_passes), a
        jp nz, 1$
        ret
    __endasm;
}

void meter_sweep_spot(void) NAKED {
    __asm
        ld a, (#_meter_spot_ptr)
        ld l, a
        ld a, (#_meter_spot_ptr + 1)
        ld h, a
        ld a, #4
        ld (#_meter_passes), a
    1$:
        ld b, #0x3E
        ld de, #0x0000
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        inc hl
        inc hl
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add e
        ld e, a
        ld a, (hl+)
        ld c, a
        ld a, (bc)
        add d
        ld d, a
        ld a, #0xA2
        add l
        ld l, a
        adc h
        sub l
        ld h, a
        push hl
        ld a, d
        add a          ; 2*D, may carry
        ld c, a
        ld a, #0x00
        rla
        ld b, a
        ld a, c
        add e          ; + E, may carry again
        ld c, a
        jr nc, 2$
        inc b
    2$:
        ld hl, #_meter_total
        ld a, (hl)
        add c
        ld (hl+), a
        ld a, (hl)
        adc b
        ld (hl), a
        pop hl
        ld a, (#_meter_passes)
        dec a
        ld (#_meter_passes), a
        jp nz, 1$
        ret
    __endasm;
}

#else   /* z80 targets: the same walks, in C */

static void meter_sweep(const uint8_t * p, const uint8_t * strides, uint8_t n, uint8_t passes) {
    while (passes--)
        for (uint8_t i = 0; i != n; i++) {
            meter_total += (uint16_t)bit_count_table[p[0]] + ((uint16_t)bit_count_table[p[1]] << 1);
            p += 2 + strides[i];
        }
}
void meter_sweep_p1(void)   { meter_sweep((const uint8_t *)0xA320, meter_strides_p1, LENGTH(meter_strides_p1), METER_P1_PASSES); }
void meter_sweep_p2(void)   { meter_sweep((const uint8_t *)0xA540, meter_strides_p2, LENGTH(meter_strides_p2), METER_P2_PASSES); }
void meter_sweep_spot(void) {
    static const uint8_t spot_strides[24] = { 2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,0xA2 };
    meter_sweep((const uint8_t *)meter_spot_ptr, spot_strides, 24, 4);
}

#endif

int16_t calculate_histogram(autoexp_area_e area) BANKED {
    CAMERA_SWITCH_RAM(CAMERA_BANK_LAST_SEEN);
    meter_total = 0;
    if (area == area_overall) {
        meter_sweep_p1();
        meter_total <<= 1;              // the first sweep carries double weight
        meter_sweep_p2();
        return meter_total / 3;         // 0..6912 -> 0..HISTOGRAM_MAX_VALUE
    }
    const meter_spot_t * spot = &meter_spots[area];
    meter_spot_ptr = (uint16_t)(last_seen + ((uint16_t)spot->y * TILE_ROW_BYTES) + ((uint16_t)spot->x * TILE_BYTES));
    meter_sweep_spot();
    return meter_total;                 // 96 rows x 24 == HISTOGRAM_MAX_VALUE
}
