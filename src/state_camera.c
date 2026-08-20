#pragma bank 255

#include <gbdk/platform.h>
#include <gbdk/metasprites.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "compat.h"
#include "gbcamera.h"
#include "musicmanager.h"
#include "systemdetect.h"
#include "systemhelpers.h"
#include "joy.h"
#include "screen.h"
#include "states.h"
#include "bankdata.h"
#include "gbprinter.h"
#include "linkcable.h"
#include "fade.h"
#include "vector.h"
#include "counter.h"
#include "protected.h"
#include "histogram.h"
#include "scrollbar.h"
#include "ir.h"
#include "sd.h"

#include "globals.h"
#include "state_camera.h"
#include "state_gallery.h"
#include "pic-n-rec.h"
#include "load_save.h"
#include "calibration.h"

#include "misc_assets.h"

#include "cursors.h"

// audio assets
#include "sound_ok.h"
#include "sound_error.h"
#include "sound_menu_alter.h"
#include "sound_timer.h"
#include "shutter01.h"
#include "shutter02.h"

// menus
#include "menus.h"
#include "menu_codes.h"
#include "menu_main.h"
#include "menu_popup_camera.h"
#include "menu_msgbox.h"

// dither patterns
#include "dither_patterns.h"

// frames
#include "print_frames.h"

BANKREF(state_camera)

camera_state_options_t camera_state;

bool image_live_preview = true;
bool recording_video = false;
bool camera_do_shutter = false;
bool one_iteration_autoexp = false;

COUNTER_DECLARE(camera_shutter_timer, uint8_t, 0);
COUNTER_DECLARE(camera_repeat_counter, uint8_t, 0);

COUNTER_DECLARE(camera_AEB_counter, uint8_t, 0);
uint16_t AEB_exposure_list[MAX_AEB_IMAGES];
#define last_AEB_exposure (AEB_exposure_list[MIDDLE_AEB_IMAGE])
bool AEB_capture_in_progress = false;

camera_mode_settings_t current_settings[N_CAMERA_MODES];

camera_shadow_regs_t SHADOW;        // camera shadow registers for reading

volatile uint8_t camera_PnR_delay;  // PicNRec delay counter

/** Auto-exposure servo, modelled to mimic the original camera.

    The error is a RATIO of the measured value to the target, not a difference, and the
    correction is the current exposure shifted right by a count taken from the table
    below -- so the step is always a fraction of where the exposure already is. That is
    the right shape for exposure, whose perceptual effect is ratiometric: adding 10 to
    an exposure of 40 is a large change, adding 10 to 4000 is nothing.

    Note that the measurement counts DARKNESS, not brightness: in packed 2bpp a set bit
    is a dark pixel, so the servo lengthens exposure when the measurement reads high.
*/
#define AUTOEXP_RATIO_SETPOINT  36      // the ratio the servo drives towards
#define AUTOEXP_RATIO_CLAMP     159     // the subtract loop stops here
#define AUTOEXP_NO_CORRECTION   16      // a shift of 16 on a 16-bit value makes the step zero

/** Shift counts indexed by the measured ratio.

    The table is non-monotonic and peaks at 16 across indices 35-37. A shift of 16 gives
    a step of zero, so the peak IS the deadband and it sits exactly on the setpoint.
    Because the direction comes from comparing the ratio against the setpoint rather
    than from which table was consulted, one table serves both directions.

    Read the two flanks: at index 0 (far too bright) the shift is 2, a -25% step, while
    at index 159 (far too dark) it is 3, a +12.5% step. Cutting exposure is allowed to
    be twice as aggressive as raising it, which is the right asymmetry -- a blown-out
    frame carries no information at all, a dark one still does.
*/
static const uint8_t autoexp_shift_table[AUTOEXP_RATIO_CLAMP + 1] = {
     2,  3,  3,  3,  4,  3,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,
     4,  4,  4,  5,  4,  5,  5,  5,  5,  5,  6,  5,  6,  6,  6,  7,
     7,  8,  9, 16, 16, 16,  9,  8,  7,  7,  6,  6,  5,  6,  5,  5,
//           ^^^^^^^^^^^^ deadband, centred on AUTOEXP_RATIO_SETPOINT
     5,  5,  4,  5,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,
     4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,
     3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,
     3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,
     3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,
     3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,
     3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3
};

/** measured / target, clamped so the result can always index the table above.

    A target of zero means the setpoint has been dragged to the very bottom of its
    range, where the servo should simply run exposure up; report the clamp for it
    rather than spinning.
*/
static uint8_t autoexp_ratio(uint16_t measured, uint8_t target) {
    if (!target) return AUTOEXP_RATIO_CLAMP;
    /* Repeated subtraction would avoid a divide the SM83 does not have, but it costs
       up to 159 iterations of 16-bit subtract on the frame's critical path. Clamping a
       division gives the identical number for every input at a fraction of the cost. */
    uint16_t ratio = measured / target;
    return (ratio > AUTOEXP_RATIO_CLAMP) ? AUTOEXP_RATIO_CLAMP : (uint8_t)ratio;
}

#define AUTOEXP_AREA_X      18
#define AUTOEXP_AREA_Y      10
#define SHUTTER_REPEAT_X    18
#define SHUTTER_REPEAT_Y    12
#define SHUTTER_TIMER_X     18
#define SHUTTER_TIMER_Y     14

#define SS_BRIGHTNESS_X     1
#define SS_BRIGHTNESS_Y     2
#define SS_BRIGHTNESS_LEN   14
static scrollbar_t ss_brightness;

#define SS_CONTRAST_X       2
#define SS_CONTRAST_Y       16
#define SS_CONTRAST_LEN     16
static scrollbar_t ss_contrast;

/** Exposure ladder, one twentieth of a stop per step.

    A true geometric series, so every step is the same perceptual increment and a held
    ramp feels linear. Values are generated in register counts and labelled in
    microseconds through the exact conversion, so the numbers shown are real exposure
    times rather than the 4.9% overestimate the old flat 16us conversion produced.

    Below about 1.4 ms the register cannot express a twentieth of a stop -- at 17 counts
    a single increment is already 0.083 stops -- so the bottom of the ladder falls back
    to single-count steps, which is the finest the hardware has.
*/
static const uint16_t exposures[] = {
    TO_EXPOSURE_VALUE(244), TO_EXPOSURE_VALUE(259), TO_EXPOSURE_VALUE(275), TO_EXPOSURE_VALUE(290),
    TO_EXPOSURE_VALUE(305), TO_EXPOSURE_VALUE(320), TO_EXPOSURE_VALUE(336), TO_EXPOSURE_VALUE(351),
    TO_EXPOSURE_VALUE(366), TO_EXPOSURE_VALUE(381), TO_EXPOSURE_VALUE(397), TO_EXPOSURE_VALUE(412),
    TO_EXPOSURE_VALUE(427), TO_EXPOSURE_VALUE(443), TO_EXPOSURE_VALUE(458), TO_EXPOSURE_VALUE(473),
    TO_EXPOSURE_VALUE(488), TO_EXPOSURE_VALUE(504), TO_EXPOSURE_VALUE(519), TO_EXPOSURE_VALUE(534),
    TO_EXPOSURE_VALUE(549), TO_EXPOSURE_VALUE(565), TO_EXPOSURE_VALUE(580), TO_EXPOSURE_VALUE(595),
    TO_EXPOSURE_VALUE(610), TO_EXPOSURE_VALUE(626), TO_EXPOSURE_VALUE(641), TO_EXPOSURE_VALUE(656),
    TO_EXPOSURE_VALUE(687), TO_EXPOSURE_VALUE(717), TO_EXPOSURE_VALUE(748), TO_EXPOSURE_VALUE(778),
    TO_EXPOSURE_VALUE(809), TO_EXPOSURE_VALUE(839), TO_EXPOSURE_VALUE(870), TO_EXPOSURE_VALUE(900),
    TO_EXPOSURE_VALUE(931), TO_EXPOSURE_VALUE(961), TO_EXPOSURE_VALUE(992), TO_EXPOSURE_VALUE(1022),
    TO_EXPOSURE_VALUE(1053), TO_EXPOSURE_VALUE(1083), TO_EXPOSURE_VALUE(1129), TO_EXPOSURE_VALUE(1175),
    TO_EXPOSURE_VALUE(1221), TO_EXPOSURE_VALUE(1266), TO_EXPOSURE_VALUE(1312), TO_EXPOSURE_VALUE(1358),
    TO_EXPOSURE_VALUE(1404), TO_EXPOSURE_VALUE(1450), TO_EXPOSURE_VALUE(1495), TO_EXPOSURE_VALUE(1541),
    TO_EXPOSURE_VALUE(1602), TO_EXPOSURE_VALUE(1663), TO_EXPOSURE_VALUE(1724), TO_EXPOSURE_VALUE(1785),
    TO_EXPOSURE_VALUE(1846), TO_EXPOSURE_VALUE(1907), TO_EXPOSURE_VALUE(1968), TO_EXPOSURE_VALUE(2045),
    TO_EXPOSURE_VALUE(2121), TO_EXPOSURE_VALUE(2197), TO_EXPOSURE_VALUE(2274), TO_EXPOSURE_VALUE(2350),
    TO_EXPOSURE_VALUE(2426), TO_EXPOSURE_VALUE(2518), TO_EXPOSURE_VALUE(2609), TO_EXPOSURE_VALUE(2701),
    TO_EXPOSURE_VALUE(2792), TO_EXPOSURE_VALUE(2884), TO_EXPOSURE_VALUE(2991), TO_EXPOSURE_VALUE(3098),
    TO_EXPOSURE_VALUE(3204), TO_EXPOSURE_VALUE(3311), TO_EXPOSURE_VALUE(3433), TO_EXPOSURE_VALUE(3555),
    TO_EXPOSURE_VALUE(3677), TO_EXPOSURE_VALUE(3799), TO_EXPOSURE_VALUE(3937), TO_EXPOSURE_VALUE(4074),
    TO_EXPOSURE_VALUE(4211), TO_EXPOSURE_VALUE(4364), TO_EXPOSURE_VALUE(4517), TO_EXPOSURE_VALUE(4669),
    TO_EXPOSURE_VALUE(4837), TO_EXPOSURE_VALUE(5005), TO_EXPOSURE_VALUE(5188), TO_EXPOSURE_VALUE(5371),
    TO_EXPOSURE_VALUE(5554), TO_EXPOSURE_VALUE(5753), TO_EXPOSURE_VALUE(5951), TO_EXPOSURE_VALUE(6165),
    TO_EXPOSURE_VALUE(6378), TO_EXPOSURE_VALUE(6607), TO_EXPOSURE_VALUE(6836), TO_EXPOSURE_VALUE(7080),
    TO_EXPOSURE_VALUE(7324), TO_EXPOSURE_VALUE(7584), TO_EXPOSURE_VALUE(7858), TO_EXPOSURE_VALUE(8133),
    TO_EXPOSURE_VALUE(8423), TO_EXPOSURE_VALUE(8713), TO_EXPOSURE_VALUE(9018), TO_EXPOSURE_VALUE(9338),
    TO_EXPOSURE_VALUE(9674), TO_EXPOSURE_VALUE(10010), TO_EXPOSURE_VALUE(10361), TO_EXPOSURE_VALUE(10727),
    TO_EXPOSURE_VALUE(11108), TO_EXPOSURE_VALUE(11505), TO_EXPOSURE_VALUE(11917), TO_EXPOSURE_VALUE(12344),
    TO_EXPOSURE_VALUE(12787), TO_EXPOSURE_VALUE(13245), TO_EXPOSURE_VALUE(13718), TO_EXPOSURE_VALUE(14206),
    TO_EXPOSURE_VALUE(14709), TO_EXPOSURE_VALUE(15228), TO_EXPOSURE_VALUE(15762), TO_EXPOSURE_VALUE(16312),
    TO_EXPOSURE_VALUE(16891), TO_EXPOSURE_VALUE(17487), TO_EXPOSURE_VALUE(18097), TO_EXPOSURE_VALUE(18738),
    TO_EXPOSURE_VALUE(19394), TO_EXPOSURE_VALUE(20081), TO_EXPOSURE_VALUE(20782), TO_EXPOSURE_VALUE(21515),
    TO_EXPOSURE_VALUE(22278), TO_EXPOSURE_VALUE(23056), TO_EXPOSURE_VALUE(23865), TO_EXPOSURE_VALUE(24704),
    TO_EXPOSURE_VALUE(25574), TO_EXPOSURE_VALUE(26474), TO_EXPOSURE_VALUE(27405), TO_EXPOSURE_VALUE(28366),
    TO_EXPOSURE_VALUE(29373), TO_EXPOSURE_VALUE(30411), TO_EXPOSURE_VALUE(31479), TO_EXPOSURE_VALUE(32593),
    TO_EXPOSURE_VALUE(33737), TO_EXPOSURE_VALUE(34927), TO_EXPOSURE_VALUE(36163), TO_EXPOSURE_VALUE(37445),
    TO_EXPOSURE_VALUE(38773), TO_EXPOSURE_VALUE(40146), TO_EXPOSURE_VALUE(41565), TO_EXPOSURE_VALUE(43030),
    TO_EXPOSURE_VALUE(44540), TO_EXPOSURE_VALUE(46112), TO_EXPOSURE_VALUE(47745), TO_EXPOSURE_VALUE(49423),
    TO_EXPOSURE_VALUE(51163), TO_EXPOSURE_VALUE(52963), TO_EXPOSURE_VALUE(54825), TO_EXPOSURE_VALUE(56763),
    TO_EXPOSURE_VALUE(58762), TO_EXPOSURE_VALUE(60837), TO_EXPOSURE_VALUE(62988), TO_EXPOSURE_VALUE(65216),
    TO_EXPOSURE_VALUE(67520), TO_EXPOSURE_VALUE(69901), TO_EXPOSURE_VALUE(72372), TO_EXPOSURE_VALUE(74921),
    TO_EXPOSURE_VALUE(77560), TO_EXPOSURE_VALUE(80292), TO_EXPOSURE_VALUE(83130), TO_EXPOSURE_VALUE(86060),
    TO_EXPOSURE_VALUE(89096), TO_EXPOSURE_VALUE(92239), TO_EXPOSURE_VALUE(95490), TO_EXPOSURE_VALUE(98862),
    TO_EXPOSURE_VALUE(102341), TO_EXPOSURE_VALUE(105957), TO_EXPOSURE_VALUE(109695), TO_EXPOSURE_VALUE(113571),
    TO_EXPOSURE_VALUE(117569), TO_EXPOSURE_VALUE(121719), TO_EXPOSURE_VALUE(126007), TO_EXPOSURE_VALUE(130447),
    TO_EXPOSURE_VALUE(135040), TO_EXPOSURE_VALUE(139801), TO_EXPOSURE_VALUE(144730), TO_EXPOSURE_VALUE(149826),
    TO_EXPOSURE_VALUE(155106), TO_EXPOSURE_VALUE(160568), TO_EXPOSURE_VALUE(166229), TO_EXPOSURE_VALUE(172089),
    TO_EXPOSURE_VALUE(178162), TO_EXPOSURE_VALUE(184448), TO_EXPOSURE_VALUE(190948), TO_EXPOSURE_VALUE(197678),
    TO_EXPOSURE_VALUE(204651), TO_EXPOSURE_VALUE(211868), TO_EXPOSURE_VALUE(219345), TO_EXPOSURE_VALUE(227081),
    TO_EXPOSURE_VALUE(235092), TO_EXPOSURE_VALUE(243378), TO_EXPOSURE_VALUE(251953), TO_EXPOSURE_VALUE(260834),
    TO_EXPOSURE_VALUE(270035), TO_EXPOSURE_VALUE(279556), TO_EXPOSURE_VALUE(289413), TO_EXPOSURE_VALUE(299622),
    TO_EXPOSURE_VALUE(310181), TO_EXPOSURE_VALUE(321121), TO_EXPOSURE_VALUE(332443), TO_EXPOSURE_VALUE(344162),
    TO_EXPOSURE_VALUE(356293), TO_EXPOSURE_VALUE(368851), TO_EXPOSURE_VALUE(381851), TO_EXPOSURE_VALUE(395325),
    TO_EXPOSURE_VALUE(409271), TO_EXPOSURE_VALUE(423706), TO_EXPOSURE_VALUE(438644), TO_EXPOSURE_VALUE(454117),
    TO_EXPOSURE_VALUE(470139), TO_EXPOSURE_VALUE(486725), TO_EXPOSURE_VALUE(503891), TO_EXPOSURE_VALUE(521667),
    TO_EXPOSURE_VALUE(540070), TO_EXPOSURE_VALUE(559113), TO_EXPOSURE_VALUE(578827), TO_EXPOSURE_VALUE(599243),
    TO_EXPOSURE_VALUE(620377), TO_EXPOSURE_VALUE(642258), TO_EXPOSURE_VALUE(664902), TO_EXPOSURE_VALUE(688354),
    TO_EXPOSURE_VALUE(712631), TO_EXPOSURE_VALUE(737762), TO_EXPOSURE_VALUE(763779), TO_EXPOSURE_VALUE(790710),
    TO_EXPOSURE_VALUE(818588), TO_EXPOSURE_VALUE(847458), TO_EXPOSURE_VALUE(877350), TO_EXPOSURE_VALUE(908295),
    TO_EXPOSURE_VALUE(940323), TO_EXPOSURE_VALUE(973480), TO_EXPOSURE_VALUE(999985)
};
/** Hold-to-accelerate for the adjustable fields.

    The joypad is interrupt driven and `joy` is consumed on every read, so it is not a
    level signal -- KEY_DOWN reads false on most frames even while the button is held,
    and cannot be used to tell a held key from a released one. What is reliable is the
    spacing of the changes themselves: they arrive one per autorepeat interval while the
    key is down, and stop when it comes up. So the ramp advances on each change and
    resets when a gap longer than a couple of intervals appears.
*/
#define EXPOSURE_RAMP_TIMEOUT (AUTOREPEAT_RATE * 3)
static uint8_t exposure_ramp = 0;
static uint16_t exposure_ramp_ts = 0;
static camera_menu_e exposure_ramp_field = idNone;
static change_direction_e exposure_ramp_dir = changeNone;

/** Find the ladder entry matching an exposure value.

    The index and the exposure are stored separately, so anything that moves the exposure
    without going through the ladder -- the one-shot autoexposure, or a save written by a
    build with a different table -- leaves them disagreeing. Resyncing avoids the next
    D-pad press jumping to wherever the stale index happened to point.
*/
static uint8_t exposure_index_for(uint16_t exposure) {
    for (uint8_t i = 0; i != LENGTH(exposures); i++)
        if (exposures[i] >= exposure) return i;
    return MAX_INDEX(exposures);
}

static uint8_t menu_ramp_multiplier(camera_menu_e field, change_direction_e dir) {
    /* A ramp is a sustained push one way. A different field, a gap in the changes, or a
       reversal all mean this is not that: toggling up and down is someone dialling in a
       value a step at a time, and accelerating it would be exactly wrong. */
    if ((field != exposure_ramp_field) || (dir != exposure_ramp_dir) ||
        ((uint16_t)(sys_time - exposure_ramp_ts) > EXPOSURE_RAMP_TIMEOUT))
        exposure_ramp = 0;
    exposure_ramp_field = field;
    exposure_ramp_dir = dir;
    exposure_ramp_ts = sys_time;
    if (exposure_ramp < 255) exposure_ramp++;
    // x1, x2, x3, x4 then x6 -- roughly half a second per tier at the autorepeat rate
    return (exposure_ramp <= 2) ? 2 :
           (exposure_ramp <= 4) ? 4 :
           (exposure_ramp <= 6) ? 6 :
           (exposure_ramp <= 9) ? 8 : 12;
}

static uint8_t exposure_ramp_step(change_direction_e dir) {
    // one ladder step in manual, two in assisted, as before -- the multiplier scales both
    return (uint8_t)(((OPTION(camera_mode) == camera_mode_manual) ? 1 : 2) * menu_ramp_multiplier(idExposure, dir));
}



static const table_value_t gains[] = {
    { CAM01_GAIN_140, "14.0" }, { CAM01_GAIN_155, "15.5" }, { CAM01_GAIN_170, "17.0" }, { CAM01_GAIN_185, "18.5" },
    { CAM01_GAIN_200, "20.0" }, { CAM01_GAIN_215, "21.5" }, { CAM01_GAIN_230, "23.0" }, { CAM01_GAIN_245, "24.5" },
    { CAM01_GAIN_260, "26.0" }, { CAM01_GAIN_275, "27.5" }, { CAM01_GAIN_290, "29.0" }, { CAM01_GAIN_305, "30.5" },
    { CAM01_GAIN_320, "32.0" }, { CAM01_GAIN_350, "35.0" }, { CAM01_GAIN_380, "38.0" }, { CAM01_GAIN_410, "41.0" },
    { CAM01_GAIN_440, "44.0" }, { CAM01_GAIN_455, "45.5" }, { CAM01_GAIN_470, "47.0" }, { CAM01_GAIN_515, "51.5" },
    { CAM01_GAIN_575, "57.5" }
};
static const table_value_t dither_patterns[N_DITHER_TYPES] = {
    [dither_type_Off]        = { dither_type_Off        , "Off"  },
    [dither_type_Default]    = { dither_type_Default    , "Def"  },
    [dither_type_2x2]        = { dither_type_2x2        , "2x2"  },
    [dither_type_Grid]       = { dither_type_Grid       , "Grid" },
    [dither_type_Maze]       = { dither_type_Maze       , "Maze" },
    [dither_type_Nest]       = { dither_type_Nest       , "Nest" },
    [dither_type_Fuzz]       = { dither_type_Fuzz       , "Fuzz" },
    [dither_type_Vertical]   = { dither_type_Vertical   , "Vert" },
    [dither_type_Horizonral] = { dither_type_Horizonral , "Hori" },
    [dither_type_Mix]        = { dither_type_Mix        , "Mix"  },
    [dither_type_Halftone]   = { dither_type_Halftone   , "Half" },
    [dither_type_Crosshatch] = { dither_type_Crosshatch , "Cros" }
};
static const table_value_t zero_points[] = {
    { CAM05_ZERO_DIS, "None" }, { CAM05_ZERO_POS, "Positv" }, { CAM05_ZERO_NEG, "Negtv" }
};
static const table_value_t edge_ratios[] = {
    { CAM04_EDGE_RATIO_050, "50%" }, { CAM04_EDGE_RATIO_075, "75%" }, { CAM04_EDGE_RATIO_100, "100%" },{ CAM04_EDGE_RATIO_125, "125%" },
    { CAM04_EDGE_RATIO_200, "200%" },{ CAM04_EDGE_RATIO_300, "300%" },{ CAM04_EDGE_RATIO_400, "400%" },{ CAM04_EDGE_RATIO_500, "500%" },
};
static const table_value_t voltage_refs[] = {
    { CAM04_VOLTAGE_REF_00, "0.0" }, { CAM04_VOLTAGE_REF_05, "0.5" }, { CAM04_VOLTAGE_REF_10, "1.0" }, { CAM04_VOLTAGE_REF_15, "1.5" },
    { CAM04_VOLTAGE_REF_20, "2.0" }, { CAM04_VOLTAGE_REF_25, "2.5" }, { CAM04_VOLTAGE_REF_30, "3.0" }, { CAM04_VOLTAGE_REF_35, "3.5" },
};
static const table_value_t edge_operations[] = {
    { CAM01_EDGEOP_2D, "2D" }, { CAM01_EDGEOP_HORIZ, "Horiz" }, { CAM01_EDGEOP_VERT, "Vert" },{ CAM01_EDGEOP_NONE, "None" }
};

void RENDER_CAM_REG_EDEXOPGAIN(void)  { CAM_REG_EDEXOPGAIN  = SHADOW.CAM_REG_EDEXOPGAIN  = ((SETTING(edge_exclusive)) ? CAM01F_EDGEEXCL_V_ON : CAM01F_EDGEEXCL_V_OFF) | edge_operations[SETTING(edge_operation)].value | gains[SETTING(current_gain)].value; }
void RENDER_CAM_REG_EXPTIME(void)     { CAM_REG_EXPTIME     = SHADOW.CAM_REG_EXPTIME     = swap_bytes(SETTING(current_exposure)); }
void RENDER_CAM_REG_EDRAINVVREF(void) { CAM_REG_EDRAINVVREF = SHADOW.CAM_REG_EDRAINVVREF = edge_ratios[SETTING(current_edge_ratio)].value | ((SETTING(invertOutput)) ? CAM04F_INV : CAM04F_POS) | voltage_refs[SETTING(current_voltage_ref)].value; }
void RENDER_CAM_REG_ZEROVOUT(void)    { CAM_REG_ZEROVOUT    = SHADOW.CAM_REG_ZEROVOUT    = zero_points[SETTING(current_zero_point)].value | TO_VOLTAGE_OUT(SETTING(voltage_out)); }
inline void RENDER_CAM_REG_DITHERPATTERN(void) { dither_pattern_apply(SETTING(dithering), SETTING(ditheringHighLight), SETTING(current_contrast) - 1); }

void RENDER_CAM_REGISTERS(void) {
    CAMERA_SWITCH_RAM(CAMERA_BANK_REGISTERS);
    RENDER_CAM_REG_EDEXOPGAIN();
    RENDER_CAM_REG_EXPTIME();
    RENDER_CAM_REG_EDRAINVVREF();
    RENDER_CAM_REG_ZEROVOUT();
    RENDER_CAM_REG_DITHERPATTERN();
}

/** Auto-exposure register bands -- the gain ladder.

    Exposure alone cannot cover the range of real scenes -- Pan Docs quotes about
    2000:1 between direct sunlight and a room lit only by a television -- and pushing
    exposure to the dark end wrecks the viewfinder frame rate, because frame time grows
    with it. So the analog gain, output reference voltage and edge operation move in
    steps as exposure travels, and each band pins them for one stretch of exposure.

    The five bands correspond one for one to the operating points listed below,
    including the detail that the lowest two share a gain and differ only in N.

    Two things about the shape. The stay ranges deliberately OVERLAP: a band is left
    upward at `stay_hi` but only left downward under `stay_lo`, roughly an octave lower.
    That overlap is hysteresis, and it is what stops the picture flapping between two
    gain settings on a scene sitting near a threshold -- every upward step doubles the
    gain, so without it the servo would immediately drive exposure back across the
    boundary it had just crossed.

    And the exposure is re-seeded to a fixed landing point on arrival rather than simply
    halved, so brightness is roughly preserved and the user never sees the picture jump.
    The landings are tuned rather than computed: `voltage_out` moves at every boundary
    too, so the brightness ratio across a handover is not purely the gain ratio. They
    come out near 1.4x rather than 2x, and they are what keeps every transition clear of
    the threshold it just crossed -- an exact halving at band 3 -> 4 would land within
    0.3% of band 4's exit and lose the hysteresis entirely.
*/
typedef struct autoexp_band_t {
    uint16_t stay_lo;           // drop to the band below when exposure falls under this
    uint16_t stay_hi;           // climb to the band above at or over this exposure
    uint16_t land_from_below;   // exposure to adopt when this band is entered climbing
    uint16_t land_from_above;   // exposure to adopt when this band is entered dropping
    int16_t  voltage_out;
    uint8_t  gain;              // index into gains[]
    uint8_t  edge_operation;    // index into edge_operations[]
    uint8_t  edge_exclusive;
} autoexp_band_t;

// Gain 14.0dB or 0 | vRef +64 mV | Horizontal edge mode | Exposure time range from  0.5ms to 0.3ms
// Gain 14.0dB or 0 | vRef +160 mV| 2-D edge mode        | Exposure time range from   67ms to 0.8ms
// Gain 20.0dB or 4 | vRef +96 mV | 2-D edge mode        | Exposure time range from  282ms to  32ms
// Gain 26.0dB or 8 | vRef -192 mV| 2-D edge mode        | Exposure time range from  573ms to 164ms
// Gain 32.0dB or 10| vRef -416 mV| No edge Operation    | Exposure time range from 1048ms to 394ms
//
// The low end of each range above is the corresponding stay_lo below, to the register
// value. Band 2's from_above landing is the one value not measured directly; it is
// interpolated from the 1.375x and 1.458x that the other two downward gain-halving
// handovers use.
static const autoexp_band_t autoexp_bands_slow[] = {
//    stay_lo  stay_hi  from_below  from_above    vout  gain  edge  excl
    { 0x0010,  0x0030,     0x0000,     0x001F,      64,    0,    1, false },
    { 0x0031,  0x1200,     0x0048,     0x0B00,     160,    0,    0, true  },
    { 0x0800,  0x6000,     0x0D80,     0x3800,      96,    4,    0, true  },
    { 0x2800,  0xC000,     0x3500,     0x8C00,    -192,    8,    0, true  },
    { 0x6000,  0xFFFF,     0x8500,     0x0000,    -416,   10,    3, false }
};
// At double speed the exposure register's time base halves, so every threshold and
// landing doubles. The top rung would then need a stay_hi past the end of a 16-bit
// register, so the gain 32.0dB band is unreachable and the ladder is four rungs.
static const autoexp_band_t autoexp_bands_fast[] = {
//    stay_lo  stay_hi  from_below  from_above    vout  gain  edge  excl
    { 0x0020,  0x0060,     0x0000,     0x003E,      64,    0,    1, false },
    { 0x0062,  0x2400,     0x0090,     0x1600,     160,    0,    0, true  },
    { 0x1000,  0xC000,     0x1B00,     0x7000,      96,    4,    0, true  },
    { 0x5000,  0xFFFF,     0x6A00,     0x0000,    -192,    8,    0, true  }
};

#define AUTOEXP_BAND_UNSET      0xFF
#define AUTOEXP_BAND_GAIN_HI    2       // the one band running at gain_hi; see autoexp_apply_band()
#define AUTOEXP_BANDS()         ((_is_CPU_FAST) ? autoexp_bands_fast : autoexp_bands_slow)
#define AUTOEXP_BANDS_LAST()    ((uint8_t)((_is_CPU_FAST) ? (LENGTH(autoexp_bands_fast) - 1) : (LENGTH(autoexp_bands_slow) - 1)))
#define AUTOEXP_EXPOSURE_MIN()  ((uint16_t)((_is_CPU_FAST) ? (EXPOSURE_LOW_LIMIT << 1) : EXPOSURE_LOW_LIMIT))

static uint8_t autoexp_band = AUTOEXP_BAND_UNSET;

void reset_autoexp_band(void) {
    autoexp_band = AUTOEXP_BAND_UNSET;
}

/** Pick the register band for `exposure`, one rung at a time and with hysteresis.

    Returns the exposure the new band wants to start from, or 0 when the band did not
    change. With no band selected yet the exposure is placed by the stay_hi thresholds
    alone and 0 is returned, since a cold selection is describing where the exposure
    already is rather than handing over from anywhere.
*/
static uint16_t autoexp_select_band(uint16_t exposure) {
    const autoexp_band_t * bands = AUTOEXP_BANDS();
    uint8_t last = AUTOEXP_BANDS_LAST();
    uint8_t band = autoexp_band;

    if (band > last) {
        for (band = 0; (band < last) && (exposure >= bands[band].stay_hi); band++);
        autoexp_band = band;
        return 0;
    }
    if ((band < last) && (exposure >= bands[band].stay_hi)) {
        autoexp_band = band + 1;
        return bands[band + 1].land_from_below;
    }
    if ((band != 0) && (exposure < bands[band].stay_lo)) {
        autoexp_band = band - 1;
        return bands[band - 1].land_from_above;
    }
    return 0;
}

// program every register the current band owns
static void autoexp_apply_band(void) {
    const autoexp_band_t * band = &AUTOEXP_BANDS()[autoexp_band];
    /* Which contrast curve each band gets is decided by the gain it runs at, not by the
       band index: every band takes dither_high_light_values except the one running at
       gain_hi, which takes dither_low_light_values. Bands 0 and 1 both run at gain_lo,
       which is why they share a curve. */
    switch_e highlight = (autoexp_band == AUTOEXP_BAND_GAIN_HI) ? set_off : set_on;
    bool apply_dither = (SETTING(ditheringHighLight) != highlight);

    SETTING(edge_exclusive)     = (band->edge_exclusive) ? set_on : set_off;
    SETTING(edge_operation)     = band->edge_operation;
    SETTING(voltage_out)        = band->voltage_out;
    SETTING(current_gain)       = band->gain;
    SETTING(ditheringHighLight) = highlight;

    /* Prefer values measured from this sensor over the table's. The table holds figures
       observed from one real camera, which is the best a constant can do -- but the right
       output bias and reference voltage depend on the individual part, so a measurement
       beats any constant. The two lowest bands share the measured low gain and the third
       takes the measured high one; the top two stay fixed at 26.0 and 32.0 dB. */
    if (camera_is_calibrated()) {
        if (autoexp_band <= 1)       SETTING(current_gain) = CALIBRATION_GAIN_LO();
        else if (autoexp_band == 2)  SETTING(current_gain) = CALIBRATION_GAIN_HI();
        SETTING(voltage_out)      = calibration_voltage_out_mv(camera_calibration.voltage_out[autoexp_band]);
        SETTING(current_voltage_ref) = camera_calibration.voltage_ref[autoexp_band];
    }

    CAMERA_SWITCH_RAM(CAMERA_BANK_REGISTERS);
    RENDER_CAM_REG_EDEXOPGAIN();
    RENDER_CAM_REG_EXPTIME();
    RENDER_CAM_REG_EDRAINVVREF();
    RENDER_CAM_REG_ZEROVOUT();
    if (apply_dither) RENDER_CAM_REG_DITHERPATTERN();
}

/** Snap the registers to whatever band the exposure falls in, ignoring hysteresis.

    This is the path taken when the user sets exposure by hand, where the exposure is
    an instruction rather than a measurement: it may jump several bands at once, and
    re-seeding it to preserve brightness would silently overwrite what was just asked
    for. So the band history is discarded and the exposure is left exactly as set.
*/
void RENDER_REGS_FROM_EXPOSURE(void) {
    autoexp_band = AUTOEXP_BAND_UNSET;
    autoexp_select_band(SETTING(current_exposure));
    autoexp_apply_band();
}

/** The servo path: step at most one band, and hand over on the band's landing point. */
void RENDER_REGS_FROM_EXPOSURE_SERVO(void) {
    uint8_t previous = autoexp_band;
    uint16_t landing = autoexp_select_band(SETTING(current_exposure));

    if (autoexp_band == previous) {
        // still inside the same band, so only the exposure register needs rewriting
        CAMERA_SWITCH_RAM(CAMERA_BANK_REGISTERS);
        RENDER_CAM_REG_EXPTIME();
        return;
    }
    // re-seed so brightness carries across the gain change rather than jumping
    if (landing) SETTING(current_exposure) = CONSTRAINT(landing, AUTOEXP_EXPOSURE_MIN(), EXPOSURE_HIGH_LIMIT);
    autoexp_apply_band();
}

bool image_captured(void) {
    CAMERA_SWITCH_RAM(CAMERA_BANK_REGISTERS);
    if (camera_PnR_delay) return false;
    uint8_t v = CAM_REG_CAPTURE;
    bool r = (((v ^ SHADOW.CAM_REG_CAPTURE) & CAM00F_CAPTURING) && !(v & CAM00F_CAPTURING));
    SHADOW.CAM_REG_CAPTURE = v;
    return r;
}
void image_capture(void) {
    CAMERA_SWITCH_RAM(CAMERA_BANK_REGISTERS);
    SHADOW.CAM_REG_CAPTURE = CAM_REG_CAPTURE = (CAM00F_POSITIVE | CAM00F_CAPTURING);
    switch (OPTION(after_action)) {
        case after_action_picnrec:
        case after_action_printpicnrec:
        case after_action_picnrec_video:
            set_image_refresh_dalay(PNR_DELAY_FRAMES);
            break;
        default:
            break;
    }
}

void display_last_seen(bool restore) {
    CAMERA_SWITCH_RAM(CAMERA_BANK_LAST_SEEN);
    uint8_t ypos = (OPTION(camera_mode) == camera_mode_manual) ? (IMAGE_DISPLAY_Y + 1) : IMAGE_DISPLAY_Y;
    screen_load_live_image(IMAGE_DISPLAY_X, ypos, CAMERA_IMAGE_TILE_WIDTH, CAMERA_IMAGE_TILE_HEIGHT,
                           OPTION(flip_live_view),
                           ((_is_COLOR) && OPTION(enable_DMA) && !((OPTION(after_action) == after_action_picnrec) || (OPTION(after_action) == after_action_printpicnrec) || (OPTION(after_action) == after_action_picnrec_video))));
    if (restore) screen_restore_rect(IMAGE_DISPLAY_X, ypos, CAMERA_IMAGE_TILE_WIDTH, CAMERA_IMAGE_TILE_HEIGHT);
}

void camera_scrollbars_reinit(void) {
    scrollbar_destroy_all();
    if (OPTION(camera_mode) == camera_mode_auto) {
        // init and set brightness scrollbar
        scrollbar_add(&ss_brightness, SS_BRIGHTNESS_X, SS_BRIGHTNESS_Y, SS_BRIGHTNESS_LEN, true);
        scrollbar_set_position(&ss_brightness, SETTING(current_brightness), 0, HISTOGRAM_MAX_VALUE);
        // init and set contrast scrollbar
        scrollbar_add(&ss_contrast, SS_CONTRAST_X, SS_CONTRAST_Y, SS_CONTRAST_LEN, false);
        scrollbar_set_position(&ss_contrast, SETTING(current_contrast), 1, NUM_CONTRAST_VALUES);
    }
}

bool camera_image_save(void) {
    static const uint8_t msgCameraRollFull[] = "Camera roll is full!";
    static image_metadata_t image_metadata;
    uint8_t n_images = images_taken();
    if (n_images < CAMERA_MAX_IMAGE_SLOTS) {
        // modify index
        uint8_t slot = VECTOR_POP(free_slots);
        protected_modify_slot(slot, n_images);
        // copy image data
        protected_lastseen_to_slot(slot, OPTION(flip_live_view));
        // generate thumbnail
        protected_generate_thumbnail(slot);
        // save metadata
        image_metadata.raw_regs = SHADOW;
        image_metadata.settings = current_settings[OPTION(camera_mode)];
        image_metadata.settings.cpu_fast = _is_CPU_FAST;
        image_metadata.crc = protected_calculate_crc((uint8_t *)&image_metadata.settings, sizeof(image_metadata.settings), PROTECTED_SEED);
        protected_metadata_write(slot, (uint8_t *)&image_metadata, sizeof(image_metadata));
        protected_image_owner_write(slot);
        // add slot to used list
        VECTOR_ADD(used_slots, slot);
        return true;
    } else {
        music_play_sfx(BANK(sound_error), sound_error, SFX_MUTE_MASK(sound_error), MUSIC_SFX_PRIORITY_HIGH);
        MessageBox(msgCameraRollFull);
        display_last_seen(true);
        return false;
    }
}

bool camera_image_save_sd(void) {
    static const uint8_t msgCameraSDError[] = "SD card error!";
    if (!lastseen_to_sd(OPTION(flip_live_view))) {
        music_play_sfx(BANK(sound_error), sound_error, SFX_MUTE_MASK(sound_error), MUSIC_SFX_PRIORITY_HIGH);
        MessageBox(msgCameraSDError);
        display_last_seen(true);
        return false;
    } else {
        return true;
    }
}

static void refresh_usage_indicator(void) {
    switch (OPTION(after_action)) {
        case after_action_picnrec_video:
        case after_action_transfer_video:
            if (recording_video) strcpy(text_buffer, ICON_REC); else *text_buffer = 0;
            break;
        default:
            sprintf(text_buffer, "%hd/%hd", (uint8_t)images_taken(), (uint8_t)images_total());
            break;
    }
    menu_text_out(HELP_CONTEXT_WIDTH, 17, IMAGE_SLOTS_USED_WIDTH, WHITE_ON_BLACK, ITEM_DEFAULT, text_buffer);
}

static void refresh_autoexp_area(void) {
    static const uint8_t * const area_indicators[N_AUTOEXP_AREAS] = {
        [area_center]  = "",             // no centre tile in the icon set yet
        [area_top]     = " " ICON_AUTOEXP_TOP,
        [area_right]   = " " ICON_AUTOEXP_RIGHT,
        [area_bottom]  = " " ICON_AUTOEXP_BOTTOM,
        [area_left]    = " " ICON_AUTOEXP_LEFT,
        [area_overall] = ""             // the default carries no marker
    };
    if (OPTION(camera_mode) != camera_mode_auto) return;
    menu_text_out(AUTOEXP_AREA_X, AUTOEXP_AREA_Y, 0, WHITE_ON_BLACK, ITEM_DEFAULT, area_indicators[OPTION(autoexp_area)]);
}

static void refresh_screen(void) {
    screen_clear_rect(0, 0, DEVICE_SCREEN_WIDTH, DEVICE_SCREEN_HEIGHT, WHITE_ON_BLACK);
    display_last_seen(true);
    refresh_usage_indicator();
    refresh_autoexp_area();
    scrollbar_repaint_all();
}

static uint8_t onPrinterProgress(void) BANKED {
    misc_render_progressbar(printer_completion, PRN_MAX_PROGRESS, text_buffer);
    menu_text_out(0, 0 + 17, HELP_CONTEXT_WIDTH, WHITE_ON_BLACK, ITEM_DEFAULT, text_buffer);
    return 0;
}

const metasprite_t grid_metasprite[] = {
    METASPR_ITEM(     -4,  -4, 0, 0), METASPR_ITEM(35, 43, 0, 0), METASPR_ITEM(0, 43, 0, 0), METASPR_ITEM(-35, 43, 0, 0),
    METASPR_ITEM(35 + 43, -43, 0, 0), METASPR_ITEM(35, 43, 0, 0), METASPR_ITEM(-35, -43 - 43, 0, 0), METASPR_ITEM(35, -43, 0, 0),
    METASPR_TERM
};
uint8_t grid_render(uint8_t hw) {
    if (OPTION(show_grid)) {
        return (hw + move_metasprite(grid_metasprite, (0x80 - cursors_TILE_COUNT), hw,
                                     DEVICE_SPRITE_PX_OFFSET_X + 16,
                                     ((OPTION(camera_mode) == camera_mode_manual) ? 8 : 0) + DEVICE_SPRITE_PX_OFFSET_Y + 16));
    }
    return hw;
}


static uint8_t vbl_frames_counter = 0;

inline void camera_charge_timer(uint8_t value) {
    COUNTER_SET(camera_shutter_timer, value);
    vbl_frames_counter = 60;
}

void shutter_VBL_ISR(void) NONBANKED {
    if (!vbl_frames_counter--) {
        vbl_frames_counter = 60;
        if (COUNTER(camera_shutter_timer)) {
            if (!--COUNTER(camera_shutter_timer)) camera_do_shutter = true;
        }
    }
    if (camera_PnR_delay) camera_PnR_delay--;
}

void reset_AEB(void) {
    if (AEB_capture_in_progress) {
        AEB_capture_in_progress = false;
        COUNTER_RESET(camera_AEB_counter);
        SETTING(current_exposure) = last_AEB_exposure;
        // if AEB capture process was cancelled, then restore exposure
        CAMERA_SWITCH_RAM(CAMERA_BANK_REGISTERS);
        RENDER_CAM_REG_EXPTIME();
    }
}
void reset_shutter(void) {
    reset_AEB();
    // cancel timers and counters
    COUNTER_RESET(camera_shutter_timer);
    screen_clear_rect(SHUTTER_TIMER_X, SHUTTER_TIMER_Y, 2, 2, WHITE_ON_BLACK);
    COUNTER_RESET(camera_repeat_counter);
    screen_clear_rect(SHUTTER_REPEAT_X, SHUTTER_REPEAT_Y, 2, 2, WHITE_ON_BLACK);
}

uint8_t INIT_state_camera(void) BANKED {
    CRITICAL {
        add_VBL(shutter_VBL_ISR);
    }
    return 0;
}

uint8_t ENTER_state_camera(void) BANKED {
    // scrollbars
    camera_scrollbars_reinit();
    // repaint screen
    refresh_screen();
    // set printer progress handler
    gbprinter_set_handler(onPrinterProgress, BANK(state_camera));
    // On CGB, start sensing IR
    if ((_is_COLOR) && (OPTION(ir_remote_shutter))) ir_sense_start();
    // reset capture timers and counters
    COUNTER_RESET(camera_shutter_timer);
    COUNTER_RESET(camera_AEB_counter);
    COUNTER_RESET(camera_repeat_counter);
    // keep each mode's ladder index in step with its stored exposure
    for (uint8_t m = 0; m != N_CAMERA_MODES; m++)
        current_settings[m].current_exposure_idx = exposure_index_for(current_settings[m].current_exposure);
    // the band ladder's hysteresis is only meaningful against a band this session
    // actually selected, so start it from the exposure that is about to be loaded
    reset_autoexp_band();
    // measure this sensor once and keep the result, rather than repeating it every boot
    if (!camera_calibration_is_current()) {
        camera_calibrate();
        if (camera_is_calibrated()) save_camera_calibration();
    }
    // load some initial settings
    RENDER_CAM_REGISTERS();
    SHADOW.CAM_REG_CAPTURE = 0;
    // fade in
    fade_in_modal();
    return 0;
}

// callback forward declarations
uint8_t onTranslateKeyCameraMenu(const struct menu_t * menu, const struct menu_item_t * self, uint8_t value);
uint8_t onIdleCameraMenu(const struct menu_t * menu, const struct menu_item_t * selection);
uint8_t * onCameraMenuItemPaint(const struct menu_t * menu, const struct menu_item_t * self);
uint8_t onHelpCameraMenu(const struct menu_t * menu, const struct menu_item_t * selection);
uint8_t * formatItemText(camera_menu_e id, const uint8_t * format, camera_mode_settings_t * settings, bool divide_exposure);

// --- Save confirmation namu ------------------------
uint8_t onSaveConfirmMenuItemProps(const struct menu_t * menu, const struct menu_item_t * self);
const menu_item_t SaveConfirmMenuItems[] = {
    {
        .sub = NULL, .sub_params = NULL,
        .ofs_x = 0, .ofs_y = 0, .width = 7,
        .caption = "Save image",
        .onPaint = NULL,
        .onGetProps = onSaveConfirmMenuItemProps,
        .result = MENU_RESULT_YES
    }, {
        .sub = NULL, .sub_params = NULL,
        .ofs_x = 7, .ofs_y = 0, .width = 7,
        .caption = ICON_B " Discard",
        .onPaint = NULL,
        .onGetProps = onSaveConfirmMenuItemProps,
        .result = MENU_RESULT_NO
    }
};
const menu_t SaveConfirmMenu = {
    .x = 2, .y = 17, .width = 0, .height = 0,
    .flags = MENU_FLAGS_INVERSE,
    .cancel_mask = J_B, .cancel_result = MENU_RESULT_NO,
    .items = SaveConfirmMenuItems, .last_item = LAST_ITEM(SaveConfirmMenuItems),
    .onTranslateKey = onTranslateKeyCameraMenu, .onTranslateSubResult = NULL
};
uint8_t onSaveConfirmMenuItemProps(const struct menu_t * menu, const struct menu_item_t * self) {
    menu; self;
    return ITEM_TEXT_CENTERED;
}

// --- Assisted menu ---------------------------------
const menu_item_t CameraMenuItemsAssisted[] = {
    {
        .sub = NULL, .sub_params = NULL,
        .ofs_x = 0, .ofs_y = 0, .width = 5,
        .id = idExposure,
        .caption = " %sms",
        .helpcontext = " Exposure time",
        .onPaint = onCameraMenuItemPaint,
        .result = MENU_RESULT_NONE
    }, {
        .sub = NULL, .sub_params = NULL,
        .ofs_x = 5, .ofs_y = 0, .width = 5,
        .id = idContrast,
        .caption = " " ICON_CONTRAST "\t%d",
        .helpcontext = " Contrast level",
        .onPaint = onCameraMenuItemPaint,
        .result = MENU_RESULT_NONE
    }, {
        .sub = NULL, .sub_params = NULL,
        .ofs_x = 10, .ofs_y = 0, .width = 5,
        .id = idDither,
        .caption = " " ICON_DITHER "\t%s",
        .helpcontext = " Dithering pattern",
        .onPaint = onCameraMenuItemPaint,
        .result = MENU_RESULT_NONE
    }, {
        // ToDo: remove this menu option when it's being set automatically via `.id = idExposure`
        .sub = NULL, .sub_params = NULL,
        .ofs_x = 15, .ofs_y = 0, .width = 5,
        .id = idDitherLight,
        .caption = " " ICON_DITHER "\t%s",
        .helpcontext = " Dithering light level",
        .onPaint = onCameraMenuItemPaint,
        .result = MENU_RESULT_NONE
    }
};
const menu_t CameraMenuAssisted = {
    .x = 0, .y = 0, .width = 0, .height = 0,
    .flags = MENU_FLAGS_INVERSE,
    .items = CameraMenuItemsAssisted, .last_item = LAST_ITEM(CameraMenuItemsAssisted),
    .onShow = NULL, .onIdle = onIdleCameraMenu, .onHelpContext = onHelpCameraMenu,
    .onTranslateKey = onTranslateKeyCameraMenu, .onTranslateSubResult = NULL
};

// --- Auto menu -------------------------------------
const menu_item_t CameraMenuItemsAuto[] = {
    {
        .sub = NULL, .sub_params = NULL,
        .ofs_x = 0, .ofs_y = 0, .width = 0,
        .id = idNone,
        .caption = " Automatic mode",
        .helpcontext = " D-Pad adjusts " ICON_BRIGHTNESS " and " ICON_CONTRAST,
        .onPaint = onCameraMenuItemPaint,
        .result = MENU_RESULT_NONE
    }
};
const menu_t CameraMenuAuto = {
    .x = 0, .y = 0, .width = 0, .height = 0,
    .flags = 0,
    .items = CameraMenuItemsAuto, .last_item = LAST_ITEM(CameraMenuItemsAuto),
    .onShow = NULL, .onIdle = onIdleCameraMenu, .onHelpContext = onHelpCameraMenu,
    .onTranslateKey = onTranslateKeyCameraMenu, .onTranslateSubResult = NULL
};

// --- Manual menu -----------------------------------
const menu_item_t CameraMenuItemsManual[] = {
    {
        .sub = NULL, .sub_params = NULL,
        .ofs_x = 0, .ofs_y = 0, .width = 5,
        .id = idExposure,
        .caption = " %sms",
        .helpcontext = " Exposure time",
        .onPaint = onCameraMenuItemPaint,
        .result = MENU_RESULT_NONE
    }, {
        .sub = NULL, .sub_params = NULL,
        .ofs_x = 5, .ofs_y = 0, .width = 5,
        .id = idGain,
        .caption = " " ICON_GAIN "\t%s",
        .helpcontext = " Sensor gain",
        .onPaint = onCameraMenuItemPaint,
        .result = MENU_RESULT_NONE
    }, {
        .sub = NULL, .sub_params = NULL,
        .ofs_x = 10, .ofs_y = 0, .width = 5,
        .id = idDither,
        .caption = " " ICON_DITHER "\t%s",
        .helpcontext = " Dithering pattern",
        .onPaint = onCameraMenuItemPaint,
        .result = MENU_RESULT_NONE
    }, {
        .sub = NULL, .sub_params = NULL,
        .ofs_x = 15, .ofs_y = 0, .width = 5,
        .id = idDitherLight,
        .caption = " " ICON_DITHER "\t%s",
        .helpcontext = " Dithering light level",
        .onPaint = onCameraMenuItemPaint,
        .result = MENU_RESULT_NONE
    }, {
        .sub = NULL, .sub_params = NULL,
        .ofs_x = 0, .ofs_y = 1, .width = 5,
        .id = idContrast,
        .caption = " " ICON_CONTRAST "\t%d",
        .helpcontext = " Contrast level",
        .onPaint = onCameraMenuItemPaint,
        .result = MENU_RESULT_NONE
    }, {
        .sub = NULL, .sub_params = NULL,
        .ofs_x = 5, .ofs_y = 1, .width = 5,
        .id = idZeroPoint,
        .caption = " %s",
        .helpcontext = " Sensor zero point",
        .onPaint = onCameraMenuItemPaint,
        .result = MENU_RESULT_NONE
    }, {
        .sub = NULL, .sub_params = NULL,
        .ofs_x = 10, .ofs_y = 1, .width = 5,
        .id = idVOut,
        .caption = " %dmv",
        .helpcontext = " Sensor voltage out",
        .onPaint = onCameraMenuItemPaint,
        .result = MENU_RESULT_NONE
    }, {
        .sub = NULL, .sub_params = NULL,
        .ofs_x = 15, .ofs_y = 1, .width = 5,
        .id = idVoltageRef,
        .caption = " " ICON_VOLTAGE "\t%sv",
        .helpcontext = " Sensor voltage reference",
        .onPaint = onCameraMenuItemPaint,
        .result = MENU_RESULT_NONE
    }, {
        .sub = NULL, .sub_params = NULL,
        .ofs_x = 0, .ofs_y = 2, .width = 5,
        .id = idInvOutput,
        .caption = " %s",
        .helpcontext = " Inverse output",
        .onPaint = onCameraMenuItemPaint,
        .result = MENU_RESULT_NONE
    }, {
        .sub = NULL, .sub_params = NULL,
        .ofs_x = 5, .ofs_y = 2, .width = 5,
        .id = idEdgeOperation,
        .caption = " " ICON_EDGE "\t%s",
        .helpcontext = " Sensor edge operation",
        .onPaint = onCameraMenuItemPaint,
        .result = MENU_RESULT_NONE
    }, {
        .sub = NULL, .sub_params = NULL,
        .ofs_x = 10, .ofs_y = 2, .width = 5,
        .id = idEdgeRatio,
        .caption = " " ICON_EDGE "\t%s",
        .helpcontext = " Sensor edge ratio",
        .onPaint = onCameraMenuItemPaint,
        .result = MENU_RESULT_NONE
    }, {
        .sub = NULL, .sub_params = NULL,
        .ofs_x = 15, .ofs_y = 2, .width = 5,
        .id = idEdgeExclusive,
        .caption = " " ICON_EDGE "\t%s",
        .helpcontext = "Sensor edge exclusive",
        .onPaint = onCameraMenuItemPaint,
        .result = MENU_RESULT_NONE
    }
};
const menu_t CameraMenuManual = {
    .x = 0, .y = 0, .width = 0, .height = 0,
    .flags = MENU_FLAGS_INVERSE,
    .items = CameraMenuItemsManual, .last_item = LAST_ITEM(CameraMenuItemsManual),
    .onShow = NULL, .onIdle = onIdleCameraMenu, .onHelpContext = onHelpCameraMenu,
    .onTranslateKey = onTranslateKeyCameraMenu, .onTranslateSubResult = NULL
};
static const menu_item_t * last_menu_items[N_CAMERA_MODES] = { NULL, NULL, NULL };
uint8_t onTranslateKeyCameraMenu(const struct menu_t * menu, const struct menu_item_t * self, uint8_t value) {
    menu; self;
    // swap J_UP/J_DOWN with J_LEFT/J_RIGHT buttons, because our menus are horizontal
    return joypad_swap_dpad(value);
}
bool isSaveCancelled(void) {
    screen_clear_rect(0, 17, HELP_CONTEXT_WIDTH, 1, WHITE_ON_BLACK);
    uint8_t menu_result = menu_execute(&SaveConfirmMenu, NULL, NULL);
    return (menu_result != MENU_RESULT_YES);
}
uint8_t onIdleCameraMenu(const struct menu_t * menu, const struct menu_item_t * selection) {
    static const shutter_sound_t shutter_sounds[N_SHUTTER_SOUNDS] = {
        [shutter_sound_0] = {BANK(shutter01), shutter01, SFX_MUTE_MASK(shutter01)},
        [shutter_sound_1] = {BANK(shutter02), shutter02, SFX_MUTE_MASK(shutter02)}
    };
    static change_direction_e change_direction;
    static bool capture_triggered = false;       // state of static variable persists between calls
    static bool render_registers;

    // If enabled, sense remote shutters. IR sensing takes time but only if initially high
    static bool remote_shutter_triggered;
    remote_shutter_triggered = ((_is_COLOR) && OPTION(ir_remote_shutter) && !capture_triggered && ir_sense_pattern());

    // save current selection
    last_menu_items[OPTION(camera_mode)] = selection;
    // process joypad buttons

// not enough buttons on the GG for the "Brightness/Contrast reset" and "AutoExp once" features
#if defined(NINTENDO)
    if (KEY_PRESSED(J_START)) {
        if (OPTION(camera_mode) == camera_mode_auto) {
            PLAY_SFX(sound_menu_alter);
            // reset both brightness and contrast to defaults, adjust the sliders
            scrollbar_set_position(&ss_brightness, (SETTING(current_brightness) = HISTOGRAM_TARGET_VALUE), 0, HISTOGRAM_MAX_VALUE);
            scrollbar_set_position(&ss_contrast, (SETTING(current_contrast) = DEFAULT_CONTRAST_VALUE), 1, NUM_CONTRAST_VALUES);
            save_camera_mode_settings(OPTION(camera_mode));
            // change of contrast means reloading of the dithering pattern
            CAMERA_SWITCH_RAM(CAMERA_BANK_REGISTERS);
            RENDER_CAM_REG_DITHERPATTERN();
        } else {
#ifdef ENABLE_AUTOEXP
            // perform one step of autoexposure if J_START is held in manual or assisted mode
            one_iteration_autoexp = true;
#endif
        }
    } else
#endif
    if (KEY_PRESSED(J_A) || remote_shutter_triggered) {
        // A is a "shutter" button
        switch (OPTION(after_action)) {
            case after_action_picnrec_video:
            case after_action_transfer_video:
                // toggle recording and start image capture
                recording_video = !recording_video;
                if (recording_video && !image_is_capturing()) image_capture();
                refresh_usage_indicator();
                break;
            default:
                switch (OPTION(trigger_mode)) {
                    case trigger_mode_repeat:
                        COUNTER_SET(camera_repeat_counter, OPTION(shutter_counter));
                    case trigger_mode_timer:
                        camera_charge_timer(OPTION(shutter_timer));
                        COUNTER_RESET(camera_AEB_counter);
                        break;
                    case trigger_mode_AEB: {
                            if (AEB_capture_in_progress) break;
                            AEB_capture_in_progress = true;
                            uint8_t aeb_over_counter = MIN(OPTION(aeb_overexp_count), MAX_AEB_OVEREXPOSURE);
                            uint8_t aeb_shift = (OPTION(aeb_overexp_step) & 0x01) + 3;
                            COUNTER_SET(camera_AEB_counter, (aeb_over_counter << 1) + 1);
                            // exposure mid point which is also the last_AEB_exposure
                            AEB_exposure_list[MIDDLE_AEB_IMAGE] = SETTING(current_exposure);
                            // under-exposure in i steps
                            for (uint8_t i = MIDDLE_AEB_IMAGE; i != (MIDDLE_AEB_IMAGE - aeb_over_counter); i--) {
                                AEB_exposure_list[i - 1] = CONSTRAINT((int32_t)AEB_exposure_list[i] - (AEB_exposure_list[i] >> aeb_shift), (_is_CPU_FAST) ? (EXPOSURE_LOW_LIMIT << 1) : EXPOSURE_LOW_LIMIT, EXPOSURE_HIGH_LIMIT);
                            }
                            // over-exposure in i steps
                            for (uint8_t i = MIDDLE_AEB_IMAGE; i != (MIDDLE_AEB_IMAGE + aeb_over_counter); i++) {
                                AEB_exposure_list[i + 1] = CONSTRAINT((int32_t)AEB_exposure_list[i] + (AEB_exposure_list[i] >> aeb_shift), (_is_CPU_FAST) ? (EXPOSURE_LOW_LIMIT << 1) : EXPOSURE_LOW_LIMIT, EXPOSURE_HIGH_LIMIT);
                            }
                            break;
                        }
                    default:
                        camera_do_shutter = true;
                        break;
                }
                break;
        }
    } else if (KEY_PRESSED(J_B)) {
        if (COUNTER(camera_shutter_timer) || COUNTER(camera_repeat_counter) || COUNTER(camera_AEB_counter)) {
            reset_shutter();
            camera_do_shutter = capture_triggered = false;
        } else {
            // open the main menu
            capture_triggered = false;
            return ACTION_MAIN_MENU;
        }
    } else if (KEY_PRESSED(J_POPUP_MENU)) {
        // select opens popup-menu
        capture_triggered = false;
        return ACTION_CAMERA_SUBMENU;
    }

    static uint8_t selection_item_id;
    selection_item_id = selection->id;

    // !!! d-pad keys are translated
    if (!COUNTER(camera_AEB_counter)) {
        if (OPTION(camera_mode) == camera_mode_auto) {
            // in automatic mode menu items are "synthetic"
            if (KEY_PRESSED(J_RIGHT))       change_direction = changeIncrease, selection_item_id = idBrightness;
            else if (KEY_PRESSED(J_LEFT))   change_direction = changeDecrease, selection_item_id = idBrightness;
            else if (KEY_PRESSED(J_DOWN))   change_direction = changeIncrease, selection_item_id = idContrast;
            else if (KEY_PRESSED(J_UP))     change_direction = changeDecrease, selection_item_id = idContrast;
            else change_direction = changeNone;
        } else {
            if (KEY_PRESSED(J_RIGHT))       change_direction = changeDecrease;
            else if (KEY_PRESSED(J_LEFT))   change_direction = changeIncrease;
            else change_direction = changeNone;
        }
    } else change_direction = changeNone;                   // disable menu when capturing AEB

    CAMERA_SWITCH_RAM(CAMERA_BANK_REGISTERS);
    if (change_direction != changeNone) {
        static uint8_t temp_uint8;
        static bool settings_changed, redraw_selection;
        redraw_selection = settings_changed = true;
        // perform changes when pressing UP/DOWN while menu item with some ID is active
        switch (selection_item_id) {
            case idExposure:
                /* Accelerate while the key stays down. A tenth of a stop is deliberately
                   fine for nudging, which would make crossing the range tedious at a fixed
                   rate, so the step grows the longer the ramp runs and resets the moment
                   the key comes up. Assisted mode starts one notch coarser, as before. */
                if (settings_changed = inc_dec_uint8(&SETTING(current_exposure_idx), exposure_ramp_step(change_direction), 0, MAX_INDEX(exposures), change_direction)) {
                    SETTING(current_exposure) = exposures[SETTING(current_exposure_idx)];
                    switch (OPTION(camera_mode)) {
                        case camera_mode_assisted:
                            // ToDo: Adjust other registers ("N", Output Ref Voltage) based on index of 'current_exposure_idx'
                            RENDER_REGS_FROM_EXPOSURE();    // Voltage Out, Gain, Edge Operation, DitherLight
                            break;
                        default:
                            RENDER_CAM_REG_EXPTIME();
                            break;
                    }
                }
                break;
            case idGain:
                if (settings_changed = inc_dec_int8(&SETTING(current_gain), 1, 0, MAX_INDEX(gains), change_direction)) RENDER_CAM_REG_EDEXOPGAIN();
                break;
            case idVOut:
                if (settings_changed = inc_dec_int16(&SETTING(voltage_out), VOLTAGE_OUT_STEP * menu_ramp_multiplier(idVOut, change_direction), MIN_VOLTAGE_OUT, MAX_VOLTAGE_OUT, change_direction)) RENDER_CAM_REG_ZEROVOUT();
                break;
            case idDither:
                temp_uint8 = SETTING(dithering);
                if (settings_changed = inc_dec_int8(&temp_uint8, 1, 0, N_DITHER_TYPES - 1, change_direction)) {
                    SETTING(dithering) = temp_uint8;
                    RENDER_CAM_REG_DITHERPATTERN();
                }
                break;
            case idDitherLight:
                SETTING(ditheringHighLight) = !SETTING(ditheringHighLight);
                RENDER_CAM_REG_DITHERPATTERN();
                break;
            case idContrast:
                if (settings_changed = inc_dec_int8(&SETTING(current_contrast), 1, 1, NUM_CONTRAST_VALUES, change_direction)) {
                    RENDER_CAM_REG_DITHERPATTERN();
                    scrollbar_set_position(&ss_contrast, SETTING(current_contrast), 1, NUM_CONTRAST_VALUES);
                    redraw_selection = (OPTION(camera_mode) != camera_mode_auto);
                }
                break;
            case idInvOutput:
                SETTING(invertOutput) = !SETTING(invertOutput);
                RENDER_CAM_REG_EDRAINVVREF();
                break;
            case idZeroPoint:
                if (settings_changed = inc_dec_int8(&SETTING(current_zero_point), 1, 0, MAX_INDEX(zero_points), change_direction)) RENDER_CAM_REG_ZEROVOUT();
                break;
            case idVoltageRef:
                if (settings_changed = inc_dec_int8(&SETTING(current_voltage_ref), 1, 0, MAX_INDEX(voltage_refs), change_direction)) RENDER_CAM_REG_EDRAINVVREF();
                break;
            case idEdgeRatio:
                if (settings_changed = inc_dec_int8(&SETTING(current_edge_ratio), 1, 0, MAX_INDEX(edge_ratios), change_direction)) RENDER_CAM_REG_EDRAINVVREF();
                break;
            case idEdgeExclusive:
                SETTING(edge_exclusive) = !SETTING(edge_exclusive);
                RENDER_CAM_REG_EDEXOPGAIN();
                break;
            case idEdgeOperation:
                if (settings_changed = inc_dec_int8(&SETTING(edge_operation), 1, 0, MAX_INDEX(edge_operations), change_direction)) RENDER_CAM_REG_EDEXOPGAIN();
                break;
            case idBrightness:
                if (settings_changed = inc_dec_int16(&SETTING(current_brightness), (HISTOGRAM_MAX_VALUE >> 5), 0, HISTOGRAM_MAX_VALUE, change_direction)) {
                    scrollbar_set_position(&ss_brightness, SETTING(current_brightness), 0, HISTOGRAM_MAX_VALUE);
                    redraw_selection = (OPTION(camera_mode) != camera_mode_auto);
                }
                break;
            default:
                settings_changed = false;
                break;
        }
        // redraw selection if requested
        if (settings_changed) {
            PLAY_SFX(sound_menu_alter);
            save_camera_mode_settings(OPTION(camera_mode));
            if (redraw_selection) menu_move_selection(menu, NULL, selection);
        }
    }

    // process the timer
    if (COUNTER_CHANGED(camera_shutter_timer)) {
        if (camera_shutter_timer) {
            PLAY_SFX(sound_timer);
            menu_text_out(SHUTTER_TIMER_X, SHUTTER_TIMER_Y, 0, WHITE_ON_BLACK, ITEM_DEFAULT, " " ICON_CLOCK);
            sprintf(text_buffer, " %hd", (uint8_t)COUNTER(camera_shutter_timer));
            menu_text_out(SHUTTER_TIMER_X, SHUTTER_TIMER_Y + 1, 2, WHITE_ON_BLACK, ITEM_DEFAULT, text_buffer);
        } else {
            screen_clear_rect(SHUTTER_TIMER_X, SHUTTER_TIMER_Y, 2, 2, WHITE_ON_BLACK);
            if (COUNTER(camera_repeat_counter)) {
                if (--COUNTER(camera_repeat_counter)) camera_charge_timer(OPTION(shutter_timer));
                if (OPTION(shutter_counter) == COUNTER_INFINITE_VALUE) COUNTER_SET(camera_repeat_counter, COUNTER_INFINITE_VALUE);
            }
        }
    }

    // process the repeat counter
    if (COUNTER_CHANGED(camera_repeat_counter)) {
        if (COUNTER(camera_repeat_counter)) {
            menu_text_out(SHUTTER_REPEAT_X, SHUTTER_REPEAT_Y, 0, WHITE_ON_BLACK, ITEM_DEFAULT, " " ICON_MULTIPLE);
            if (OPTION(shutter_counter) == COUNTER_INFINITE_VALUE) {
                menu_text_out(SHUTTER_REPEAT_X, SHUTTER_REPEAT_Y + 1, 2, WHITE_ON_BLACK, ITEM_DEFAULT, " Inf");
            } else {
                sprintf(text_buffer, " %hd", (uint8_t)COUNTER(camera_repeat_counter));
                menu_text_out(SHUTTER_REPEAT_X, SHUTTER_REPEAT_Y + 1, 2, WHITE_ON_BLACK, ITEM_DEFAULT, text_buffer);
            }
        } else screen_clear_rect(SHUTTER_REPEAT_X, SHUTTER_REPEAT_Y, 2, 2, WHITE_ON_BLACK);
    }

    // make the picture if not in progress yet
    if (camera_do_shutter) {
        if (!capture_triggered) {
            music_play_sfx(shutter_sounds[OPTION(shutter_sound)].bank, shutter_sounds[OPTION(shutter_sound)].sound, shutter_sounds[OPTION(shutter_sound)].mask, MUSIC_SFX_PRIORITY_NORMAL);
            if (!image_is_capturing()) image_capture();
            capture_triggered = true;
        }
        camera_do_shutter = false;
    }

    // check image was captured, if yes, then restart capturing process
    if (image_captured()) {
        switch (OPTION(after_action)) {
            case after_action_picnrec:
            case after_action_printpicnrec:
                if (capture_triggered) picnrec_trigger();
                break;
            case after_action_picnrec_video:
                if (recording_video) picnrec_trigger();
                break;
            case after_action_transfer_video:
                if (recording_video) {
                    remote_activate(REMOTE_DISABLED);
                    linkcable_transfer_reset();
                    linkcable_transfer_image(get_flipped_last_seen_image(OPTION(flip_live_view), false), CAMERA_BANK_LAST_SEEN);
                    remote_activate(REMOTE_ENABLED);
                }
                break;
            default:
                break;
        }
        display_last_seen(false);
        if (capture_triggered) {
            capture_triggered = false;
            // check save confirmation
            if (OPTION(save_confirm) && (OPTION(trigger_mode) != trigger_mode_repeat) && (OPTION(trigger_mode) != trigger_mode_AEB)) {
                if ((OPTION(after_action) == after_action_save) ||
                    (OPTION(after_action) == after_action_print) ||
                    (OPTION(after_action) == after_action_printsave) ||
                    (OPTION(after_action) == after_action_transfer) ||
                    (OPTION(after_action) == after_action_transfersave) ||
                    (OPTION(after_action) == after_action_printpicnrec) ||
                    (OPTION(after_action) == after_action_savesd)) {
                        if (isSaveCancelled()) return ACTION_NONE;
                        onHelpCameraMenu(menu, selection);
                }
            }
            // perform after action(s)
            switch (OPTION(after_action)) {
                case after_action_save:
                    if (!camera_image_save()) {
                        reset_shutter();
                        camera_do_shutter = capture_triggered = false;
                    } else refresh_usage_indicator();
                    break;
                case after_action_printsave:
                    if (!camera_image_save()) {
                        reset_shutter();
                        camera_do_shutter = capture_triggered = false;
                    } else refresh_usage_indicator();
                case after_action_print:
                case after_action_printpicnrec:
                    return ACTION_CAMERA_PRINT;
                case after_action_transfersave:
                    if (!camera_image_save()) {
                        reset_shutter();
                        camera_do_shutter = capture_triggered = false;
                    } else refresh_usage_indicator();
                case after_action_transfer:
                    return ACTION_CAMERA_TRANSFER;
                case after_action_savesd:
                    if (!camera_image_save_sd()) {
                        reset_shutter();
                        camera_do_shutter = capture_triggered = false;
                    }
                    break;
                default:
                    break;
            }
        }

        // process AEB counter
        if (COUNTER_CHANGED(camera_AEB_counter)) {
            if (COUNTER(camera_AEB_counter)) {
                menu_text_out(SHUTTER_REPEAT_X, SHUTTER_REPEAT_Y, 0, WHITE_ON_BLACK, ITEM_DEFAULT, " " ICON_MULTIPLE);
                sprintf(text_buffer, " %hd", (uint8_t)COUNTER(camera_AEB_counter));
                menu_text_out(SHUTTER_REPEAT_X, SHUTTER_REPEAT_Y + 1, 2, WHITE_ON_BLACK, ITEM_DEFAULT, text_buffer);
                camera_do_shutter = true;
                // set new calculated exposure here instead of this:
                SETTING(current_exposure) = AEB_exposure_list[--COUNTER(camera_AEB_counter) + (MIDDLE_AEB_IMAGE - MIN(OPTION(aeb_overexp_count), MAX_AEB_OVEREXPOSURE))];
            } else {
                screen_clear_rect(SHUTTER_REPEAT_X, SHUTTER_REPEAT_Y, 2, 2, WHITE_ON_BLACK);
                SETTING(current_exposure) = last_AEB_exposure;
                AEB_capture_in_progress = false;
            }
            CAMERA_SWITCH_RAM(CAMERA_BANK_REGISTERS);
            RENDER_CAM_REG_EXPTIME();
        }
#ifdef ENABLE_AUTOEXP
        else if ((one_iteration_autoexp) || (OPTION(camera_mode) == camera_mode_auto)) {
            uint16_t measured = calculate_histogram(OPTION(autoexp_area));
            CAMERA_SWITCH_RAM(CAMERA_BANK_REGISTERS);  // restore register bank after histogram calculating

            // The servo drives measured/target towards AUTOEXP_RATIO_SETPOINT, so the
            // user's setpoint enters as the divisor -- which is exactly the byte the
            // servo divides by. Raising it lowers the ratio and shortens exposure, the
            // same direction the old difference metric ran in.
            uint8_t target = SETTING(current_brightness) / AUTOEXP_RATIO_SETPOINT;
            uint8_t ratio = autoexp_ratio(measured, target);
            uint8_t shift = autoexp_shift_table[ratio];

            uint16_t current_exposure = SETTING(current_exposure);
            uint16_t result_exposure = current_exposure;

            if (shift < AUTOEXP_NO_CORRECTION) {
                uint16_t step = current_exposure >> shift;
                // At small exposures the shift can take the step to zero. The bottom band
                // has no rung below it to escape to, so a floor of 1 keeps the servo from
                // stalling with a standing error. Everywhere the shift produces a step at
                // all this is a no-op.
                if (!step) step = 1;
                if (ratio >= AUTOEXP_RATIO_SETPOINT) {
                    // reads too dark -> lengthen exposure, saturating rather than wrapping
                    uint16_t lengthened = current_exposure + step;
                    result_exposure = (lengthened < current_exposure) ? EXPOSURE_HIGH_LIMIT : lengthened;
                } else {
                    // reads too bright -> shorten exposure
                    result_exposure = (step < current_exposure) ? (current_exposure - step) : 0;
                }
                result_exposure = CONSTRAINT(result_exposure, AUTOEXP_EXPOSURE_MIN(), EXPOSURE_HIGH_LIMIT);
            }

            if (result_exposure != SETTING(current_exposure)) {
                SETTING(current_exposure) = result_exposure;
                render_registers = true;
                if ((!one_iteration_autoexp) && (OPTION(display_exposure))) menu_text_out(14, 0, 6, WHITE_ON_BLACK, ITEM_DEFAULT, formatItemText(idExposure, "%sms", &CURRENT_SETTINGS, _is_CPU_FAST));
            } else render_registers = false;

            // not enough buttons on the GG for the "AutoExp once" feature
            #if defined(NINTENDO)
            if ((one_iteration_autoexp) && ((JOYPAD_LAST() & J_START) == 0)) {
                one_iteration_autoexp = false;
                // put the ladder index back in step with the exposure the servo settled on
                SETTING(current_exposure_idx) = exposure_index_for(SETTING(current_exposure));
                SETTING(current_exposure) = exposures[SETTING(current_exposure_idx)];
                // snapping to a ladder entry can cross several bands at once, so let the
                // ladder reselect from scratch instead of walking one rung at a time
                reset_autoexp_band();
                render_registers = true;
                // redraw menu
                PLAY_SFX(sound_menu_alter);
                menu_redraw(menu, NULL, selection);
            }
            #endif

            // The band ladder decides for itself whether anything besides the exposure
            // register needs rewriting, so both camera modes take the same path now:
            // the old "large error, so also reprogram gain" heuristic was standing in
            // for the hysteresis the ladder now has.
            if (render_registers) RENDER_REGS_FROM_EXPOSURE_SERVO();

    #if (DEBUG_AUTOEXP==1)
            sprintf(text_buffer, "%hu", (uint8_t)ratio);
            menu_text_out(14, 1, 6, WHITE_ON_BLACK, ITEM_DEFAULT, text_buffer);
    #endif
        }
#endif
        if ((image_live_preview) || (recording_video)) image_capture();
    }

    // render grid and all present scrollbars
    hide_sprites_range(scrollbar_render_all(grid_render(0)), MAX_HARDWARE_SPRITES);

    // wait for VBlank if not capturing (avoid HALT CPU state)
    if (!image_is_capturing() && !recording_video) sync_vblank();

    return 0;
}
uint8_t * formatItemText(camera_menu_e id, const uint8_t * format, camera_mode_settings_t * settings, bool divide_exposure) {
    static const uint8_t * const on_off[]   = {"Off",    "On"} ;
    static const uint8_t * const low_high[] = {"Low",    "High"};
    static const uint8_t * const norm_inv[] = {"Normal", "Inverted"};
    switch (id) {
        case idExposure: {
            uint32_t value = FROM_EXPOSURE_VALUE((divide_exposure) ? settings->current_exposure >> 1 : settings->current_exposure) / 10;
            uint8_t * buf = text_buffer_extra;
            uint8_t len = strlen(ultoa(value, buf, 10));
            // if too short value, add leading zeroes
            while (len < 3) {
                *--buf = '0', len++;
            }
            // insert comma
            len++;
            uint8_t * tail = buf + len;
            *tail-- = 0;
            *tail-- = *(tail - 1);
            *tail-- = *(tail - 1);
            *tail = ',';
            // cut trailing zeroes
            if (value < 1000) {
                tail = buf + len - 1;
                if (*tail == '0') *tail-- = 0;
                if (*tail == '0') *tail-- = 0;
                if (*tail == ',') *tail = 0;
            } else *tail = 0;
            // render
            sprintf(text_buffer, format, buf);
            break;
        }
        case idGain:
            sprintf(text_buffer, format, gains[settings->current_gain].caption);
            break;
        case idVOut:
            sprintf(text_buffer, format, settings->voltage_out);
            break;
        case idContrast:
            sprintf(text_buffer, format, settings->current_contrast);
            break;
        case idDither:
            sprintf(text_buffer, format, dither_patterns[settings->dithering].caption);
            break;
        case idDitherLight:
            sprintf(text_buffer, format, low_high[settings->ditheringHighLight]);
            break;
        case idInvOutput:
            sprintf(text_buffer, format, norm_inv[settings->invertOutput]);
            break;
        case idZeroPoint:
            sprintf(text_buffer, format, zero_points[settings->current_zero_point].caption);
            break;
        case idVoltageRef:
            sprintf(text_buffer, format, voltage_refs[settings->current_voltage_ref].caption);
            break;
        case idEdgeRatio:
            sprintf(text_buffer, format, edge_ratios[settings->current_edge_ratio].caption);
            break;
        case idEdgeExclusive:
            sprintf(text_buffer, format, on_off[settings->edge_exclusive]);
            break;
        case idEdgeOperation:
            sprintf(text_buffer, format, edge_operations[settings->edge_operation].caption);
            break;
        default:
            if (format) strcpy(text_buffer, format); else *text_buffer = 0;
            break;
    }
    return text_buffer;
}
uint8_t * onCameraMenuItemPaint(const struct menu_t * menu, const struct menu_item_t * self) {
    menu;
    return formatItemText(self->id, self->caption, &CURRENT_SETTINGS, _is_CPU_FAST);
}
uint8_t onHelpCameraMenu(const struct menu_t * menu, const struct menu_item_t * selection) {
    menu;
    // we draw help context here
    menu_text_out(0, 17, HELP_CONTEXT_WIDTH, HELP_CONTEXT_COLOR, ITEM_DEFAULT, selection->helpcontext);
    return 0;
}

uint8_t * camera_format_item_text(camera_menu_e id, const uint8_t * format, camera_mode_settings_t * settings) BANKED {
    return formatItemText(id, format, settings, settings->cpu_fast);
}

uint8_t UPDATE_state_camera(void) BANKED {
    static uint8_t menu_result;
    JOYPAD_RESET();
    one_iteration_autoexp = false;
    // start capturing of the image
    if ((image_live_preview) || (recording_video)) image_capture();
    // execute menu for the mode
    switch (OPTION(camera_mode)) {
        case camera_mode_manual:
            menu_result = menu_execute(&CameraMenuManual, NULL, last_menu_items[OPTION(camera_mode)]);
            break;
        case camera_mode_assisted:
            menu_result = menu_execute(&CameraMenuAssisted, NULL, last_menu_items[OPTION(camera_mode)]);
            break;
        case camera_mode_auto:
            menu_result = menu_execute(&CameraMenuAuto, NULL, last_menu_items[OPTION(camera_mode)]);
            break;
        default:
            // error, must not get here
            menu_result = ACTION_CAMERA_SUBMENU;
            break;
    }
    // wait until capturing process is finished
    while (image_is_capturing());
    // process menu result
    switch (menu_result) {
        case ACTION_CAMERA_PRINT:
            remote_activate(REMOTE_DISABLED);
            if (gbprinter_detect(PRINTER_DETECT_TIMEOUT) == PRN_STATUS_OK) {
                if (gbprinter_print_image(get_flipped_last_seen_image(OPTION(flip_live_view), false), CAMERA_BANK_LAST_SEEN, print_frames + OPTION(print_frame_idx), BANK(print_frames)) == PRN_STATUS_CANCELLED) {
                    // cancel button pressed while printing
                    reset_AEB();
                    COUNTER_RESET(camera_shutter_timer);
                    screen_clear_rect(SHUTTER_TIMER_X, SHUTTER_TIMER_Y, 2, 2, WHITE_ON_BLACK);
                    COUNTER_RESET(camera_repeat_counter);
                    screen_clear_rect(SHUTTER_REPEAT_X, SHUTTER_REPEAT_Y, 2, 2, WHITE_ON_BLACK);
                    camera_do_shutter = false;
                };
            } else PLAY_SFX(sound_error);
            remote_activate(REMOTE_ENABLED);
            break;
        case ACTION_CAMERA_TRANSFER:
            remote_activate(REMOTE_DISABLED);
            linkcable_transfer_reset();
            linkcable_transfer_image(get_flipped_last_seen_image(OPTION(flip_live_view), false), CAMERA_BANK_LAST_SEEN);
            remote_activate(REMOTE_ENABLED);
            break;
        case ACTION_MAIN_MENU:
            recording_video = false;
            reset_AEB();
            if (!menu_main_execute()) {
                COUNTER_RESET(camera_shutter_timer);
                COUNTER_RESET(camera_repeat_counter);
                refresh_screen();
            }
            break;
        case ACTION_CAMERA_SUBMENU: {
            recording_video = false;
            reset_AEB();
            do {
                switch (menu_result = menu_popup_camera_execute()) {
                    case ACTION_MODE_MANUAL:
                    case ACTION_MODE_ASSISTED:
                    case ACTION_MODE_AUTO:
                        static const camera_mode_e cmodes[] = {camera_mode_manual, camera_mode_assisted, camera_mode_auto};
                        OPTION(camera_mode) = cmodes[menu_result - ACTION_MODE_MANUAL];
                        // each mode carries its own exposure, so the band history from
                        // the mode being left does not apply to the one being entered
                        reset_autoexp_band();
                        RENDER_CAM_REGISTERS();
                        break;
                    case ACTION_TRIGGER_ABUTTON:
                    case ACTION_TRIGGER_TIMER:
                    case ACTION_TRIGGER_INTERVAL:
                    case ACTION_TRIGGER_AEB:
                        static const trigger_mode_e tmodes[] = {trigger_mode_abutton, trigger_mode_timer, trigger_mode_repeat, trigger_mode_AEB};
                        OPTION(trigger_mode) = tmodes[menu_result - ACTION_TRIGGER_ABUTTON];
                        break;
                    case ACTION_ACTION_PICNREC:
                    case ACTION_ACTION_PRINTPICNREC:
                    case ACTION_ACTION_PICNREC_VIDEO:
                        if ((_is_COLOR) && (OPTION(double_speed))) {
                            OPTION(double_speed) = false;
                            fade_out_modal();
                            CPU_SLOW();
                            music_setup_timer_ex(_is_CPU_FAST);
                            fade_in_modal();
                        }
                    case ACTION_ACTION_SAVE:
                    case ACTION_ACTION_PRINT:
                    case ACTION_ACTION_SAVEPRINT:
                    case ACTION_ACTION_TRANSFER:
                    case ACTION_ACTION_SAVETRANSFER:
                    case ACTION_ACTION_TRANSF_VIDEO:
                    case ACTION_ACTION_SAVESD:
                        static const after_action_e aactions[] = {
                            after_action_save, after_action_print, after_action_printsave,
                            after_action_transfer, after_action_transfersave, after_action_picnrec,
                            after_action_printpicnrec, after_action_picnrec_video, after_action_transfer_video,
                            after_action_savesd
                        };
                        OPTION(after_action) = aactions[menu_result - ACTION_ACTION_SAVE];
                        break;
                    case ACTION_AUTOEXP_CENTER:
                    case ACTION_AUTOEXP_TOP:
                    case ACTION_AUTOEXP_RIGHT:
                    case ACTION_AUTOEXP_BOTTOM:
                    case ACTION_AUTOEXP_LEFT:
                    case ACTION_AUTOEXP_OVERALL:
                        static const autoexp_area_e aareas[] = {
                            area_center, area_top, area_right, area_bottom, area_left, area_overall
                        };
                        OPTION(autoexp_area) = aareas[menu_result - ACTION_AUTOEXP_CENTER];
                        break;
                    case ACTION_RESTORE_DEFAULTS:
                        restore_default_mode_settings(OPTION(camera_mode));
                    case ACTION_SET_DITHERING:
                        RENDER_CAM_REGISTERS();
                        save_camera_mode_settings(OPTION(camera_mode));
                        break;
                    default:
                        // unknown command or cancel
                        PLAY_SFX(sound_ok);
                        break;
                }
                save_camera_state();
            } while (menu_result == MENU_RESULT_NO);
            COUNTER_RESET(camera_shutter_timer);
            COUNTER_RESET(camera_repeat_counter);
            camera_scrollbars_reinit();
            refresh_screen();
            break;
        }
        default:
            // unknown command or cancel
            PLAY_SFX(sound_ok);
            break;
    }
    return FALSE;
}

uint8_t LEAVE_state_camera(void) BANKED {
    fade_out_modal();
    recording_video = false;
    reset_AEB();
    gbprinter_set_handler(NULL, 0);
    if (_is_COLOR) ir_sense_stop();
    scrollbar_destroy_all();
    return 0;
}
