#ifndef __STATE_CAMERA_H_INCLUDE__
#define __STATE_CAMERA_H_INCLUDE__

#include <gbdk/platform.h>
#include <stdbool.h>

#include "compat.h"
#include "gbcamera.h"
#include "globals.h"
#include "flip.h"

#include "systemdetect.h"
#include "systemhelpers.h"

/* 65536 exposure counts is exactly one second: a count is 16 M-cycles at 1048576 Hz,
   i.e. 15625/1024 us. The old macros used a flat 16 us, which made every displayed
   exposure read 4.9% high. The rational form below is exact in integer arithmetic. */
#define TO_EXPOSURE_VALUE(A) ((uint16_t)((((uint32_t)(A) * 1024UL) + 7812UL) / 15625UL))
/* 15625 as shifted adds. The z80 build has no 16x16->32 multiply helper, and this is
   only ever called to label the display, so the shifts cost nothing that matters. */
inline uint32_t FROM_EXPOSURE_VALUE(uint16_t count) {
    uint32_t v = count;   /* 15625 == (1<<13)+(1<<12)+(1<<11)+(1<<10)+(1<<8)+(1<<3)+1 */
    return ((v << 13) + (v << 12) + (v << 11) + (v << 10) + (v << 8) + (v << 3) + v + 512UL) >> 10;
}

#define EXPOSURE_LOW_LIMIT TO_EXPOSURE_VALUE(256)
#define EXPOSURE_HIGH_LIMIT CAM02_MAX_VALUE

BANKREF_EXTERN(state_camera)

typedef enum {
    set_off = 0,
    set_on
} switch_e;

typedef enum {
    camera_mode_manual,
    camera_mode_assisted,
    camera_mode_auto,
    N_CAMERA_MODES
} camera_mode_e;

typedef enum {
    trigger_mode_abutton,
    trigger_mode_timer,
    trigger_mode_repeat,
    trigger_mode_AEB,
    N_TRIGGER_MODES
} trigger_mode_e;

typedef enum {
    after_action_save,
    after_action_print,
    after_action_printsave,
    after_action_transfer,
    after_action_transfersave,
    after_action_picnrec,
    after_action_printpicnrec,
    after_action_picnrec_video,
    after_action_transfer_video,
    after_action_savesd,
    N_AFTER_ACTIONS
} after_action_e;

typedef enum {
    shutter_sound_0,
    shutter_sound_1,
    N_SHUTTER_SOUNDS
} shutter_sound_e;

typedef enum {
    idNone = 0,
    idExposure,
    idGain,
    idVOut,
    idContrast,
    idDither,
    idDitherLight,
    idInvOutput,
    idZeroPoint,
    idVoltageRef,
    idEdgeRatio,
    idEdgeExclusive,
    idEdgeOperation,
    idBrightness
} camera_menu_e;

typedef enum {
    area_center = 0,
    area_top,
    area_right,
    area_bottom,
    area_left,
    /* Appended rather than placed first so that settings saved by earlier builds keep
       pointing at the area they were set to. The menu lists it first regardless. */
    area_overall,
    N_AUTOEXP_AREAS
} autoexp_area_e;

typedef enum {
    cart_type_HDR = 0,
    cart_type_iG_AIO,
    N_CART_TYPES
} cart_type_e;

typedef struct table_value_t {
    uint8_t value;
    const uint8_t * caption;
} table_value_t;

typedef struct shutter_sound_t {
    uint8_t bank;
    uint8_t * sound;
    uint8_t mask;
} shutter_sound_t;

typedef struct camera_state_options_t {
    camera_mode_e camera_mode        : 4;
    trigger_mode_e trigger_mode      : 4;
    after_action_e after_action      : 4;
    shutter_sound_e shutter_sound    : 4;
    uint8_t gallery_picture_idx;
    uint8_t print_frame_idx;
    switch_e print_fast              : 1;
    switch_e fancy_sgb_border        : 1;
    switch_e show_grid               : 1;
    switch_e save_confirm            : 1;
    switch_e ir_remote_shutter       : 1;
    switch_e boot_to_camera_mode     : 1;
    switch_e double_speed            : 1;
    uint8_t shutter_timer;
    uint8_t shutter_counter;
    uint8_t cgb_palette_idx          : 4;
    switch_e display_exposure        : 1;
    switch_e enable_DMA              : 1;
    camera_flip_e flip_live_view     : 2;
    uint8_t aeb_overexp_count;
    uint8_t aeb_overexp_step;
    autoexp_area_e autoexp_area      : 4;
    cart_type_e cart_type            : 4;
} camera_state_options_t;

#define OPTION(OPT) camera_state.OPT
extern camera_state_options_t camera_state;

typedef struct camera_shadow_regs_t {
    uint8_t CAM_REG_CAPTURE;
    uint8_t CAM_REG_EDEXOPGAIN;
    uint16_t CAM_REG_EXPTIME;
    uint8_t CAM_REG_EDRAINVVREF;
    uint8_t CAM_REG_ZEROVOUT;
} camera_shadow_regs_t;

typedef struct camera_mode_settings_t {
    uint16_t current_exposure;
    uint8_t current_exposure_idx;
    int8_t current_gain;
    int8_t current_zero_point;
    int8_t current_edge_ratio;
    int8_t current_voltage_ref;
    int16_t voltage_out;
    uint8_t current_contrast;
    uint8_t edge_operation;
    int16_t current_brightness;
    uint8_t dithering                : 4;
    switch_e ditheringHighLight      : 1;
    switch_e invertOutput            : 1;
    switch_e edge_exclusive          : 1;
    switch_e cpu_fast                : 1;
} camera_mode_settings_t;

typedef struct image_metadata_t {
    camera_shadow_regs_t raw_regs;
    camera_mode_settings_t settings;
    uint16_t crc;
} image_metadata_t;
CHECK_SIZE_NOT_LARGER(image_metadata_t, CAMERA_THUMB_TILE_WIDTH * 4 * 2);   // 4 rows last rows of each last thumbnail tile 2 bytes each row

#define MODE_SETTING(SET,STAT) current_settings[OPTION(STAT)].SET
#define SETTING(SET) MODE_SETTING(SET,camera_mode)
#define CURRENT_SETTINGS current_settings[OPTION(camera_mode)]
extern camera_mode_settings_t current_settings[N_CAMERA_MODES];

extern bool recording_video;

extern camera_shadow_regs_t SHADOW;

#define PNR_DELAY_FRAMES 6
extern volatile uint8_t camera_PnR_delay;

inline void set_image_refresh_dalay(uint8_t delay) {
    camera_PnR_delay = delay;
}
inline bool image_is_capturing(void) {
    CAMERA_SWITCH_RAM(CAMERA_BANK_REGISTERS);
#if defined(CAMERA_EMULATE_CAPTURE)
    CAM_REG_CAPTURE &= ~CAM00F_CAPTURING;
    return false;
#else
    return ((camera_PnR_delay) || (CAM_REG_CAPTURE & CAM00F_CAPTURING));
#endif
}

uint8_t * camera_format_item_text(camera_menu_e id, const uint8_t * format, camera_mode_settings_t * settings) BANKED;

#define COUNTER_INFINITE_VALUE 31

/* The dither generator indexes contrast record (value - 1), and record 7 spreads the
   highlight thresholds over 48 counts against record 8's 39 -- which is what keeps a lit
   wall gradating smoothly instead of stepping. */
#define DEFAULT_CONTRAST_VALUE 8
#define DEFAULT_EXPOSURE_INDEX 90     // ~6000us on the 1/20-stop ladder

#define MAX_AEB_IMAGES 29
#define MAX_AEB_OVEREXPOSURE (MAX_AEB_IMAGES >> 1)
#define MIDDLE_AEB_IMAGE (MAX_AEB_IMAGES >> 1)

#endif
