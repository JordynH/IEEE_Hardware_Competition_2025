#include "encoder.h"
#include "driver/pcnt.h"
#include "esp_log.h"

#define TAG "ENCODER"

// Define PCNT units for each motor
#define PCNT_UNIT_FR PCNT_UNIT_0
#define PCNT_UNIT_FL PCNT_UNIT_1
#define PCNT_UNIT_BR PCNT_UNIT_2
#define PCNT_UNIT_BL PCNT_UNIT_3

// Define Encoder Pins
#define ENC_A_FR 16
// #define ENC_B_FR 22
#define ENC_B_FR 36
#define ENC_A_FL 34
#define ENC_B_FL 15
#define ENC_A_BR 2
#define ENC_B_BR 39
#define ENC_A_BL 26
#define ENC_B_BL 35

void encoder_init(pcnt_unit_t unit, int encA, int encB) {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = encA,  // A channel
        .ctrl_gpio_num = encB,   // B channel
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .counter_h_lim = 10000,
        .counter_l_lim = -10000,
        .unit = unit,
        .channel = PCNT_CHANNEL_0
    };

    pcnt_unit_config(&pcnt_config);

    // Enable input filter (useful for debouncing mechanical noise)
    pcnt_set_filter_value(unit, 250); // Adjust if needed
    pcnt_filter_enable(unit);

    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

void init_all_encoders() {
    encoder_init(PCNT_UNIT_FR, ENC_A_FR, ENC_B_FR);
    encoder_init(PCNT_UNIT_FL, ENC_A_FL, ENC_B_FL);
    encoder_init(PCNT_UNIT_BR, ENC_A_BR, ENC_B_BR);
    encoder_init(PCNT_UNIT_BL, ENC_A_BL, ENC_B_BL);
}

int read_encoder(pcnt_unit_t unit) {
    int16_t count = 0;
    pcnt_get_counter_value(unit, &count);
    return count;
}
