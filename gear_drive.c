#include "gear_drive.h"
#include <stdio.h>
#include <nrf.h>
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_drv_saadc.h"
#include "app_timer.h"
#include "nrf_gpio.h"
APP_TIMER_DEF(gear_button_timer);
//#define GEAR_IN_A 9
//#define GEAR_IN_B 10
#define GEAR_IN_A 19
#define GEAR_IN_B 20
#define GEAR_BUTTON 21
#define LED_RED 22
#define LED_GREEN 23
#define GEAR_SENSOR 2
#define GEAR_BUTTON_TIMER_INTERVAL 50
#define GEAR_DRIVE_TIMEOUT 3000

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_TICKS(MS, PRESCALER)\
            ((uint32_t)ROUNDED_DIV((MS) * (uint64_t)APP_TIMER_CLOCK_FREQ, ((PRESCALER) + 1) * 1000))


uint32_t now_millis = 0;
//static uint32_t disable_gear_transition_millis = 0;
//static uint32_t gear_sensor_value = 0;
//static uint32_t gear_sensor_high = 0, gear_sensor_low = 0;

enum gear_state_codes {
    waiting_low, waiting_high, falling, rising
};
enum ret_codes {
    repeat, done
};
struct transition {
    enum gear_state_codes src_state;
    enum ret_codes ret_code;
    enum gear_state_codes dst_state;
};
struct transition gear_state_transitions[] = {
        {waiting_low,  repeat, waiting_low},
        {waiting_low,  done,   rising},
        {waiting_high, repeat, waiting_high},
        {waiting_high, done,   falling},
        {falling,      repeat, falling},
        {falling,      done,   waiting_low},
        {rising,       repeat, rising},
        {rising,       done,   waiting_high}
};


enum gear_state_codes gear_cur_state;
uint32_t gear_drive_time_stop = 0;

enum ret_codes gear_waiting_low(void) {
    if (nrf_gpio_pin_read(GEAR_BUTTON) == 0)
        return repeat;
    gear_drive_time_stop = now_millis + GEAR_DRIVE_TIMEOUT;
    return done;
}

enum ret_codes gear_waiting_high(void) {
    if (nrf_gpio_pin_read(GEAR_BUTTON))
        return repeat;
    gear_drive_time_stop = now_millis + GEAR_DRIVE_TIMEOUT;
    return done;
}

enum ret_codes gear_falling(void) {
//    if(abs(gear_sensor_low-NRF_ADC->RESULT)<5 && gear_sensor_low>0 && abs(gear_sensor_high - gear_sensor_low)>100){
//        gear_drive_stop();
//        return done;
//    }
    if (gear_drive_time_stop <= now_millis) {
//        if(gear_sensor_low==0)
//            gear_sensor_low=NRF_ADC->RESULT;
        gear_drive_stop();
        return done;
    }
    gear_drive_fall();
    return repeat;
}

enum ret_codes gear_rising(void) {
//    if(abs(gear_sensor_high-NRF_ADC->RESULT)<5 && gear_sensor_high>0 && abs(gear_sensor_high - gear_sensor_low)>100){
//        gear_drive_stop();
//        return done;
//    }
    if (gear_drive_time_stop <= now_millis) {
//        if(gear_sensor_high==0)
//            gear_sensor_high=NRF_ADC->RESULT;
        gear_drive_stop();
        return done;
    }
    gear_drive_rise();
    return repeat;
}

void gear_drive_rise() {
    nrf_gpio_pin_write(GEAR_IN_A, 0);
    nrf_gpio_pin_write(GEAR_IN_B, 1);
    nrf_gpio_pin_write(LED_GREEN, 0);

}

void gear_drive_fall() {
    nrf_gpio_pin_write(GEAR_IN_A, 1);
    nrf_gpio_pin_write(GEAR_IN_B, 0);
    nrf_gpio_pin_write(LED_RED, 0);
}

void gear_drive_stop() {
    nrf_gpio_pin_write(GEAR_IN_A, 0);
    nrf_gpio_pin_write(GEAR_IN_B, 0);
    nrf_gpio_pin_write(LED_GREEN, 1);
    nrf_gpio_pin_write(LED_RED, 1);
}

int (*gear_state_functions[])(void) = {gear_waiting_low, gear_waiting_high, gear_falling, gear_rising};

//void gear_adc_sample();

void timer_gear_button_event_handler(void *p_context) {
    now_millis += GEAR_BUTTON_TIMER_INTERVAL;
    int (*gear_state_func)(void) = gear_state_functions[gear_cur_state];
    enum ret_codes gear_state_fun_ret_code = gear_state_func();
    gear_cur_state = gear_state_transitions[gear_cur_state * 2 + gear_state_fun_ret_code].dst_state;
}
void gear_init(void) {
    nrf_gpio_cfg_output(GEAR_IN_A);
    nrf_gpio_pin_write(GEAR_IN_A, 0);
    nrf_gpio_cfg_output(GEAR_IN_B);
    nrf_gpio_pin_write(GEAR_IN_B, 0);
    nrf_gpio_cfg_output(LED_GREEN);
    nrf_gpio_pin_write(LED_GREEN, 1);
    nrf_gpio_cfg_output(LED_RED);
    nrf_gpio_pin_write(LED_RED, 1);

    nrf_gpio_cfg_input(GEAR_BUTTON, NRF_GPIO_PIN_PULLUP);

    nrf_gpio_cfg_output(GEAR_IN_B);
    nrf_gpio_pin_write(GEAR_IN_B, 0);

    gear_cur_state = nrf_gpio_pin_read(GEAR_BUTTON) ? waiting_high : waiting_low;
    app_timer_create(&gear_button_timer, APP_TIMER_MODE_REPEATED, timer_gear_button_event_handler);
    app_timer_start(gear_button_timer, APP_TIMER_TICKS(GEAR_BUTTON_TIMER_INTERVAL, APP_TIMER_PRESCALER), NULL);
}



/*#define VERSION_ADC      ((0 << 5) | (31 & 0x1F))    // P0.31
#define VERSION_EN       ((0 << 5) | (26 & 0x1F))    // P0.26

void version_enable_init() {
    nrf_gpio_cfg_output(VERSION_EN);
    nrf_gpio_pin_write(VERSION_EN,1);
}

int main1(void)
{
    volatile int16_t result = 0;
    volatile float precise_result = 0;
    version_enable_init();
    nrf_delay_ms(100);

    // Start HFCLK from crystal oscillator, this will give the SAADC higher accuracy
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;

    // Configure SAADC singled-ended channel, Internal reference (0.6V) and 1/6 gain.
    NRF_SAADC->CH[0].CONFIG = (SAADC_CH_CONFIG_GAIN_Gain1_6    << SAADC_CH_CONFIG_GAIN_Pos) |
                              (SAADC_CH_CONFIG_MODE_SE         << SAADC_CH_CONFIG_MODE_Pos) |
                              (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
                              (SAADC_CH_CONFIG_RESN_Bypass     << SAADC_CH_CONFIG_RESN_Pos) |
                              (SAADC_CH_CONFIG_RESP_Bypass     << SAADC_CH_CONFIG_RESP_Pos) |
                              (SAADC_CH_CONFIG_TACQ_3us        << SAADC_CH_CONFIG_TACQ_Pos);

    // Configure the SAADC channel with VDD as positive input, no negative input(single ended).
    //NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_VDD << SAADC_CH_PSELP_PSELP_Pos;
    //NRF_SAADC->CH[0].PSELP = VERSION_ADC;
    NRF_SAADC->CH[0].PSELP = NRF_SAADC_INPUT_AIN0;
    NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos;

    // Configure the SAADC resolution.
    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_14bit << SAADC_RESOLUTION_VAL_Pos;

    // Configure result to be put in RAM at the location of "result" variable.
    NRF_SAADC->RESULT.MAXCNT = 1;
    NRF_SAADC->RESULT.PTR = (uint32_t)&result;

    // No automatic sampling, will trigger with TASKS_SAMPLE.
    NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;

    // Enable SAADC (would capture analog pins if they were used in CH[0].PSELP)
    NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos;

    // Calibrate the SAADC (only needs to be done once in a while)
    NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
    while (NRF_SAADC->EVENTS_CALIBRATEDONE == 0);
    NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
    while (NRF_SAADC->STATUS == (SAADC_STATUS_STATUS_Busy <<SAADC_STATUS_STATUS_Pos));

    // Start the SAADC and wait for the started event.
    NRF_SAADC->TASKS_START = 1;
    while (NRF_SAADC->EVENTS_STARTED == 0);
    NRF_SAADC->EVENTS_STARTED = 0;

    // Do a SAADC sample, will put the result in the configured RAM buffer.
    gear_adc_sample();


}

void gear_adc_sample() {
    NRF_SAADC->EVENTS_END = 0;
    NRF_SAADC->TASKS_SAMPLE = 1;
//    while (NRF_SAADC->EVENTS_END == 0);
    // Convert the result to voltage
    // Result = [V(p) - V(n)] * GAIN/REFERENCE * 2^(RESOLUTION)
    // Result = (VDD - 0) * ((1/6) / 0.6) * 2^14
    // VDD = Result / 4551.1
    // precise_result = (float)result / 4551.1f;
}*/
