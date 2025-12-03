#ifndef LED_CONTROL_H
#define LED_CONTROL_H
#include <Arduino.h>
#include "esp_timer.h"
#include "utils.h"

struct ledStrip{
    u_int8_t pwm_channel;
    u_int8_t led_pin;
    u_int8_t pwm_freq;
    u_int8_t pwm_res;
    u_int8_t duty;
    u_int8_t target_duty;
    u_int8_t max_duty;
    u_int8_t min_duty;
    u_int8_t start_duty;
    u_int16_t time_fade;
    int64_t t;
    int64_t fade_start_time;
    u_int8_t machine_state;
    float m;
    bool fading;
    bool pulsing;
    bool state;
    //default constructor
    ledStrip() : pwm_channel(0), led_pin(0), pwm_freq(5000), pwm_res(8), duty(0), target_duty(0), max_duty(255), min_duty(0), start_duty(0), time_fade(1000), t(0), fade_start_time(0), machine_state(0), m(0.0), fading(false), pulsing(false), state(false) {}

    void begin(int PWM_CHANNEL, int LED_PIN, int PWM_FREQ, int PWM_RES) {
        pwm_channel = PWM_CHANNEL;
        led_pin = LED_PIN;
        pwm_freq = PWM_FREQ;
        pwm_res = PWM_RES;
        max_duty = (1 << PWM_RES) - 1;
        min_duty = 0;
        time_fade = 8000; // default fade time
        m = float(max_duty - min_duty) / time_fade;
        ledcSetup(pwm_channel, pwm_freq, pwm_res);
        ledcAttachPin(led_pin, pwm_channel);
        ledcWrite(pwm_channel, 0);
    }

    bool canFadeIn(){
        // This method checks if the led can start the fade in manouvre
        if(!machine_state || machine_state == 3) return true;
        else return false;
    }

    bool canFadeOut(){
        // This method checks if the led can start the fade out manouvre
        if(machine_state == 2) return true;
        else return false;
    }
    
};

void manageLed(ledStrip &strip) ;
#endif