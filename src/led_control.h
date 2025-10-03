#ifndef LED_CONTROL_H
#define LED_CONTROL_H
#include <Arduino.h>
#include "esp_timer.h"


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
    int64_t fade_start_time;
    float m;
    bool fading;
    bool state;
    //default constructor
    ledStrip() : pwm_channel(0), led_pin(0), pwm_freq(5000), pwm_res(8), duty(0), target_duty(0), max_duty(255), min_duty(0), start_duty(0), time_fade(1000), fade_start_time(0), m(0.0), fading(false), state(false) {}

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

    void startFadeIn() {
        target_duty = max_duty;
        start_duty = duty;
        fade_start_time = esp_timer_get_time();
        fading = true;
        state = true;
    }

    void startFadeOut() {
        target_duty = min_duty;
        start_duty = duty;
        fade_start_time = esp_timer_get_time();
        fading = true;
        state = false;
    }

    void manageLed() { // This method brings duty to target duty with a fade effect
        char buffer[50];
         
        if (fading) {
            // Use 64-bit timer for start/current time
            int64_t current_time_us = esp_timer_get_time(); 
            
            // Elapsed time is also 64-bit, calculated in microseconds
            int64_t elapsed_time_us = current_time_us - fade_start_time;
            
            // Convert to milliseconds to use with your existing 'm' (rate of change)
            unsigned long elapsed_time_ms = elapsed_time_us / 1000; 
            int32_t new_duty;

            if (target_duty > duty) { // fading in
                new_duty = start_duty + m * elapsed_time_ms;
                if (new_duty >= target_duty) {
                    duty = target_duty;
                    fading = false;
                } else {
                    duty = (u_int8_t)new_duty; // Cast back to u_int8_t
                }
            } else if(target_duty < duty) { // fading out
                new_duty = start_duty - m * elapsed_time_ms;
                if (new_duty <= target_duty) {
                    duty = target_duty;
                    fading = false;
                } else {
                    duty = (u_int8_t)new_duty; // Cast back to u_int8_t
                }
            } else fading = false; // reached target

            if(fading || (duty == target_duty)){
                snprintf(buffer, sizeof(buffer), "Duty: %d, Target: %d", duty, target_duty);
                //Serial.println(buffer);      
                ledcWrite(pwm_channel, duty);
            }
            
        }    
    }
};


#endif