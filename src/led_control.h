#ifndef LED_CONTROL_H
#define LED_CONTROL_H
#include <Arduino.h>

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
    u_int16_t fade_start_time;
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
        fade_start_time = millis();
        fading = true;
        state = true;
    }

    void startFadeOut() {
        target_duty = min_duty;
        start_duty = duty;
        fade_start_time = millis();
        fading = true;
        state = false;
    }

    void manageLed() { // This method brings duty to target duty with a fade effect
        char buffer[50];
         
        if (fading) {
            unsigned long current_time = millis();
            unsigned long elapsed_time = current_time - fade_start_time;
            
            if (target_duty > duty) { // fading in
                duty = start_duty + m * elapsed_time;
            } else if(target_duty < duty) { // fading out
                duty = start_duty - m * elapsed_time;   
            } else fading = false; // reached target
            if(fading){
                snprintf(buffer, sizeof(buffer), "Duty: %d, Target: %d", duty, target_duty);
                //Serial.println(buffer);      
            }
            ledcWrite(pwm_channel, duty);
        }    
    }
};


#endif