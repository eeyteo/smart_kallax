#include "led_control.h"

void manageLed(ledStrip &strip) {
    // With a state machine manage the behiebour of the led fading
    char buff[100];
    switch(strip.machine_state){
        case 0: // idle
            break;
        case 1: // Set the time and start fading in
            strip.fade_start_time = esp_timer_get_time()/1000;
            strip.start_duty = strip.duty;
            strip.machine_state = 2; // move to fading in
            break;
        case 2: // start fading in
            strip.t = (esp_timer_get_time()/1000 - strip.fade_start_time); // time in ms
            strip.duty = minmax(strip.start_duty + strip.m * strip.t , strip.min_duty, strip.max_duty);
            ledcWrite(strip.pwm_channel, strip.duty);
            sniprintf(buff, sizeof(buff), "turning on the led with duty: %d. Starting duty = %d, m = %f, t = %d", strip.duty, strip.start_duty, strip.m, strip.t);
            //if(strip.duty != strip.max_duty) Serial.println(buff);
            break;
        case 3: // Set the time and start fading out
            strip.fade_start_time = esp_timer_get_time() / 1000; //[ms]
            strip.start_duty = strip.duty;
            strip.machine_state = 4; // move to fading out
            break;
        case 4: // fading out
            strip.t = (esp_timer_get_time()/1000 - strip.fade_start_time) ; // time in ms
            strip.duty = minmax(strip.start_duty - strip.m * strip.t, strip.min_duty, strip.max_duty);
            ledcWrite(strip.pwm_channel, strip.duty);
            sniprintf(buff, sizeof(buff), "turning off the led with duty: %d. Starting duty = %d, m = %f, t = %d", strip.duty, strip.start_duty, strip.m, strip.t);
            //Serial.println(buff);
            if(strip.duty == strip.min_duty){
                strip.machine_state = 0; // back to idle
            }
            break;
        default:
            strip.machine_state = 0; // reset to idle in case of error
            break;
    }
}