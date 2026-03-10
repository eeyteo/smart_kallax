// node_config.h
#ifndef NODE_CONFIG_H
#define NODE_CONFIG_H

// Node Configuration - Edit these for each installation
#define NODE_NAME "gym_kallax"           // Unique name for this node (lowercase, no spaces)
#define NODE_DISPLAY_NAME "Gym Kallax"  // Display name in Home Assistant

// Hardware Configuration
#define LED_PRESENT true                 // Set to false if no LED strip
#define LED_PIN 32                       // GPIO pin for LED
#define RED_BTN 33                       // GPIO pin for Red button
#define GREEN_BTN 25                     // GPIO pin for Green button
#define BLUE_BTN 26                      // GPIO pin for Blue button
#define WHITE_BTN 27                     // GPIO pin for White button
#define PWM_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RES 8

#endif