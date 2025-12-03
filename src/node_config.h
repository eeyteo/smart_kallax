// node_config.h
#ifndef NODE_CONFIG_H
#define NODE_CONFIG_H

// Node Configuration - Edit these for each installation
#define NODE_NAME "gym_kallax"           // Unique name for this node (lowercase, no spaces)
#define NODE_DISPLAY_NAME "Gym Kallax"  // Display name in Home Assistant
#define NODE_IP 192,168,1,62            // Static IP address for this node

// Hardware Configuration
#define LED_PRESENT true                 // Set to false if no LED strip
#define LED_PIN 32                       // GPIO pin for LED
#define PWM_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RES 8

#endif