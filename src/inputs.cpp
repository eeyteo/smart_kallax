#include "inputs.h"


void setupInputs(InputObj& redButton, InputObj& greenButton, InputObj& blueButton, InputObj& whiteButton) {
    // Create inputs for the button connected to GPIOs with pull-up and a minimum active time of 500ms
    redButton.setup();
    greenButton.setup();
    blueButton.setup();
    whiteButton.setup();
}

void updateInputs(bool isOnline, WiFiClient& espClient, HTTPClient& http, InputObj& redButton, InputObj& greenButton, InputObj& blueButton, InputObj& whiteButton) {
    // Update the state of each button and send updates to Home Assistant if changed
    redButton.updateState(isOnline, espClient, http);
    greenButton.updateState(isOnline, espClient, http);
    blueButton.updateState(isOnline, espClient, http);
    whiteButton.updateState(isOnline, espClient, http);
}