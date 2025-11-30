#ifndef SERIAL_COM_H
#define SERIAL_COM_H
#include <Arduino.h>
#include "mmwave.h"


void listenGet(mmWaveSensor &mmWave);
int extractNumber(String command);

#endif