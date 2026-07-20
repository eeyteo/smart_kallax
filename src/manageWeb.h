#ifndef MANAGE_WEB_H
#define MANAGE_WEB_H

#include <SPIFFS.h> 
#include <WebServer.h>
#include <Arduino.h>


void servePage(WebServer &server, const char* path);

#endif