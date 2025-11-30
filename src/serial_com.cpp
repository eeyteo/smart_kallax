#include "serial_com.h"  
  
void listenGet(mmWaveSensor &mmWave) {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.isEmpty()) return;

  Serial.print("Received command: ");
  Serial.println(input);

  // --- Parse target ---
  int target = 0;  // Default target
  String command, param;

  int spaceIndex = input.indexOf(' ');
  String firstToken = (spaceIndex == -1) ? input : input.substring(0, spaceIndex);
  String rest = (spaceIndex == -1) ? "" : input.substring(spaceIndex + 1);
  rest.trim();

  if (firstToken.startsWith("target")) {
    target = firstToken.substring(6).toInt();
    command = rest;
  } 
  else if (firstToken.startsWith("t") && firstToken.length() > 1 && isDigit(firstToken.charAt(1))) {
    target = firstToken.substring(1).toInt();
    command = rest;
  } 
  else {
    command = input; // No explicit target
  }

  // Extract command and optional parameter
  spaceIndex = command.indexOf(' ');
  if (spaceIndex != -1) {
    param = command.substring(spaceIndex + 1);
    command = command.substring(0, spaceIndex);
    param.trim();
  }
  command.trim();


  // --- Command lookup table ---
  struct CommandEntry {
    const char* name;
    std::function<void()> action;
  };

  CommandEntry commands[] = {
    {"getParam", [&]() {
        Serial.println("Executing getParam...");
        if (getParam(mmWave))
          Serial.printf("✓ Successfully read sensor's parameters\n");
        else
          Serial.printf("✗ Failed to read sensor's parameters\n");
      }},
    
    {"showFrame", [&]() {
        mmWave.t = 100;
        Serial.println("Start showing the frame");
      }},
    {"setMaxMotionRange", [&]() {
        int value = extractNumber(param);
        if (value != -1) {
          mmWave.maxMotionRange.value = value;
          setMaxMotionRange(mmWave, mmWave.maxMotionRange.value);
          Serial.printf("Set Maximum Motion Range to %d cm\n", mmWave.maxMotionRange.value);
        } else {
          Serial.println("Invalid parameter for setMaxMotionRange");
        }
      }},
    {"help", [&]() {
        Serial.println("Available commands:");
        Serial.println("  getParam            - Print parameters for mm wave sensor");
        Serial.println("  showFrame           - show the frame for an amount of time");
        Serial.println("  setMaxMotionRange X - Set Maximum Motion Range to X cm");
        Serial.println("  help                          - Show available commands");
        Serial.println("Examples:");
        Serial.println("  getParam");
        Serial.println("  showFrame");
        Serial.println("  setMaxMotionRange 500");
      }}
  };

  // --- Execute matching command ---
  bool found = false;
  for (auto &cmd : commands) {
    if (command.equalsIgnoreCase(cmd.name)) {
      cmd.action();
      found = true;
      break;
    }
  }

  if (!found) {
    Serial.printf("Unknown command: '%s'\n", command.c_str());
    Serial.println("Type 'help' for available commands.");
  }

  Serial.println();
}




int extractNumber(String command) {
  // Helper function to extract numbers from commands
    for (int i = 0; i < command.length(); i++) {
        if (isDigit(command[i])) {
            return command.substring(i).toInt();
        }
    }
    return -1;
}