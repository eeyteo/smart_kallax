#include "endpoints.h"


void handleFirmwareUpdate(WebServer &server) {
    // Handle the firmware update submission (via POST request)
    server.on("/update", HTTP_POST,
        // Response after the firmware upload
        [&server]() {
            server.sendHeader("Connection", "close");
            server.send(200, "text/plain", (Update.hasError()) ? "Update Failed!" : "Update Successful. Rebooting...");
            delay(1000);
            ESP.restart(); // Reboot the ESP32 after the update
        },
        // Upload process (this is called during the POST request)
        [&server]() {
            HTTPUpload& upload = server.upload();
            if (upload.status == UPLOAD_FILE_START) {
                Serial.printf("Update: %s\n", upload.filename.c_str());
                if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { // Start with max available size
                    Update.printError(Serial);
                }
            } else if (upload.status == UPLOAD_FILE_WRITE) {
                // Flash the firmware
                if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                    Update.printError(Serial);
                }
            } else if (upload.status == UPLOAD_FILE_END) {
                if (Update.end(true)) { // End the update
                    Serial.printf("Update Success: %u bytes\n", upload.totalSize);
                } else {
                    Update.printError(Serial);
                }
            }
        }
    );
}