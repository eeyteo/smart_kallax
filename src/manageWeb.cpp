#include "manageWeb.h"

// Function to serve HTML pages
void servePage(WebServer &server, const char* path) {
    String fullPath = "/" + String(path);
    File file = SPIFFS.open(fullPath, "r");  // <-- use LittleFS
    if (!file) {
        server.send(404, "text/plain", "File not found");
        return;
    }
    server.streamFile(file, "text/html");
    file.close();
}
