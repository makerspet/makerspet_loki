#include "ap.h"

//Variables to save values from HTML form
String ssid;
String pass;
String dest_ip;
String dest_port;

// File paths to save input values permanently
const char* ssidPath = "/ssid.txt";
const char* passPath = "/pass.txt";
const char* destIpPath = "/dest_ip.txt";
const char* destPortPath = "/dest_port.txt";

// Read File from SPIFFS
String readFile(fs::FS &fs, const char * path) {
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if(!file || file.isDirectory()){
    Serial.println("- failed to open file for reading");
    return String();
  }
  
  String fileContent;
  while(file.available()){
    fileContent = file.readStringUntil('\n');
    break;     
  }
  return fileContent;
}

// Initialize SPIFFS
void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("Error mounting SPIFFS");
    return;
  }
  Serial.println("SPIFFS mounted successfully");

  // Load values saved in SPIFFS
  ssid = readFile(SPIFFS, ssidPath);
  pass = readFile(SPIFFS, passPath);
  dest_ip = readFile(SPIFFS, destIpPath);
  dest_port = readFile(SPIFFS, destPortPath);

  Serial.print("ssid='");
  Serial.print(ssid);
  Serial.print("' pass='");
  Serial.print(pass);
  Serial.print("' dest_ip='");
  Serial.print(dest_ip);
  Serial.print("' dest_port='");
  Serial.print(dest_port);
  Serial.println("'");
}

// Write file to SPIFFS
void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- frite failed");
  }
}

void ObtainWiFiCreds(void (*callback)()) {
  AsyncWebServer server(80);  // Create AsyncWebServer object on port 80

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting up Access Point ");
  Serial.println(SSID_AP);
  // NULL sets an open Access Point
  WiFi.softAP(SSID_AP, NULL);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP); 

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/wifimanager.html", "text/html");
  });
  
  server.serveStatic("/", SPIFFS, "/");
  
  server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
    int params = request->params();
    for(int i=0;i<params;i++){
      AsyncWebParameter* p = request->getParam(i);
      if(p->isPost()){
        // HTTP POST ssid value
        if (p->name() == PARAM_INPUT_SSID) {
          ssid = p->value().c_str();
          Serial.print("SSID set to: ");
          Serial.println(ssid);
          // Write file to save value
          writeFile(SPIFFS, ssidPath, ssid.c_str());
        }
        // HTTP POST pass value
        if (p->name() == PARAM_INPUT_PASS) {
          pass = p->value().c_str();
          Serial.print("Password set to: ");
          Serial.println(pass);
          // Write file to save value
          writeFile(SPIFFS, passPath, pass.c_str());
        }
        // HTTP POST destination IP value
        if (p->name() == PARAM_INPUT_DEST_IP) {
          dest_ip = p->value().c_str();
          Serial.print("Destination IP set to: ");
          Serial.println(dest_ip);
          // Write file to save value
          writeFile(SPIFFS, destIpPath, dest_ip.c_str());
        }
        // HTTP POST destination port value
        if (p->name() == PARAM_INPUT_DEST_PORT) {
          dest_port = p->value().c_str();
          Serial.print("Destination port set to: ");
          Serial.println(dest_port);
          // Write file to save value
          writeFile(SPIFFS, destPortPath, dest_port.c_str());
        }
        //Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
      }
    }
    request->send(200, "text/html; charset=utf-8",
      "<HTML><BODY>"
      "<center><h1><br>Connecting to your router...</h1>"
      "<p>SSID: " + ssid + "</p>"
      "<p>WiFi password: [not shown]</p>"
      "<p>Destination IP: " + dest_ip + "</p>"
      "<p>Destination Port: " + dest_port + "</p>"
      "</center></BODY></HTML>");
    unsigned long ms = millis();
    while(millis() - ms < 3000)
      yield();
    ESP.restart();
  });
  server.begin();
  while(true) {
    callback();
    yield();
  }
}

String getSSID() {
  return ssid;
}

String getPassw() {
  return pass;
}

String getDestIP() {
  return dest_ip;
}

String getDestPort() {
  return dest_port;
}

void resetWiFiSettings() {
  SPIFFS.remove(ssidPath);
  SPIFFS.remove(passPath);
  SPIFFS.remove(destIpPath);
  SPIFFS.remove(destPortPath);
}
