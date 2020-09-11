// Learning From                https://www.instructables.com/id/IOT-Made-Simple-Playing-With-the-ESP32-on-Arduino-/
// Learning From                https://randomnerdtutorials.com/esp32-web-server-arduino-ide/
// Learning From                https://searchnetworking.techtarget.com/definition/TCP-IP
// Code From                    https://lastminuteengineers.com/creating-esp32-web-server-arduino-ide/
// Official Documentation       https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/
// Official Github              https://github.com/espressif/arduino-esp32
// Bonus: OLED 0.91 inch 128x32 https://www.instructables.com/id/Tutorial-to-Interface-OLED-091inch-128x32-With-Ard/

#include <WiFi.h>
#include <WebServer.h>

/* Put your SSID & Password */
const char* ssid     = "ESP32";  // Enter SSID here
const char* password = "12345678";  //Enter Password here

// declare an object of WebServer library
WebServer server(80);

// interrupt pin
#define intPin 14
bool intVal = 0;

void setup() {
  pinMode(intPin, INPUT);
  attachInterrupt(intPin, ISR, CHANGE);

  Serial.begin(9600);
  WiFi.softAP(ssid, password);
  IPAddress local_ip = WiFi.softAPIP();
  delay(1000);

  server.on("/", handle_OnConnect);
  server.onNotFound(handle_NotFound);

  server.begin();
  Serial.println("Server Started:");
  Serial.print("IP: ");
  Serial.print(local_ip) ;
}

void loop() {
  server.handleClient();
}

void handle_OnConnect() {
  intVal = digitalRead(intPin);
  server.send(200, "text/html", SendHTML());
}

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

void ISR() {Serial.println("LIMIT_SW_INTERRUPT");}

String SendHTML(){
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr +="<title>LED Control</title>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<h1>ESP32 Web Server</h1>\n";
  if (intVal) {ptr +="<h2>Pressed</h2>";}
  ptr +="</body>\n";
  ptr +="</html>\n";
  return(ptr);
}