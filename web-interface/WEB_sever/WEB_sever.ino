#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <Wire.h>
#include <WebSocketsServer.h>
#include "SPIFFS.h"

#define USER_METRICS_ADDR 0
#define SETTINGS_ADDR sizeof(struct user_metrics)
#define DATA_READY_PIN 10
#define DATA_HEADER 0x01

bool WebDataAvailable = 0;

WebServer server(80);

WebSocketsServer webSocket = WebSocketsServer(81);

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      }
      break;
    case WStype_TEXT:
      Serial.printf("[%u] get Text: %s\n", num, payload);
      break;
  }
}

struct user_metrics {
  float weight;
  float height;
  char gender[20];
};

struct settings {
  float K;
  float G;
};

struct indicators {
  int exerciseMode;
  int activeSide;
  int repetitions;
};

//*****************************************************************************************************************************//
// Newly added code start
struct Data {
  float T_Cmd_M;   // master command torque
  float T_Cmd_S;   // slave command torque
  float T_Act_M;   // master actual torque
  float T_Act_S;   // slave actual torque
  float Pos_M;     // master position
  float Pos_S;     // slave position
  bool Is_Up = 0;  // movement direction
};


Data data_1 = {};
// Newly added code end
//****************************************************************************************************************************//



user_metrics user_metrics = {};
settings user_settings = {};

indicators current_indicators = {};

/********** For I2C*******************/

#define SLAVE_ADDRESS 0x00  //
#define BUFFER_SIZE 32

char receivedData[BUFFER_SIZE];
volatile boolean I2cDataReceived = false;

/*************************************/

void setup() {
  pinMode(DATA_READY_PIN, OUTPUT);
  digitalWrite(DATA_READY_PIN, LOW);

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  server.on("/Picture1.jpg", HTTP_GET, []() {
    File file = SPIFFS.open("/Picture1.jpg", "r");
    if (!file) {
      server.send(404, "text/plain", "File not found");
      return;
    }
    server.streamFile(file, "image/jpeg");
    file.close();
  });

  server.on("/Picture3.jpg", HTTP_GET, []() {
    File file = SPIFFS.open("/Picture3.jpg", "r");
    if (!file) {
      server.send(404, "text/plain", "File not found");
      return;
    }
    server.streamFile(file, "image/jpeg");
    file.close();
  });
  server.on("/Settings_bg.jpg", HTTP_GET, []() {
    File file = SPIFFS.open("/Settings_bg.jpg", "r");
    if (!file) {
      server.send(404, "text/plain", "File not found");
      return;
    }
    server.streamFile(file, "image/jpeg");
    file.close();
  });


  EEPROM.begin(sizeof(struct settings) + sizeof(struct user_metrics));
  EEPROM.get(USER_METRICS_ADDR, user_metrics);
  EEPROM.get(SETTINGS_ADDR, user_settings);

  // Set up the WiFi access point
  // WiFi.mode(WIFI_AP);
  WiFi.softAP("BMTR_ACCESS", "123456789");

  Serial.begin(115200);
  Serial.println("Access point created");

  Wire.begin(SLAVE_ADDRESS);     // Initialize I2C with the slave address
  Wire.onReceive(receiveEvent);  // Register receive event(stm32 sending data via i2c this is handlled in receiveEvent )
  Wire.onRequest(requestEvent);  // stm32 requeting data via i2c this is handlled in requestEvent
  Serial.println("I2C Slave Ready");
  //pinMode(LED_BUILTIN, OUTPUT);

  // Print the saved user data
  Serial.println("User Metrics:");
  Serial.print("Weight: ");
  Serial.print(user_metrics.weight);
  Serial.println(" kg");
  Serial.print("Height: ");
  Serial.print(user_metrics.height);
  Serial.println(" cm");
  Serial.print("Gender: ");
  Serial.println(user_metrics.gender);
  Serial.println("User Settings:");
  Serial.print("K: ");
  Serial.print(user_settings.K);
  Serial.println(" N/m");
  Serial.print("G: ");
  Serial.print(user_settings.G);
  Serial.println(" Hz");

  server.on("/", HTTP_GET, handlePortal);
  server.on("/", HTTP_POST, handlePortalPost);
  server.on("/settings", HTTP_GET, handleSettings);
  server.on("/save-settings", HTTP_POST, handleSaveSettings);
  server.on("/plot", HTTP_GET, handlePlot);

  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
 
  server.handleClient();
  webSocket.loop();
  if (!WebDataAvailable) {
    // built in led is inverted
    // digitalWrite(LED_BUILTIN, LOW);
  } else {
    //digitalWrite(LED_BUILTIN, HIGH);
  }

  if (I2cDataReceived) {
    //Serial.print("Received: ");
    Serial.println(receivedData);
    if (strcmp(receivedData, "STM Ready") == 0) {
      Serial.println("STM32 is ready to receive");
    } else {
      Serial.println("STM32 is not ready to receive");
    }
    I2cDataReceived = false;
  }

  // Process the received data
}

void receiveEvent(int byteCount) {

  if (byteCount == sizeof(float)) {
    float receivedValue;
    Wire.readBytes((uint8_t*)&receivedValue, sizeof(float));

    // Create JSON string with time and value
    String jsonString = "{\"type\":\"plot\",\"time\":" + String(millis() / 1000.0) + ",\"value\":" + String(receivedValue) + "}";

    // Send to all connected WebSocket clients
    webSocket.broadcastTXT(jsonString);

    Serial.println("Received and broadcasted: " + jsonString);
  }

  else if (byteCount == sizeof(struct indicators)) {
    Wire.readBytes((uint8_t*)&current_indicators, sizeof(struct indicators));
    Serial.println("Received indicator data from STM32");

    String jsonString = "{\"type\":\"indicator\",\"exerciseMode\":" + String(current_indicators.exerciseMode) + ",\"activeSide\":" + String(current_indicators.activeSide) + ",\"repetitions\":" + String(current_indicators.repetitions) + "}";

    // Send to all connected WebSocket clients
    webSocket.broadcastTXT(jsonString);

  }

  //*****************************************************************************************************************************//
  // Newly added code start
  else if (byteCount == sizeof(struct Data)) {
    Wire.readBytes((uint8_t*)&data_1, sizeof(struct Data));
    Serial.println("Received Torque and position data from STM32");

    String jsonString = "{\"type\":\"data01\",\"T_Cmd_M\":" + String(data_1.T_Cmd_M) + ",\"T_Cmd_S\":" + String(data_1.T_Cmd_S) + ",\"T_Act_M\":" + String(data_1.T_Act_M) + ",\"T_Act_S\":" + String(data_1.T_Act_S) + ",\"Pos_M\":" + String(data_1.Pos_M) + ",\"Pos_S\":" + String(data_1.Pos_S) + ",\"Is_Up\":" + String(data_1.Is_Up ? "true" : "false") + "}";
    // Send to all connected WebSocket clients
    webSocket.broadcastTXT(jsonString);

    // Serial.println(data_1.T_Cmd_M);
    // Serial.println(data_1.T_Cmd_S);
    // Serial.println(data_1.T_Act_M);
    // Serial.println(data_1.T_Act_S);
    // Serial.println(data_1.Pos_M);
    // Serial.println(data_1.Pos_S);
    // Serial.println(data_1.Is_Up);

  }
  // Newly added code end
  //*****************************************************************************************************************************//



  else if (byteCount == sizeof(struct user_metrics) + sizeof(struct settings)) {
    // This is confirmation data from STM32
    struct user_metrics confirmed_metrics;
    struct settings confirmed_settings;

    Wire.readBytes((uint8_t*)&confirmed_metrics, sizeof(struct user_metrics));
    Wire.readBytes((uint8_t*)&confirmed_settings, sizeof(struct settings));

    // Compare confirmed data with sent data
    if (memcmp(&confirmed_metrics, &user_metrics, sizeof(struct user_metrics)) == 0 && memcmp(&confirmed_settings, &user_settings, sizeof(settings)) == 0) {
      Serial.println("Data confirmed by STM32");
    } else {
      Serial.println("Data mismatch with STM32");
    }
  } else {
    // Handle other incoming data (like "STM Ready" message)
    int i = 0;
    while (Wire.available() && i < BUFFER_SIZE - 1) {
      char c = Wire.read();
      receivedData[i] = c;
      i++;
    }
    receivedData[i] = '\0';  // Null-terminate the string
    I2cDataReceived = true;
  }
}

void requestEvent() {

  //Wire.write("Hello from ESP32");  // Send this string when master requests data

  uint8_t buffer[sizeof(user_metrics) + sizeof(user_settings)];                  // Create a buffer to hold all the data
  memcpy(buffer, &user_metrics, sizeof(user_metrics));                           // Copy user_metrics into the buffer
  memcpy(buffer + sizeof(user_metrics), &user_settings, sizeof(user_settings));  // Copy user_settings into the buffer after user_metrics
  Wire.write(buffer, sizeof(buffer));                                            // Send the buffer over I2C
  Serial.println("DATA sent");

  digitalWrite(DATA_READY_PIN, LOW);
}

void handlePortal() {
  File file = SPIFFS.open("/Index.html", "r");
  if (!file) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  String html = file.readString();
  file.close();

  html.replace("%WEIGHT%", String(user_metrics.weight));
  html.replace("%HEIGHT%", String(user_metrics.height));
  html.replace("%MALE_SELECTED%", String(strcmp(user_metrics.gender, "male") == 0 ? "selected" : ""));
  html.replace("%FEMALE_SELECTED%", String(strcmp(user_metrics.gender, "female") == 0 ? "selected" : ""));

  server.send(200, "text/html", html);
  WebDataAvailable = 0;
}


void handlePortalPost() {
  user_metrics.weight = server.arg("weight").toFloat();
  user_metrics.height = server.arg("height").toFloat();
  strncpy(user_metrics.gender, server.arg("gender").c_str(), sizeof(user_metrics.gender) - 1);
  user_metrics.gender[sizeof(user_metrics.gender) - 1] = '\0';

  EEPROM.put(USER_METRICS_ADDR, user_metrics);
  EEPROM.commit();

  digitalWrite(DATA_READY_PIN, HIGH);

  File file = SPIFFS.open("/thank_you.html", "r");
  if (!file) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  server.streamFile(file, "text/html");
  file.close();

  // Print the user input data to the serial monitor
  Serial.println("User Metrics:");
  Serial.print("Weight: ");
  Serial.print(user_metrics.weight);
  Serial.println(" kg");
  Serial.print("Height: ");
  Serial.print(user_metrics.height);
  Serial.println(" cm");
  Serial.print("Gender: ");
  Serial.println(user_metrics.gender);

  WebDataAvailable = 1;
}


void handleSettings() {
  File file = SPIFFS.open("/settings.html", "r");
  if (!file) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  String html = file.readString();
  file.close();

  html.replace("%SPRING_10%", String(user_settings.K == 10 ? "selected" : ""));
  html.replace("%SPRING_20%", String(user_settings.K == 20 ? "selected" : ""));
  html.replace("%SPRING_30%", String(user_settings.K == 30 ? "selected" : ""));
  html.replace("%CUTOFF_FREQ%", String(user_settings.G));

  html.replace("%EXERCISE_MODE%", String(current_indicators.exerciseMode));
  html.replace("%ACTIVE_SIDE%", String(current_indicators.activeSide));
  html.replace("%REPETITIONS%", String(current_indicators.repetitions));


  //*****************************************************************************************************************************//
  // Newly added code start

  html.replace("%T_Cmd_M%", String(data_1.T_Cmd_M));
  html.replace("%T_Cmd_S%", String(data_1.T_Cmd_S));
  html.replace("%T_Act_M%", String(data_1.T_Act_M));
  html.replace("%T_Act_S%", String(data_1.T_Act_S));
  html.replace("%Pos_M%", String(data_1.Pos_M));
  html.replace("%Pos_S%", String(data_1.Pos_S));
    html.replace("%Is_Up%", String(data_1.Is_Up ? "true" : "false"));
  // Newly added code end
  //*****************************************************************************************************************************//


  server.send(200, "text/html", html);
}

void handleSaveSettings() {
  user_settings.K = server.arg("spring_constant").toFloat();
  user_settings.G = server.arg("cutoff_frequency").toFloat();

  EEPROM.put(SETTINGS_ADDR, user_settings);
  EEPROM.commit();

  digitalWrite(DATA_READY_PIN, HIGH);

  // Print the user settings data to the serial monitor
  Serial.println("User Settings:");
  Serial.print("K: ");
  Serial.print(user_settings.K);
  Serial.println(" N/m");
  Serial.print("G: ");
  Serial.print(user_settings.G);
  Serial.println(" Hz");

  File file = SPIFFS.open("/settings_saved.html", "r");
  if (!file) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  server.streamFile(file, "text/html");
  file.close();
}
void handlePlot() {
  File file = SPIFFS.open("/plot.html", "r");
  if (!file) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  server.streamFile(file, "text/html");
  file.close();
}
