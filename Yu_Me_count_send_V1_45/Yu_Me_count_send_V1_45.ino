#include <WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <Preferences.h>
#include <LiquidCrystal_PCF8574.h>
#include <WebServer.h>
#include <Wire.h>

// Revision Log
// v1.31 - Added web configuration, AP mode, and dynamic device ID
// v1.32 - Fixed MQTT connection issues and improved connection reliability
// v1.33 - Added heartbeat signaling on initial connection and reconnection
// v1.34 - Optimized LCD messages to fit better on screen
// v1.35 - Reduced serial output to only print counters on change or every 10 minutes
// v1.36 - Fixed counter reporting to strictly only report on change or every 10 minutes
// v1.37 - Fixed IP address display on LCD by showing full IP and shifting left
// v1.38 - Fixed compilation issues with include statements
// v1.39 - Optimized MQTT traffic to only send when counters change or at 10-minute intervals
// v1.40 - Streamlined serial output to reduce repetitive messages
// v1.41 - Added heartbeat value confirmation and removed redundant octet display
// v1.42 - Replaced numeric heartbeat with descriptive status messages
// v1.43 - Enhanced MQTT connection stability with improved reconnection logic
// v1.44  - Implemented proper LWT using PubSubClient library
// v1.45  - Simplified to use regular status updates instead of LWT

Preferences preferences; // Create a Preferences object for NVS
Ticker pulseTimer;       // Ticker for handling pulses

// WiFi credentials - now as variables that can be modified
String ssid = "Yu.me Wifi";
String password = "86868686";

// Adafruit IO credentials
#define IO_USERNAME  "nuphar007"
#define IO_KEY       "aio_sEwd80LfF4IVYqPF6Buq7iODz6Qi"
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883 // MQTT port

// Unique Device ID - now loaded from NVS
String deviceID = "spare"; // Default device ID

// Create MQTT client with PubSubClient
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// Define GPIO Pins
const int inputDollars = 25;   // D02 for Dollars input
const int inputWins = 14;      // D06 for Wins input
const int resetButton = 17;    // D10 for Reset input
const int gamesOutput = 16;    // D11 for Games output

// Counters for each input
volatile unsigned long dollarsCount = 0;
volatile unsigned long winsCount = 0;
volatile unsigned long gamesOutputCount = 0;   // Count for Games output pulses

// LCD Setup
LiquidCrystal_PCF8574 lcd(0x27);   // Set the LCD address to 0x27

// Queue for output pulses based on input counters
volatile bool dollarsStateHigh = false;
volatile bool winsStateHigh = false;
volatile unsigned long outputQueueCount = 0;

// Debounce timing - updated to use microseconds
const unsigned long debounceTime = 60;  // Adjusted debounce time
unsigned long lastDebounceTimeDollars = 0;
unsigned long lastDebounceTimeWins = 0;

// Long press reset timing
const unsigned long resetHoldTime = 3000;  // 3 seconds to reset
unsigned long resetButtonPressTime = 0;
volatile bool resetPressed = false;

// NVS Preferences keys
const char* nvsNamespace = "pulseCounter";

// Track when a change is first detected for NVS save
static unsigned long changeDetectedTime = 0;

// Serial print interval variables
unsigned long previousPrintTime = 0;   // Tracks the last time Serial printed
const unsigned long printInterval = 1000; // Set interval for Serial print (1 second)

// WiFi reconnection interval
const unsigned long wifiReconnectInterval = 300000; // 5 minutes
unsigned long lastWifiReconnectAttempt = 0;

// Track last sent values for conditional publishing
unsigned long lastSentDollarsCount = 0;
unsigned long lastSentWinsCount = 0;

// Flags for ISR handling
volatile bool dollarsISRFlag = false;
volatile bool winsISRFlag = false;

// Web server
WebServer server(80);

// Topic paths
String dollarFeedPath;
String winFeedPath;
String statusFeedPath;

void updateLCDStatus(const char* message) {
  lcd.setCursor(0, 3);  // Set cursor to the last row (row index 3)
  const int lcdWidth = 20;
  char formattedMessage[lcdWidth + 1];
  strncpy(formattedMessage, message, lcdWidth);
  formattedMessage[lcdWidth] = '\0';
  lcd.print("                    ");  // Clear the line
  lcd.setCursor(0, 3);
  lcd.print(formattedMessage);
}

void connectToWiFi() {
  updateLCDStatus("WiFi Connect...");
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid.c_str(), password.c_str());
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 30000) {
    delay(1000);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    updateLCDStatus("WiFi OK");
    Serial.println("Connected to WiFi.");
    
    // Print IP address to Serial Monitor
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    
    // Start web server in client mode
    server.on("/", handleRoot);
    server.on("/saveSettings", HTTP_POST, handleSaveSettings);
    server.begin();
    Serial.println("HTTP server started in client mode");
  } else {
    Serial.println("Failed to connect to WiFi. Switching to AP mode.");
    startAPMode();
  }
}

void startAPMode() {
  const char* apSSID = "ESP32_Config";
  const char* apPassword = "12345678";

  WiFi.softAP(apSSID, apPassword);
  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(ip);

  updateLCDStatus("AP Mode");
  lcd.setCursor(0, 2);
  lcd.print("IP: ");
  lcd.print(ip.toString());

  // Start web server in AP mode
  server.on("/", handleRoot);
  server.on("/saveSettings", HTTP_POST, handleSaveSettings);
  server.begin();
  Serial.println("HTTP server started in AP mode");
}

void handleRoot() {
  // Load values from NVS
  String storedSSID = preferences.getString("ssid", ssid);
  String storedPassword = preferences.getString("password", password);
  String storedDeviceID = preferences.getString("deviceID", deviceID);

  String html = "<html><head>";
  html += "<script>";
  html += "function toggleVisibility(id) {";
  html += "var x = document.getElementById(id);";
  html += "if (x.type === 'password') { x.type = 'text'; } else { x.type = 'password'; }";
  html += "}";
  html += "</script></head><body>";

  html += "<h1>Configuration</h1>";
  html += "<form action=\"/saveSettings\" method=\"post\">";

  html += "Device ID:<br><input type=\"text\" name=\"deviceID\" value=\"" + storedDeviceID + "\"><br>";
  html += "WiFi SSID:<br><input type=\"text\" name=\"ssid\" value=\"" + storedSSID + "\"><br>";
  
  // WiFi Password with Toggle Button
  html += "WiFi Password:<br>";
  html += "<input type=\"password\" id=\"password\" name=\"password\" value=\"" + storedPassword + "\">";
  html += "<button type=\"button\" onclick=\"toggleVisibility('password')\">Show/Hide</button><br>";

  html += "<input type=\"submit\" value=\"Save\">";
  html += "</form></body></html>";

  server.send(200, "text/html", html);
}

void handleSaveSettings() {
  if (server.hasArg("deviceID") && server.hasArg("ssid") && server.hasArg("password")) {
    // Convert device ID to lowercase for Adafruit IO compatibility
    String newDeviceID = server.arg("deviceID");
    newDeviceID.toLowerCase(); // Convert to lowercase
    
    String newSSID = server.arg("ssid");
    String newPassword = server.arg("password");

    preferences.putString("deviceID", newDeviceID);
    preferences.putString("ssid", newSSID);
    preferences.putString("password", newPassword);

    Serial.println("Settings saved:");
    Serial.println("Device ID: " + newDeviceID);
    Serial.println("SSID: " + newSSID);
    Serial.println("Password: " + newPassword);

    // Send response to client before restarting
    server.send(200, "text/html", "<html><body><h2>Settings saved. Rebooting...</h2></body></html>");
    delay(1000); // Allow time for response to be sent before restart

    // Restart device
    ESP.restart();
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void sendPulseData(bool forceSend = false) {
  if (!mqtt.connected()) {
    Serial.println("MQTT not connected. Cannot send data.");
    return;
  }
  
  bool dataChanged = false;
  
  // Check if dollars count has changed or forceSend is true
  if (dollarsCount != lastSentDollarsCount || forceSend) {
    if (!mqtt.publish(dollarFeedPath.c_str(), String(dollarsCount).c_str())) {
      Serial.println("Failed to send dollar pulses");
    } else {
      lastSentDollarsCount = dollarsCount;
      dataChanged = true;
    }
  }

  // Check if wins count has changed or forceSend is true
  if (winsCount != lastSentWinsCount || forceSend) {
    if (!mqtt.publish(winFeedPath.c_str(), String(winsCount).c_str())) {
      Serial.println("Failed to send win pulses");
    } else {
      lastSentWinsCount = winsCount;
      dataChanged = true;
    }
  }

  if (dataChanged || forceSend) {
    Serial.println("Data sent to Adafruit IO: $=" + String(dollarsCount) + ", Wins=" + String(winsCount));
  } else {
    Serial.println("No changes detected. Skipping data send.");
  }
}

void sendStatus() {
  if (mqtt.connected()) {
    if (mqtt.publish(statusFeedPath.c_str(), "online")) {
      Serial.println("Status sent: online");
    } else {
      Serial.println("Failed to send status");
    }
  } else {
    Serial.println("MQTT not connected. Cannot send status.");
  }
}

void connectToMQTT() {
  int retryCount = 0;
  int maxRetries = 3;
  
  updateLCDStatus("MQTT Connect...");

  // Check WiFi first
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected. Cannot connect to MQTT.");
    updateLCDStatus("No WiFi");
    return;
  }

  // Create client ID
  char clientID[50];
  snprintf(clientID, sizeof(clientID), "ESP32_%s_%u", deviceID.c_str(), random(0xffff));
  
  // Try to connect to MQTT server
  while (!mqtt.connected() && retryCount < maxRetries) {
    Serial.print("Connecting to MQTT... ");
    
    if (mqtt.connect(clientID, IO_USERNAME, IO_KEY)) {
      updateLCDStatus("MQTT OK");
      Serial.println("Connected!");
      
      // Immediately send online status
      if (mqtt.publish(statusFeedPath.c_str(), "online")) {
        Serial.println("Status sent: online");
      } else {
        Serial.println("Failed to send status");
      }
      
      return; // Successfully connected, exit function
    } else {
      retryCount++;
      Serial.print("Failed (");
      Serial.print(retryCount);
      Serial.print("/");
      Serial.print(maxRetries);
      Serial.println(")");
      Serial.println("Retrying in 5 seconds...");
      updateLCDStatus("MQTT Retry");
      delay(5000);
    }
  }

  if (!mqtt.connected()) {
    Serial.println("Failed to connect to MQTT after max retries.");
    updateLCDStatus("MQTT Fail");
  }
}

void debounceHandler(int pin, volatile unsigned long &lastDebounceTime, volatile unsigned long &count, volatile bool &stateHigh, volatile bool &isrFlag) {
  unsigned long currentTime = micros(); // Use microseconds for more precise debouncing
  unsigned long timeDifference;
  if (currentTime >= lastDebounceTime) {
    timeDifference = currentTime - lastDebounceTime;
  } else {
    timeDifference = (ULONG_MAX - lastDebounceTime) + currentTime + 1;
  }
  if (timeDifference > debounceTime * 1000) { // Convert debounceTime to microseconds
    bool currentState = digitalRead(pin);
    if (!stateHigh && currentState == HIGH) {
      stateHigh = true;
      isrFlag = true; // Set flag instead of directly incrementing counter
    } else if (stateHigh && currentState == LOW) {
      stateHigh = false;
      isrFlag = true; // Set flag instead of directly incrementing counter
    }
    lastDebounceTime = currentTime;
  }
}

void IRAM_ATTR dollarsISR() {
  debounceHandler(inputDollars, lastDebounceTimeDollars, dollarsCount, dollarsStateHigh, dollarsISRFlag);
}

void IRAM_ATTR winsISR() {
  debounceHandler(inputWins, lastDebounceTimeWins, winsCount, winsStateHigh, winsISRFlag);
}

void pulseTimerHandler() {
  static bool highPulse = true;
  if (outputQueueCount > 0) {
    if (highPulse) {
      digitalWrite(gamesOutput, LOW);
    } else {
      digitalWrite(gamesOutput, HIGH);
      noInterrupts();
      outputQueueCount--;
      gamesOutputCount++;
      interrupts();
    }
    highPulse = !highPulse;
  } else {
    digitalWrite(gamesOutput, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting setup...");

  // Initialize LCD
  lcd.begin(20, 4);
  lcd.setBacklight(255);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Starting setup...");

  // Initialize NVS and load settings
  preferences.begin(nvsNamespace, false);
  
  // Load device ID and WiFi credentials from NVS
  deviceID = preferences.getString("deviceID", "spare");
  ssid = preferences.getString("ssid", "Yu.me Wifi");
  password = preferences.getString("password", "86868686");
  
  Serial.print("Device ID: ");
  Serial.println(deviceID);
  
  // Create feed paths
  dollarFeedPath = String(IO_USERNAME) + "/feeds/" + deviceID + "-dollar-pulses";
  winFeedPath = String(IO_USERNAME) + "/feeds/" + deviceID + "-win-pulses";
  statusFeedPath = String(IO_USERNAME) + "/feeds/" + deviceID + "-status";
  
  // Configure MQTT server
  mqtt.setServer(AIO_SERVER, AIO_SERVERPORT);
  mqtt.setBufferSize(512);
  
  // Print feed paths for debugging
  Serial.println("Dollar feed path: " + dollarFeedPath);
  Serial.println("Win feed path: " + winFeedPath);
  Serial.println("Status feed path: " + statusFeedPath);

  // Initialize GPIOs
  pinMode(inputDollars, INPUT_PULLUP);
  pinMode(inputWins, INPUT_PULLUP);
  pinMode(resetButton, INPUT_PULLUP);  // GPIO 17
  pinMode(gamesOutput, OUTPUT);        // GPIO 16
  digitalWrite(gamesOutput, LOW);

  // Sync the initial state to prevent missed first pulses
  dollarsStateHigh = digitalRead(inputDollars);
  winsStateHigh = digitalRead(inputWins);

  // Attach interrupts for each input with CHANGE mode
  attachInterrupt(digitalPinToInterrupt(inputDollars), dollarsISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(inputWins), winsISR, CHANGE);

  // Connect to WiFi
  connectToWiFi();

  // Load counter values from NVS
  dollarsCount = preferences.getULong("dollarsCount", 0);
  winsCount = preferences.getULong("winsCount", 0);
  gamesOutputCount = preferences.getULong("gamesOutputCount", 0);

  // Set up pulse timer for handling output pulses
  pulseTimer.attach_ms(100, pulseTimerHandler);

  // Initialize last sent values
  lastSentDollarsCount = dollarsCount;
  lastSentWinsCount = winsCount;

  // Enable WiFi auto-reconnect
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  // Connect to MQTT
  connectToMQTT();

  // Send initial data to Adafruit IO only if connected
  Serial.println("Checking MQTT connection...");
  if (mqtt.connected()) {
    // Send the pulse data
    sendPulseData(true); // Force send initial data
  } else {
    Serial.println("MQTT not connected. Will send data once connected in main loop.");
  }
}

void loop() {
  unsigned long currentTime = millis();

  // Initialize static variables for counter reporting
  static bool firstLoopRun = true;
  static unsigned long lastReportedDollarsCount = 0;
  static unsigned long lastReportedWinsCount = 0;
  static unsigned long lastSerialPrintTime = 0;

  // Handle web server requests
  server.handleClient();

  // Process MQTT messages and maintain connection
  if (mqtt.connected()) {
    mqtt.loop();
  }

  // WiFi reconnection handling
  if (WiFi.status() != WL_CONNECTED && (currentTime - lastWifiReconnectAttempt >= wifiReconnectInterval)) {
    Serial.println("WiFi disconnected. Attempting to reconnect...");
    connectToWiFi();
    lastWifiReconnectAttempt = currentTime;
  }

  // MQTT reconnection handling
  static unsigned long lastMQTTRetryTime = 0;
  const unsigned long mqttRetryInterval = 300000; // Try to reconnect every 5 minutes
  static bool reportedDisconnect = false;
  
  if (!mqtt.connected()) {
    if (!reportedDisconnect) {
      Serial.println("MQTT disconnected.");
      reportedDisconnect = true;
    }
    
    if (currentTime - lastMQTTRetryTime >= mqttRetryInterval) {
      Serial.println("Attempting to reconnect to MQTT...");
      connectToMQTT();
      lastMQTTRetryTime = currentTime;
      
      // After reconnecting, try to send data if we're now connected
      if (mqtt.connected()) {
        Serial.println("MQTT reconnection successful. Sending data...");
        
        // Send pulse data
        sendPulseData(true);
        reportedDisconnect = false;
      }
    }
  } else {
    // Only reset flag when we're connected
    reportedDisconnect = false;
  }

  // Handle ISR flags in the main loop
  if (dollarsISRFlag) {
    noInterrupts();
    if (dollarsStateHigh) {
      dollarsCount++;
      outputQueueCount++;
    }
    dollarsISRFlag = false;
    interrupts();
  }

  if (winsISRFlag) {
    noInterrupts();
    if (winsStateHigh) {
      winsCount++;
      outputQueueCount++;
    }
    winsISRFlag = false;
    interrupts();
  }

  // Check for reset button long press
  if (digitalRead(resetButton) == LOW) {
    if (!resetPressed) {
      resetPressed = true;
      resetButtonPressTime = currentTime;
    } else if (resetPressed && (currentTime - resetButtonPressTime >= resetHoldTime)) {
      noInterrupts();
      dollarsCount = 0;
      winsCount = 0;
      gamesOutputCount = 0;
      outputQueueCount = 0;
      dollarsStateHigh = digitalRead(inputDollars);
      winsStateHigh = digitalRead(inputWins);
      lastDebounceTimeDollars = micros(); // Updated to micros()
      lastDebounceTimeWins = micros(); // Updated to micros()
      interrupts();
      preferences.putULong("dollarsCount", dollarsCount);
      preferences.putULong("winsCount", winsCount);
      preferences.putULong("gamesOutputCount", gamesOutputCount);
      updateLCDStatus("Reset Done");
      Serial.println("All counters have been reset.");
      resetPressed = false;
    }
  } else {
    resetPressed = false;
  }

  // Periodic save of counters to NVS with 10-second delay after change
  // Track when a change is first detected for NVS save
  static unsigned long lastSaveTime = 0;
  bool saveRequired = (dollarsCount != preferences.getULong("dollarsCount", 0)) ||
                      (winsCount != preferences.getULong("winsCount", 0)) ||
                      (gamesOutputCount != preferences.getULong("gamesOutputCount", 0));

  if (saveRequired) {
    if (changeDetectedTime == 0) {
      changeDetectedTime = currentTime; // Record the time of the first change
    }

    if (currentTime - changeDetectedTime >= 10000) {
      noInterrupts();
      preferences.putULong("dollarsCount", dollarsCount);
      preferences.putULong("winsCount", winsCount);
      preferences.putULong("gamesOutputCount", gamesOutputCount);
      interrupts();
      Serial.println("Counters saved to NVS.");
      lastSaveTime = currentTime;
      changeDetectedTime = 0; // Reset the change detection timestamp
    }
  } else {
    changeDetectedTime = 0; // Reset the change detection timestamp if no changes are required
  }

  // Send pulse data every 10 minutes only if there is a change
  static unsigned long lastSendTime = 0;
  const unsigned long sendDataInterval = 600000; // 10 minutes
  
  bool counterChangedSinceLastSend = (dollarsCount != lastSentDollarsCount) || (winsCount != lastSentWinsCount);
  bool timeForSend = (currentTime - lastSendTime >= sendDataInterval);
  
  if (counterChangedSinceLastSend && (timeForSend || firstLoopRun)) {
    sendPulseData(true);
    lastSendTime = currentTime;
  }

  // Send regular online status update every 10 minutes
  static unsigned long lastStatusUpdateTime = 0;
  const unsigned long statusUpdateInterval = 600000;  // 10 minutes
  if (currentTime - lastStatusUpdateTime >= statusUpdateInterval) {
    if (mqtt.connected()) {
      sendStatus();
    } else {
      Serial.println("MQTT not connected. Skipping status update.");
    }
    lastStatusUpdateTime = currentTime;
  }

  // Print initial counter values once
  if (firstLoopRun) {
    Serial.print("Initial Counter Status - Dollars: ");
    Serial.print(dollarsCount);
    Serial.print(", Wins: ");
    Serial.println(winsCount);
    
    lastReportedDollarsCount = dollarsCount;
    lastReportedWinsCount = winsCount;
    lastSerialPrintTime = currentTime;
    firstLoopRun = false;
  }

  // Print current counter statuses only on change or every 10 minutes
  const unsigned long serialPrintInterval = 600000; // 10 minutes (600,000 ms)
  
  bool countersChanged = (dollarsCount != lastReportedDollarsCount) || (winsCount != lastReportedWinsCount);
  bool timeForPrint = (currentTime - lastSerialPrintTime >= serialPrintInterval);
  
  if (countersChanged || timeForPrint) {
    Serial.print("Counter Status - Dollars: ");
    Serial.print(dollarsCount);
    Serial.print(", Wins: ");
    Serial.println(winsCount);
    
    // Update the last reported values
    lastReportedDollarsCount = dollarsCount;
    lastReportedWinsCount = winsCount;
    lastSerialPrintTime = currentTime;
  }

  // Update LCD always (every second)
  if (currentTime - previousPrintTime >= printInterval) {
    // Update LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Wins: ");
    lcd.print(winsCount);
    lcd.setCursor(0, 1);
    lcd.print("$: "); // Changed from "Dollars: " to "$: " for better space usage
    lcd.print(dollarsCount);
    lcd.setCursor(0, 2);
    lcd.print("Device: ");
    lcd.print(deviceID);
    
    // Display status and full IP address, shifted left by 4 characters
    updateLCDStatus("RunOK");
    lcd.setCursor(6, 3); // Position for IP display, shifted left
    lcd.print(WiFi.localIP().toString());
    
    previousPrintTime = currentTime;
  }
}