#include <TFT_eSPI.h>
#include <OneButton.h>
#include <math.h>
#include <Preferences.h>
#include <WiFiManager.h>
#include <PubSubClient.h>

WiFiClient espClient;
PubSubClient mqttClient(espClient);

const char* mqtt_root = "homeassistant";

TFT_eSPI tft = TFT_eSPI(); 
WiFiManager wm;
Preferences preferences;

char mqtt_server[40];
char mqtt_port[6];

bool hasValidMQTTSettings = false;
unsigned long lastMQTTReconnectAttempt = 0;
const long mqttReconnectInterval = 10 * 60000;

const unsigned long publishInterval = 60000;
unsigned long lastPublishTime = 0;

WiFiManagerParameter wmparam_mqtt_server("server", "mqtt server", mqtt_server, 40);
WiFiManagerParameter wmparam_mqtt_port("port", "mqtt port", mqtt_port, 6);

// Button setup
OneButton button1(11, true, true);
OneButton button2(12, true, true);

// Calibration points
float calPoint4 = NAN;
float calPoint7 = NAN;
float calPoint10 = NAN;

float tempCalPoint4 = NAN;
float tempCalPoint7 = NAN;
float tempCalPoint10 = NAN;

bool isAPActive = false;
bool isInConfigMode = false;
bool isInCalMode = false;
float calibrationVoltage = 0.0;
int currentCalPoint = NAN;
bool stepCompleted = false;
bool isInCalRequiredMode = false;

const int adcPin = 9;      // ADC pin number
float voltage = 0.0;        // Voltage measured from ADC
const int numSamples = 400; 

float readAverageADC() {
  long sum = 0;  // Use long to avoid overflow with large number of samples

  for (int i = 0; i < numSamples; i++) {
    sum += analogRead(adcPin);  // Summing up all the ADC readings
  }

  float average = sum / numSamples;  // Calculating the average

  return average;
}

float readVoltage() {
	float rawADC = readAverageADC();
	return rawADC * (1100 / 4095. );
}

float convertPH(float voltage) {
// Check how many calibration points are set
    int numCalPoints = 0;
    if (!isnan(calPoint4)) numCalPoints++;
    if (!isnan(calPoint7)) numCalPoints++;
    if (!isnan(calPoint10)) numCalPoints++;

    // Handling different cases based on the number of calibration points
    switch (numCalPoints) {
        case 3:
            // Piecewise linear interpolation between three points
            if (voltage <= calPoint7) {
                // Interpolate between calPoint4 and calPoint7
                return interpolate(calPoint4, 4, calPoint7, 7, voltage);
            } else {
                // Interpolate between calPoint7 and calPoint10
                return interpolate(calPoint7, 7, calPoint10, 10, voltage);
            }

        case 2:
            // Linear interpolation between two points
            if (!isnan(calPoint4) && !isnan(calPoint7)) {
                return interpolate(calPoint4, 4, calPoint7, 7, voltage);
            } else if (!isnan(calPoint7) && !isnan(calPoint10)) {
                return interpolate(calPoint7, 7, calPoint10, 10, voltage);
            } else if (!isnan(calPoint4) && !isnan(calPoint10)) {
                return interpolate(calPoint4, 4, calPoint10, 10, voltage);
            }
            break;

        default:
            // Not enough calibration points
            return NAN;
    }
    return NAN; // Return NAN if something goes wrong
}

// Helper function to perform linear interpolation
float interpolate(float v1, float pH1, float v2, float pH2, float v) {
    return pH1 + (pH2 - pH1) * (v - v1) / (v2 - v1);
}

void handleCalibration() {	
  Serial.println("Starting calibration sequence");
  button1.reset();
  button2.reset();
  button1.attachLongPressStart(nullptr);
  button2.attachLongPressStart(nullptr);
  button1.attachClick(nullptr);
  button2.attachClick(nullptr);
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.println("Starting calibration sequence");
  
  for (int i = 0; i < 20 ; i++) {
  	Serial.println(String(i) + ". Button 1 State:" + String(button1.state()));
    button1.tick();
    button2.tick();
	delay(100);
  }  
  isInCalRequiredMode = false;
  isInCalMode = true;
  tempCalPoint4 = NAN;
  tempCalPoint7 = NAN;
  tempCalPoint10 = NAN;

  button2.attachLongPressStart(cancelCalibration);
  Serial.println("Calibration step 1");
  int calStep = 1;
  stepCompleted = false;
  while (!stepCompleted && isInCalMode) {
    calibrationVoltage = readVoltage();
	if (calibrationVoltage < 333 && isnan(tempCalPoint4)) currentCalPoint = 4;
	else if (calibrationVoltage > 666 && isnan(tempCalPoint10)) currentCalPoint = 10;
	else if (calibrationVoltage >= 333 && calibrationVoltage <= 666 && isnan(tempCalPoint7)) currentCalPoint = 7;
	else currentCalPoint = 0;

    if(currentCalPoint == 0) {displayStartCalibrationPrompt(calStep); button1.attachClick(nullptr);}
    else {displayCalibrationPrompt(calStep, currentCalPoint, calibrationVoltage); button1.attachClick(acceptCalPoint);}

    delay(100); // Small delay to reduce CPU load

    button1.tick();
    button2.tick();
  }
  if (!isInCalMode) {
    setupDefaultMode();
    return;}
  
  button1.reset();
  button1.attachClick(nullptr);
  Serial.println("Calibration step 2");
  calStep = 2;
  stepCompleted = false;
  while (!stepCompleted && isInCalMode) {

    calibrationVoltage = readVoltage();

	if (calibrationVoltage < 333 && isnan(tempCalPoint4)) currentCalPoint = 4;
	else if (calibrationVoltage > 666 && isnan(tempCalPoint10)) currentCalPoint = 10;
	else if (calibrationVoltage >= 333 && calibrationVoltage <= 666 && isnan(tempCalPoint7)) currentCalPoint = 7;
	else currentCalPoint = 0;

    if(currentCalPoint == 0) {displayStartCalibrationPrompt(calStep); button1.attachClick(nullptr);}
    else {displayCalibrationPrompt(calStep, currentCalPoint, calibrationVoltage); button1.attachClick(acceptCalPoint);}

    delay(100); // Small delay to reduce CPU load

    button1.tick();
    button2.tick();
  }
  if (!isInCalMode) {
    setupDefaultMode();
    return;}

  button1.reset();
  button1.attachClick(nullptr);
  button1.attachLongPressStart(acceptNewCalibration);
  Serial.println("Calibration step 3");
  calStep = 3;
  stepCompleted = false;
  while (!stepCompleted && isInCalMode) {

    calibrationVoltage = readVoltage();

	if (calibrationVoltage < 333 && isnan(tempCalPoint4)) currentCalPoint = 4;
	else if (calibrationVoltage > 666 && isnan(tempCalPoint10)) currentCalPoint = 10;
	else if (calibrationVoltage >= 333 && calibrationVoltage <= 666 && isnan(tempCalPoint7)) currentCalPoint = 7;
	else currentCalPoint = 0;

    if(currentCalPoint == 0) {displayStartCalibrationPrompt(calStep); button1.attachClick(nullptr);}
    else {displayCalibrationPrompt(calStep, currentCalPoint, calibrationVoltage); button1.attachClick(acceptCalPoint);}

    delay(100); // Small delay to reduce CPU load
	
    button1.tick();
    button2.tick();
  }
  if (!isInCalMode) {
    setupDefaultMode();
    return;}
  
  button1.reset();
  button1.attachClick(nullptr);
  Serial.println("Calibration final confirmation");
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.println("Hold button 1 to accept calibration");
  while (isInCalMode) {
    button1.tick();
    button2.tick();
    delay(100); // Small delay to reduce CPU load
  }
  setupDefaultMode();
}

void acceptCalPoint() {
  switch (currentCalPoint) {
    case 4: tempCalPoint4 = calibrationVoltage; break;
    case 7: tempCalPoint7 = calibrationVoltage; break;
    case 10: tempCalPoint10 = calibrationVoltage; break;
  }
  Serial.println("Calibration Accepted for pH " + String(currentCalPoint));
  stepCompleted = true;
}

// Function to accept new calibration and save it to EEPROM
void acceptNewCalibration() {
  calPoint4 = tempCalPoint4;
  calPoint7 = tempCalPoint7;
  calPoint10 = tempCalPoint10;

	preferences.begin("pHSensor", false);
	preferences.putFloat("calPoint4", calPoint4);
	preferences.putFloat("calPoint7", calPoint7);
	preferences.putFloat("calPoint10", calPoint10);
	preferences.end();

  publishCalValues();
	
  isInCalMode = false;
  Serial.println("Calibration accepted");

  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.println("Calibration accepted");
  delay(2000);
}

void cancelCalibration() {
  isInCalMode = false;
  Serial.println("Calibration Cancelled");
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.println("Calibration cancelled");
  delay(2000);
}

void displayCalibrationPrompt(int step, int currentCalPoint, int voltage) {
  String calString;
  switch (currentCalPoint) {
    case 4: calString = "4.0"; break;
    case 7: calString  = "7.0"; break;
    case 10: calString  = "10.0"; break;
  }
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.println("Calibration point " + String(step));
  tft.println("Calibrating pH " + calString);
  tft.println(String(voltage) + " mV");
}

void displayStartCalibrationPrompt(int step) {
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  String s;
  if (step == 1) s = "first";
  else s = "next";
  tft.println("Insert the pH probe into " + s + " calibration standard");
}

void cancelConfiguration() {
	if (isAPActive) {
		deactivateAP();
	}
	Serial.println("Exiting configuration");
	isInConfigMode = false;
}

void activateAP() {
	isAPActive = true;
	Serial.println("Starting AP");
  wm.setConfigPortalBlocking(false);
	wm.startConfigPortal("WS-pHSensor", "potentialofhydrogen");
	button1.attachLongPressStart(deactivateAP);
}

void deactivateAP(){
	isAPActive = false;
	Serial.println("Stopping AP");
	wm.stopConfigPortal();
	button1.attachLongPressStart(activateAP);
}

void handleConfiguration () {
  isInCalRequiredMode = false;
  isInConfigMode = true;
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.println("Entering configuration");
  button1.reset();
  button2.reset();
  for (int i = 0; i < 20 ; i++) {
  	Serial.println(String(i) + ". Button 1 State:" + String(button1.state()));
    button1.tick();
    button2.tick();
	delay(100);
  }
  button1.attachClick(nullptr);
  button2.attachClick(nullptr);
  button1.attachLongPressStart(activateAP);
  button2.attachLongPressStart(cancelConfiguration);

  while(isInConfigMode) {
	if (isAPActive) {
		tft.fillScreen(TFT_BLACK);
  	tft.setCursor(0, 0);
		tft.println("Hold btn 1 to stop AP");
		tft.println("Hold btn 2 to exit");
		while (isAPActive && isInConfigMode) {
			button1.tick();
			button2.tick();
			wm.process();
			delay(100);
		}
	}
	else {
		tft.fillScreen(TFT_BLACK);
  	tft.setCursor(0, 0);
		tft.println("Hold btn 1 to start AP");
		tft.println("Hold btn 2 to exit");
		while (!isAPActive && isInConfigMode) {
			button1.tick();
			button2.tick();
			delay(100);
		}
	}
  }
  setupDefaultMode();
}

void setupDefaultMode() {
	Serial.println("Starting pH mode");
  button1.attachClick(nullptr);
  button2.attachClick(nullptr);
  button1.attachLongPressStart(handleCalibration);
  button2.attachLongPressStart(handleConfiguration);

  if (strlen(mqtt_server) > 0 && strlen(mqtt_port) > 0) hasValidMQTTSettings = true;
  else hasValidMQTTSettings = false;
  if (hasValidMQTTSettings && !mqttClient.connected()) {
      mqttClient.setServer(mqtt_server, atoi(mqtt_port));
      mqttConnect(); // Attempt to connect to MQTT
  }
}

void displayPHAndVoltage(float pH, int voltage) {
    tft.fillScreen(TFT_BLACK); // Clear the screen

    // Prepare the pH string
    char buffer[20]; // Buffer to hold the formatted pH string
    sprintf(buffer, "pH %.2f", pH); // Format pH to two decimal places

    // Calculate positions
    int centerX = tft.width() / 2;
    int centerY = tft.height() / 2;

    // Display pH
    tft.setTextDatum(MC_DATUM);
    tft.drawString(buffer, centerX, centerY - 10); // Adjust Y position as needed

    // Prepare the voltage string
    sprintf(buffer, "%d mV", voltage); // Use the voltage as it is

    // Display voltage
    tft.drawString(buffer, centerX, centerY + 10); // Adjust Y position as needed
}

void handleCalibrationRequired() {
  isInCalRequiredMode = true;
  float pH = convertPH(readVoltage());
	tft.fillScreen(TFT_BLACK);
	tft.setCursor(0, 0);
	tft.println("Calibration needed");
  while (isnan(pH) & isInCalRequiredMode) {
	button1.tick();
	button2.tick();
	pH = convertPH(readVoltage());
	delay(100);
  }
  setupDefaultMode();
}

void updateCalValuesFromFlash() {
  Serial.println("Fetching cal values from memory");
  preferences.begin("pHSensor", true);
	calPoint4 = preferences.getFloat("calPoint4", NAN);
  if (calPoint4 <= 0 || calPoint4 > 1100){
    calPoint4 = NAN;
    preferences.putFloat("calPoint4", calPoint4);
  } 
	calPoint7 = preferences.getFloat("calPoint7", NAN);
	if (calPoint7 <= 0 || calPoint7 > 1100) {
    calPoint7 = NAN;
    preferences.putFloat("calPoint7", calPoint7);
	}
  calPoint10 = preferences.getFloat("calPoint10", NAN);
	if (calPoint10 <= 0 || calPoint10 > 1100) {
    calPoint10 = NAN;
    preferences.putFloat("calPoint10", calPoint10);
	}
	preferences.end();

  Serial.print("Calibration Point 4: ");
  Serial.println(calPoint4);
  Serial.print("Calibration Point 7: ");
  Serial.println(calPoint7);
  Serial.print("Calibration Point 10: ");
  Serial.println(calPoint10);
}

void updateSettingsFromFlash() {
  Serial.println("Fetching settings from memory");
  
  preferences.begin("mqtt", true);
  strncpy(mqtt_server, preferences.getString("mqtt_server", "").c_str(), sizeof(mqtt_server) - 1);
  mqtt_server[sizeof(mqtt_server) - 1] = '\0'; // Ensure null-termination
  strncpy(mqtt_port, preferences.getString("mqtt_port", "1883").c_str(), sizeof(mqtt_port) - 1);
  mqtt_port[sizeof(mqtt_port) - 1] = '\0'; // Ensure null-termination
	preferences.end();

  Serial.print("MQTT server: ");
  Serial.println(mqtt_server);
  Serial.print("MQTT port: ");
  Serial.println(mqtt_port);
}

void setupWifiManager() {
  wmparam_mqtt_server.setValue(mqtt_server, 40);
  wmparam_mqtt_port.setValue(mqtt_port, 6);
  wm.addParameter(&wmparam_mqtt_server);
  wm.addParameter(&wmparam_mqtt_port);
  wm.setSaveParamsCallback(saveParamsCallback);
  wm.setSaveConfigCallback(saveConfigCallback);
  
  wm.setEnableConfigPortal(false);
  wm.setConfigPortalBlocking(false);
  wm.autoConnect();
}

void saveParamsCallback () {
  Serial.println("Saving params");
  strncpy(mqtt_server, wmparam_mqtt_server.getValue(), sizeof(mqtt_server) - 1);
  mqtt_server[sizeof(mqtt_server) - 1] = '\0'; // Ensure null-termination
  strncpy(mqtt_port, wmparam_mqtt_port.getValue(), sizeof(mqtt_port) - 1);
  mqtt_port[sizeof(mqtt_port) - 1] = '\0'; // Ensure null-termination
  Serial.print("mqtt server: ");
  Serial.println(mqtt_server);
  Serial.print("mqtt port: ");
  Serial.println(mqtt_port);
  preferences.begin("mqtt", false);
	preferences.putString("mqtt_server", mqtt_server);
	preferences.putString("mqtt_port", mqtt_port);
	preferences.end();
}

void saveConfigCallback() {
	isAPActive = false;
	Serial.println("Connected to WiFi");
	button1.attachLongPressStart(activateAP);
}

void setupTFT() {
  tft.init();               // Initialize the TFT screen
  tft.setRotation(3);       // Set the rotation before using fillScreen
  tft.fillScreen(TFT_BLACK);  // Clear screen
  tft.setTextColor(TFT_WHITE, TFT_BLACK); // Set the font color and the background color
  tft.setTextSize(2);       // Set the text size
}

void mqttConnect() {
    if (!mqttClient.connected()) {
        Serial.println("Attempting MQTT connection...");
        // Attempt to connect (clientID, username, password can be added as parameters)
        if (mqttClient.connect("ESP32Client")) {
            Serial.println("connected to MQTT server");
            publishDeviceConfig();
            publishCalValues();
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 10 minutes");
            lastMQTTReconnectAttempt = millis();
        }
    }
}

void processMQTT() {
  unsigned long now = millis();
  if (!mqttClient.connected()) {
      if (now - lastMQTTReconnectAttempt > mqttReconnectInterval) {
          lastMQTTReconnectAttempt = now;
          // Try to reconnect
          mqttConnect();
      }
  } else {
      mqttClient.loop(); // This needs to be called regularly to allow MQTT communication
      if (now - lastPublishTime >= publishInterval) {
        lastPublishTime = now;
        publishState();
      }
  }
}

void publishMQTT(const char* topic, const char* payload, bool retain) {
    Serial.print("Attempting to send message to topic ");
    Serial.println(topic);
    Serial.println(payload);
    if (mqttClient.connected()) {
        if (mqttClient.publish(topic, payload, retain)) {
          Serial.println("MQTT message sent successfully.");
        } else {Serial.println("MQTT message failed to send.");}

    } else {
        Serial.println("MQTT not connected, message not sent.");
    }
}

void publishDeviceConfig() {
    String configTopic = String(mqtt_root) + "/sensor/WS_pH_sensor_pH/config";
    String configPayloadPH = "{";
    configPayloadPH += "\"name\": \"WS pH Sensor pH\",";
    configPayloadPH += "\"device_class\": \"ph\",";
    configPayloadPH += "\"state_topic\": \"" + String(mqtt_root) + "/sensor/WS_pH_sensor/state\",";
    configPayloadPH += "\"json_attributes_topic\": \"" + String(mqtt_root) + "/sensor/WS_pH_sensor/attributes\",";
    configPayloadPH += "\"unit_of_measurement\": \"pH\",";
    configPayloadPH += "\"value_template\": \"{{ value_json.pH }}\",";
    configPayloadPH += "\"unique_id\": \"pH_sensor_65b6221c-b311-4e83-bc7e-7eb8ceabf070\",";
    configPayloadPH += "\"device\": {";
    configPayloadPH += "\"identifiers\": [\"WS_pH_Sensor\"],";
    configPayloadPH += "\"name\": \"pH Sensor\"";
    configPayloadPH += "}}";

    publishMQTT(configTopic.c_str(), configPayloadPH.c_str(), true);

    String configTopicVoltage = String(mqtt_root) + "/sensor/WS_pH_sensor_voltage/config";
    String configPayloadVoltage = "{";
    configPayloadVoltage += "\"name\": \"WS pH Sensor Voltage\",";
    configPayloadVoltage += "\"device_class\": \"voltage\",";
    configPayloadVoltage += "\"state_topic\": \"" + String(mqtt_root) + "/sensor/WS_pH_sensor/state\",";
    configPayloadVoltage += "\"unit_of_measurement\": \"mV\",";
    configPayloadVoltage += "\"value_template\": \"{{ value_json.voltage }}\",";
    configPayloadVoltage += "\"unique_id\": \"pH_sensor_mV_fce62792-a9a4-471d-954e-36aa61ee852c\",";
    configPayloadVoltage += "\"device\": {";
    configPayloadVoltage += "\"identifiers\": [\"WS_pH_Sensor\"],";
    configPayloadVoltage += "\"name\": \"pH Sensor\"";
    configPayloadVoltage += "}}";

    publishMQTT(configTopicVoltage.c_str(), configPayloadVoltage.c_str(), true);
}

void publishCalValues() {
    String stateTopic = String(mqtt_root) + "/sensor/WS_pH_sensor/attributes";
    String statePayload = "{";
    statePayload += "\"calPoint4\":" + String(calPoint4, 2) + ",";
    statePayload += "\"calPoint7\":" + String(calPoint7, 2) + ",";
    statePayload += "\"calPoint10\":" + String(calPoint10, 2);
    statePayload += "}";

    publishMQTT(stateTopic.c_str(), statePayload.c_str(), true);
}

void publishState() {
    float voltage = readVoltage();
    float pH = convertPH(voltage);

    String stateTopic = String(mqtt_root) + "/sensor/WS_pH_sensor/state";
    String statePayload = "{";
    statePayload += "\"pH\":" + String(pH, 2) + ",";
    statePayload += "\"voltage\":" + String(voltage, 2);
    statePayload += "}";

    publishMQTT(stateTopic.c_str(), statePayload.c_str(), false);
}

bool isConnectedToWiFi() {
    return WiFi.status() == WL_CONNECTED;
}

void setup() {
  Serial.begin(115200);     // Start the Serial Monitor at 115200 baud rate
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP 
  analogReadResolution(12); // Set the ADC resolution to 12 bits
  analogSetAttenuation(ADC_2_5db);
  mqttClient.setBufferSize(4096);
  button1.setDebounceMs(20);
  button2.setDebounceMs(20);

  setupTFT();

  updateSettingsFromFlash();

  updateCalValuesFromFlash();

  setupWifiManager();

  setupDefaultMode();
}

void loop() {
  button1.tick();
  button2.tick();
  processMQTT();
  float voltage = readVoltage();
  float pH = convertPH(voltage);
  if (isnan(pH)){ handleCalibrationRequired();}
  else {displayPHAndVoltage(pH, voltage);}
  delay(100); // Wait for a second
}