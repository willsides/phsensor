#include <TFT_eSPI.h>
#include <OneButton.h>
#include <math.h>  // Include for NAN and isnan()

TFT_eSPI tft = TFT_eSPI();  // Create object "tft"

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

bool isInCalMode = false;
float calibrationVoltage = 0.0;
int currentCalPoint = NAN;
bool stepCompleted = false;

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
  
  isInCalMode = true;
  tempCalPoint4 = NAN;
  tempCalPoint7 = NAN;
  tempCalPoint10 = NAN;

  button2.attachClick(cancelCalibration);
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
  if (!isInCalMode) return;
  
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
  if (!isInCalMode) return;

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
  if (!isInCalMode) return;
  
  button1.reset();
  button1.attachClick(nullptr);
  Serial.println("Calibration final confirmation");
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

// todo: Write values to EEPROM
void acceptNewCalibration() {
  calPoint4 = tempCalPoint4;
  calPoint7 = tempCalPoint7;
  calPoint10 = tempCalPoint10;
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

void setupDefaultMode() {
  button1.attachLongPressStart(handleCalibration);
}

void setup() {
  Serial.begin(115200);     // Start the Serial Monitor at 115200 baud rate
  tft.init();               // Initialize the TFT screen
  tft.setRotation(3);       // Set the rotation before using fillScreen
  tft.fillScreen(TFT_BLACK);  // Clear screen
  tft.setTextColor(TFT_WHITE, TFT_BLACK); // Set the font color and the background color
  tft.setTextSize(2);       // Set the text size
  analogReadResolution(12); // Set the ADC resolution to 12 bits
  analogSetAttenuation(ADC_2_5db);

  button1.setDebounceMs(50);
  button2.setDebounceMs(50);
  
  setupDefaultMode();
}

void loop() {
  button1.tick();
  button2.tick();
  float voltage = readVoltage();
  tft.fillScreen(TFT_BLACK); // Clear the screen with black
  tft.setCursor(0, 0);       // Set the cursor at the top left corner of the screen
  tft.setTextDatum(MC_DATUM); // Set the text datum to middle-center
  tft.drawString("Voltage: " + String(voltage, 3) + "V", tft.width()/2, tft.height()/2); // Draw the string in the center

  delay(100); // Wait for a second
}