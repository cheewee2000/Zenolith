// ArduinoJson - https://arduinojson.org
// Copyright Benoit Blanchon 2014-2021
// MIT License
//
// This example shows how to deserialize a JSON document with ArduinoJson.
//
// https://arduinojson.org/v6/example/parser/

#include <ArduinoJson.h>
#include <SD.h>
const int chipSelect = 10; //m4

#include <SPI.h>
String UUID = "486403839";

struct Config {
  char UUID[64];
  float xSetpoint;
  float ySetpoint;
  float zSetpoint;

};

const char *filename = "UUID.txt";  // <- SD library uses 8.3 filenames
Config config;                         // <- global configuration object




void filter() {
  File file = SD.open(filename);

  // The filter: it contains "true" for each value we want to keep
  StaticJsonDocument<200> filter;
  filter["486403849"] = false;

  // Deserialize the document
  StaticJsonDocument<400> doc;
  deserializeJson(doc, file, DeserializationOption::Filter(filter));

  // Print the result
  serializeJsonPretty(doc, Serial);

}


void loadConfiguration(const char *filename, Config &config, String UUID) {
  // Open file for reading
  File file = SD.open(filename);
  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<512> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error)
    Serial.println(F("Failed to read file, using default configuration"));

  // Copy values from the JsonDocument to the Config
  config.xSetpoint = doc["486403849"]["xSetpoint"] | -1;
  config.ySetpoint = doc[UUID]["ySetpoint"] | -1;
  config.zSetpoint = doc[UUID]["zSetpoint"] | -1;
  Serial.println(config.xSetpoint );



//  strlcpy(config.UUID,                  // <- destination
//          doc["486403849"] | "0000",  // <- source
//          sizeof(config.UUID));         // <- destination's capacity
//  Serial.println(config.UUID );

  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();
}






// Prints the content of a file to the Serial
void printFile(const char *filename) {
  // Open file for reading
  File file = SD.open(filename);
  if (!file) {
    Serial.println(F("Failed to read file"));
    return;
  }

  // Extract each characters by one by one
  while (file.available()) {
    Serial.print((char)file.read());
  }
  Serial.println();

  // Close the file
  file.close();
}


void setup() {
  // Initialize serial port
  Serial.begin(9600);
  while (!Serial) continue;

  // Initialize SD library
  while (!SD.begin(chipSelect)) {
    Serial.println(F("Failed to initialize SD library"));
    delay(1000);
  }

  // Should load default config if run for the first time
  Serial.println(F("Loading configuration..."));
  loadConfiguration(filename, config, UUID);


  // Dump config file
  Serial.println(F("Print config file..."));
  printFile(filename);

  filter();
}

void loop() {
  // not used in this example
}
