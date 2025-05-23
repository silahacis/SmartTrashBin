// Code generated by Arduino IoT Cloud, DO NOT EDIT.

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>

const char DEVICE_LOGIN_NAME[]  = "7031eb63-6b0f-4edf-b9ed-09fee80b78d7";

const char SSID[]               = SECRET_SSID;    // Network SSID (name)
const char PASS[]               = SECRET_OPTIONAL_PASS;    // Network password (use for WPA, or use as key for WEP)
const char DEVICE_KEY[]  = SECRET_DEVICE_KEY;    // Secret device password

void onLidControlChange();

float averageFullness;
float latitude;
float longitude;
int fullness;
CloudLocation location;
bool lidControl;
bool notify;

void initProperties(){

  ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
  ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);
  ArduinoCloud.addProperty(averageFullness, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(latitude, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(longitude, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(fullness, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(location, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(lidControl, READWRITE, ON_CHANGE, onLidControlChange);
  ArduinoCloud.addProperty(notify, READ, ON_CHANGE, NULL);

}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
