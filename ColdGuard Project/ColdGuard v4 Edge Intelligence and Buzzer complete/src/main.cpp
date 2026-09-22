//Arduino library for PlatformIO
#include <Arduino.h>

//include the DHTesp Library
//This library allows the ESP32 to communicate with the DHT22
//temperature and humidity sensor.
#include <DHTesp.h>

//Include the I2C LCD library.
//This allows the ESP32 to display information
//on the 16x2 LCD screen.
#include <LiquidCrystal_I2C.h>

//Wi-Fi library built into the ESP32 framework. 
#include <WiFi.h>

//Allows us to create an encrypted TLS connection
//to Adafruit IO.
#include <WiFiClientSecure.h>

//Libary for MQTT client 
#include <PubSubClient.h>

//WIFI SETTINGS
//Wokwi provides this virtual Wi-Fi network.
const char* WIFI_SSID = "Wokwi-GUEST";

//ADAFRUIT IO MQTT SETTINGS
//Adafruit IO MQTT broker.
const char* MQTT_SERVER = "io.adafruit.com";

//Secure MQTT/TLS port.
const int MQTT_PORT = 8883;

//Adafruit IO username for Dashboard.
const char* AIO_USERNAME = "";

//Adafruit IO key. 
const char* AIO_KEY = "";

//Wokwi-GUEST does not require a password.
const char* WIFI_PASSWORD = "";

//ADAFRUIT IO FEED TOPICS
//Adafruit IO topic format:
//username/feeds/feed-key
//Example: LachlanB/feeds/temperature
//Topic used to publish temperature readings.
String temperatureTopic = 
  String(AIO_USERNAME) + "/feeds/temperature";

//Topic used to publish humidity readings.
String humidityTopic =
  String(AIO_USERNAME) + "/feeds/humidity";

//Topic used to publish door status readings.
String doorTopic =
  String(AIO_USERNAME) + "/feeds/door";

//Topic used to publish cooling/relay status readings.
String relayTopic = 
  String(AIO_USERNAME) + "/feeds/relay";

//Topic used to publish System state readings.
String stateTopic =
  String(AIO_USERNAME) + "/feeds/state";

//Adafruit IO feed used to display the result
//of ColdGuard's local edge-intelligence analysis.
String anomalyTopic =
  String(AIO_USERNAME) + "/feeds/anomaly";

//Topic used to publish System Fault readings.
String faultTopic =
  String(AIO_USERNAME) + "/feeds/fault";

//Topic used to publish Recent Events / Stream to Recent Events feed.
String eventsTopic =
  String(AIO_USERNAME) + "/feeds/events";

//SENSOR UPDATE TIMER
//Stores the last time sensor information
//was printed to the Serial Monitor.
unsigned long lastSensorUpdate = 0;

//How often we want to print sensor information.
//2000 milliseconds = 2 seconds.
const unsigned long SENSOR_INTERVAL = 2000;

//EDGE INTELLIGENCE - ROLLING TEMPERATURE WINDOW
//ColdGuard stores the most recent 30 temperature readings.
//These readings will later be used to calculate: 
// - Average temperature
// - Standard deviation
// - Rate of temperature change
// - STABLE / DRIFTING / CRITICAL classification
const int TEMP_WINDOW_SIZE = 30;

//Array that stores the latest 30 temperature readings.
float temperatureWindow[TEMP_WINDOW_SIZE];

//Position where the next temperature reading will be stored.
int temperatureWindowIndex = 0;

//Tracks how many valid readings have been collected.
//This prevents calculations from using empty array positions
//while ColdGuard is first starting.
int temperatureReadingCount = 0;

//Stores the latest result from the edge-intelligence
//classification system.
//This is global because both the sensor-processing
//section and the MQTT publishing section need to access it.
String edgeClassification = "WARMING_UP";

//Stores the last edge Classification that was
//successfully sent to Adafruit IO.
//
//This allows ColdGuard to detect when the classification
//changes and publish the new value immediately.
String lastPublishedEdgeClassification = "";

//EDGE INTELLIGENCE CLASSIFICATION THRESHOLDS
//A Z-score of 2.0 means the current temperature
//is at least two standard deviations away
//from the recent rolling mean.
const float Z_SCORE_DRIFT_THRESHOLD = 2.0;

//A temperature change of 0.5 degrees Celsius
//per minute or greater is considered a meaningful trend.
const float SLOPE_DRIFT_THRESHOLD = 0.5;

//MQTT PUBLISH Timer
//Stores the last time dashboard telemetry
//was published to Adafruit IO.
unsigned long lastMqttPublish = 0;

//Publish dashboard telemtry every 15 seconds.
const unsigned long MQTT_PUBLISH_INTERVAL = 15000;

//The DHT22 data pin is connected to GPIO 33 on the ESP32
const int DHT_PIN = 33;

//GPIO pin connected to the analogue output
//of the photoresistor/LDR module.
const int LDR_PIN = 34;

//BUZZER PIN
//GPIO 25 constrols the buzzer.
//The buzzer provides an audible warning when
//the fridge door has been left open too long.
const int BUZZER_PIN = 25;

//BUZZER PWM SETTINGS
//The passive buzzer needs a rapidly
//changing signal to create an audible tone.
//
//The ESP32 LEDC hardware generates this signal.
const int BUZZER_CHANNEL = 0;
const int BUZZER_FREQUENCY = 2000;
const int BUZZER_RESOLUTION = 8;

//DOOR TIMER SETTINGS
//Maximum time the fridge door can remain open
//before the buzzer alarm activates.
//60000 milliseconds = 60 seconds.
const unsigned long DOOR_OPEN_LIMIT = 60000;

//TEMPERATURE DWELL TIMERS
//Demo timing: 
//Temperature must remain above 8.0C for 12 seconds
//before the cooling response is considered sustained.
const unsigned long COOLING_DWELL_TIME = 12000;

//Temperature must remain above 8.0C for 60 seconds
//before ColdGuard enters EXCURSION.
const unsigned long EXCURSION_DWELL_TIME = 60000;

//Tracks when the high-temperature condition began.
unsigned long highTempStartedAt = 0;

//Tracks whether the high-temperature timer is active.
bool highTempTimerRunning = false;

//Stores the time when the door was first opened.
unsigned long doorOpenedAt = 0;

//Keeps track of whether the door timer is active.
bool doorTimerRunning = false;

//DOOR REMINDER CHIRP SETTINGS
//Once the door alarm becoems active, the buzzer
//will sound briefly rather than continously.
//
//How long each reminder chirp lasts.
const unsigned long DOOR_CHIRP_DURATION = 150;

//How long to wait between reminder chirps.
//For the Wokwi demo we use 10 seconds.
const unsigned long DOOR_CHIRP_INTERVAL = 5000;

//Stores when the previous reminder chirp started.
unsigned long lastDoorChirpAt = 0;

//Stores when the current chirp started.
unsigned long doorChirpStartedAt = 0;

//True only while the short reminder chirp is sounding.
bool doorChirpActive = false;

//RGB LED PINS

//Red LED channel.
const int RGB_RED_PIN = 14;

//Green LED channel.
const int RGB_GREEN_PIN = 27;

//Blue LED channel.
const int RGB_BLUE_PIN = 13;

//RELAY PIN
//GPIO 26 controls the relay module.
//The relay represents the refrigerator's
//compressor/cooling system.
const int RELAY_PIN = 26;

//SAFE TEMPERATURE RANGE
const float SAFE_MIN_TEMP = 2.0;
const float SAFE_MAX_TEMP = 8.0;

//SENSOR PLAUSIBILITY RANGE
//Readings outside this range are considered
//implausible for the vaccine fridge and indicate
//a possible sensor fault.
const float PLAUSIBLE_MIN_TEMP = -10.0;
const float PLAUSIBLE_MAX_TEMP = 30.0;

//COOLING CONTROL THRESHOLDS
//If temperature reaches or exceeds 8.5°C,
//ColdGuard will turn cooling ON.
const float COOLING_ON_TEMP = 8.5;

//Once cooling is running, the temperature must fall
//to 7.5°C before ColdGuard turns cooling OFF.
//Having seperate ON and OFF thresholds creates
//hysteresis and prevents rapid delay switching.
const float COOLING_OFF_TEMP = 7.5;

//Stores the current cooling state.
//false = cooling OFF
//true = cooling ON
//This variable gives the controller state memory.
bool coolingOn = false;

//EXCURSION LED BLINK TIMER
//Used to blink the red LED without delay(). 
//This allows the rest of Colduard to continue running
//while the warning LED is flashing.
//Stores the last time the LED changed state.
unsigned long lastLedBlink = 0;

//500 milliseconds = LED changes every half second.
const unsigned long LED_BLINK_INTERVAL = 500;

//Stores whether the blinking LED is currently ON or OFF.
bool excursionLedOn = false;

//FINITE STATE MACHINE
//ColdGuard can only be in one main operating state at a time.
enum SystemState {

  //Fridge is operating normally.
  NORMAL,

  //Fridge door is currently open.
  DOOR_OPEN,

  //Cooling system is actively trying to reduce temperature.
  COOLING,

  //Temperature has remained unsafe for too long,
  //or the temperature has fallen below the safe minimum.
  EXCURSION,

  //A sensor reading is invalid or implausible.
  FAILSAFE
};

//Stores ColdGuard's current operating state.
//The system starts in NORMAL when the ESP32 boots.
SystemState currentState = NORMAL;

//Temporary LDR Threshold. 
//Assumed values are for now:
// Dark / door closed = 3800
//Bright / door open = 2000
//Any value below 2900 will temporrily
//be treated as the fridge door being open.
const int LDR_THRESHOLD = 2900;

//Create a DHTesp object called "dht".
//We use this object to configure and read the DHT22 sensor.
DHTesp dht;

//LCD DISPLAY
//Create an LCD object.
//0x27 = common I2C address used by the Wokwi LCD.
//16 = number of columns.
//2 = number of rows.
LiquidCrystal_I2C lcd(0x27, 16, 2);

//NETWORK OBJECTS
//Creates a secure Wi-Fi connection.
WiFiClientSecure wifiClient;

//Uses that secre connection for MQTT.
PubSubClient mqttClient(wifiClient);

//FUNCTION: publishEvent
//Sends a text event to the Adafruit IO events feed.
void publishEvent(const char* message) {

  bool eventPublishSuccess =
    mqttClient.publish(
      eventsTopic.c_str(),
      message
    );


if (eventPublishSuccess) {

  Serial.print("MQTT Event published: ");
  Serial.println(message);
} else {

  Serial.println("MQTT Event publish FAILED");
}
}

//FUNCTION: setRGB
//Controls the colour of the RGB status LED.
void setRGB(bool red, bool green, bool blue) {

  //Set the red LED channel.
  digitalWrite(RGB_RED_PIN, red);

  //Set the green LED channel.
  digitalWrite(RGB_GREEN_PIN, green);

  //Set the blue LED channel.
  digitalWrite(RGB_BLUE_PIN, blue);
}

//ADD TEMPERATURE READING TO ROLLING WINDOW
void addTemperatureReading(float temperature) {

  //Store the newest temperature at the current array position.
  temperatureWindow[temperatureWindowIndex] = temperature;

  //Move to the next array position.
  temperatureWindowIndex++;

  //If we reach the end of the 30-reading array,
  //return to position 0.
  //This creates the circular buffer.
  if (temperatureWindowIndex >= TEMP_WINDOW_SIZE) {
    temperatureWindowIndex = 0;
  }

  //Increase the number of collected readings until
  //the complete 30-reading window has been filled.
  if (temperatureReadingCount < TEMP_WINDOW_SIZE) {
    temperatureReadingCount++;
  }
}

//CALCULATE ROLLING TEMPERATURE MEAN
//Calculates the average temperature of all valid readings
//currently stored in the rolling temperature window. 
float calculateTemperatureMean() {

  //If no readings have been collected yet,
  //there is nothing to calculate.
  if (temperatureReadingCount ==0) {
    return 0.0;
  }

  //Stores the total of all temperature readings.
  float total = 0.0;

  //Add every valid temperature reading together.
  for (int i = 0; i < temperatureReadingCount; i++) {

    total += temperatureWindow[i];
  }

  //Mean = total of all readings divided
  //by the number of readings.
  return total / temperatureReadingCount;
}

//CALCULATE ROLLING TEMPERATURE STANDARD DEVIATION
//Standard deviation measures how spread out the
//temperature readings are around their average.
//
//Small standard deviation: 
//Temperature has remained relatively stable.
//
//Large standard deviation: 
//Temperature has been changing or fluctuating.
float calculateTemperatureStdDev(float mean) {

  //At least two readings are required for a useful
  //standard deviation calculation.
  if (temperatureReadingCount < 2) {
    return 0.0;
  }

  //Stores the total squared difference
  //between each reading and the mean.
  float squaredDifferenceTotal = 0.0;

  //Check every valid temperature in the window.
  for (int i = 0; i < temperatureReadingCount; i++) {

    //Calculate how far this reading is
    //from the rolling mean.
    float difference = 
      temperatureWindow[i] - mean;

      //Square the difference and add it tp the total.
      squaredDifferenceTotal +=
        difference * difference;
  }

  //Calculate the variance.
  //Variance is the average of all the
  //squared differences from the mean.
  float variance =
    squaredDifferenceTotal /
    temperatureReadingCount;

  //Standard deviation is the square root
  //of the variance.
  return sqrt(variance);
}

//CALCULATE TEMPERATURE RATE OF CHANGE
//Calculates how quickly the temperature has changed
//across the rolling temperature window.
//
//The result is expressed in degrees Celcius per minute.
//
//Postive value = temperature is rising.
//Negative value = temperature is falling.
//Value near zero = temperature is table.
float calculateTemperatureSlope() {

  //At least two valid readings are required
  //to calculate a rate of change.
  if (temperatureReadingCount < 2) {
    return 0.0;
  }

  //Stores the position of the oldest reading
  //currently available in the rolling window.
  int oldestIndex;

  //Before the 30-reading window is completely full,
  //the oldest reading is still at position 0.
  if (temperatureReadingCount < TEMP_WINDOW_SIZE) {

    oldestIndex = 0;
  } else {
    //Once the circular buffer is full,
    //temperatureWindowIndex points to the position
    //that will be replaced next.
    //
    //That position therefore currently contains
    //the oldest reading. 
    oldestIndex = temperatureWindowIndex;
  }

  //The newest reading is one position behind
  //temperatureWindowIndex.
  //
  //The modulo calculation safely wraps around
  //from position 0 back to position 29.
  int newestIndex = 
    (temperatureWindowIndex - 1 + TEMP_WINDOW_SIZE)
    % TEMP_WINDOW_SIZE;

  //Calculate the total temperature change
  //between the oldest and newest readings.
  float temperatureChange = 
    temperatureWindow[newestIndex] -
    temperatureWindow[oldestIndex]; 

  //Calculate how much time seperates the oldest
  //and newest readings.
  //
  //Each reading is collected every SNSOR_interval.
  //
  //Each reading is collected every SENSOR_INTERVAL.
  //For example: 
  //30 readings contain 29 intervals.
  //29 x 2 seconds = 58 seconds.
  float elapsedMinutes =
    ((temperatureReadingCount - 1) * SENSOR_INTERVAL)
    / 60000.0;

  //Protect against division by zeo.
  if (elapsedMinutes <= 0.0) {
    return 0.0;
  }

  //Rate of change = 
  //temperature change / elapsed time.
  //
  //Result is degrees Celsius per minute.
  return temperatureChange / elapsedMinutes;
}

//CALCULATE TEMPERATURE Z-SCORE
//The Z-score measures how far the current temperature
//is from the rolling mean. 
//
//It expresses that distance in standard deviations.
//
//Examples: 
//Z-score near 0 = close to the recent average.
//Positive Z-score = above the recent average.
//Negative Z-score = below the recent average.
//Large absolute Z-score = unusual compared with
//recent temperature behaviour.
float calculateTemperatureZScore(
  float currentTemperature,
  float mean,
  float standardDeviation
) {

  //If standard deviation is extremely small,
  //the temperature history has been almost 
  //perfectly stable.
  //
  //Dividing by zero would produce an invalid result,
  //so return 0 when there is effectively no variation.
  if (standardDeviation < 0.01) {
    return 0.0;
  }

  //Z-score formula: 
  //
  //         current temperature - mean
  // Z = ----------------------------------
  //             standard deviation
  //
  return
    (currentTemperature - mean) /
    standardDeviation;
}

//FUNCTION: getStateName
//Converts the FSM state into readable text.
//This text can be shown in Serial Monitor
//and published to the Adafruit IO dashboard.
const char* getStateName(SystemState state) {

  switch (state){

    case NORMAL: 
      return "NORMAL";

    case DOOR_OPEN: 
      return "DOOR_OPEN";

    case COOLING: 
      return "COOLING";

    case EXCURSION: 
      return "EXCURSION";

    case FAILSAFE: 
      return "FAILSAFE";

    default: 
      return "UNKNOWN";
  }
}

//FUNCTION: connectWiFi
//Connects the ESP32 to Wokwi-GUEST.
void connectWiFi() {

  Serial.print("Connecting to Wokwi WiFi");

  //Start the Wi-Fi conenction.
  //Channel 6 is used by Wokwi-GUEST.
  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD,
    6
  );

  //Keep checking until Wi-Fi connects.
  while (WiFi.status() != WL_CONNECTED) {

    delay(100);
    
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected!");

  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());
}

//FUNCTION: connectMQTT
//Conencts the ESP32 to Adafruit IO. 
void connectMQTT() {

  //Keep attempting connection until successful.
  while (!mqttClient.connected()) {

    Serial.print("Connecting to Adafruit IO MQTT...");

    //Give this ESP32 a unique MQTT client ID.
    String clientId =
      "ColdGuard-ESP32-" +
      String(random(0xffff), HEX);

  //Connect using:
  //client ID
  //Adafruit username
  //Adafruit AIO key
  if (
    mqttClient.connect(
      clientId.c_str(),
      AIO_USERNAME,
      AIO_KEY
    )
  ) {

    Serial.println("connected!");
  } else {

    Serial.print("failed, MQTT state = ");
    Serial.println(mqttClient.state());

    Serial.println("Retrying in 5 seconds...");

    delay(5000);
  }
 }
}

//EVENT TRACKING VARIABLES
//Stores the previous values so ColdGuard can detect
//when something has actually changed.
bool previousDoorOpen = false;
bool previousCoolingOn = false;
bool previousDoorAlarm = false;

//Prevents ColdGuard from creating fake "change"
//events immediately when the ESP32 first starts.
bool eventStateInitialised = false;

void setup() {
  //Start serial communication
  Serial.begin(115200);

  //Serial Monitor tells you ColdGuard
  //has started before it attempts Wi-Fi/MQTT.
  Serial.println("ColdGuard starting...");

  //Configure the DHT sensor.
  //DHT_PIN tells the library that the sensor is connected to GPIO 33.
  //DHTesp::DHT22 tells the library that the sensor type is a DHT22.
  dht.setup(DHT_PIN, DHTesp::DHT22);

  //Configure GPIO 34 as an input.
  //The ESP32 will read the analogue voltage
  //produced by the LDR module on this pin. 
  pinMode(LDR_PIN, INPUT);

  //Configure the relay control pin as an output.
  pinMode(RELAY_PIN, OUTPUT);

  //Cooling starts OFF.
  digitalWrite(RELAY_PIN, LOW);

  //CONFIGURE BUZZER PWM
  //
  //Configure ESP32 PWM channgel 0 to generate
  //a 2000 Hz signal for the passive buzzer.
  ledcSetup(
    BUZZER_CHANNEL,
    BUZZER_FREQUENCY,
    BUZZER_RESOLUTION
  ); 

  //Connect GPIO 25 to the PWM channel.
  ledcAttachPin(
    BUZZER_PIN,
    BUZZER_CHANNEL
  );

  //Start with the buzzer silent.
  //Duty cycle 0 = no PWM signal = no sound.
  ledcWrite(
    BUZZER_CHANNEL,
    0
  );

  //Configure all three RGB LED channels as outputs.
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);

  //Start with the LED turned off.
  setRGB(false, false, false);

  //Initialise LCD
  //Start communication with the LCD.
  lcd.init();

  //Turn the LCD blacklight.
  lcd.backlight();

  //Clear anything currently displayed.
  lcd.clear();

  //Start at column 0, row 0.
  lcd.setCursor(0, 0);

  //Display the ColdGuard name.
  lcd.print("ColdGuard");

  //Move to column 0, row 1.
  lcd.setCursor(0, 1);

  //Display startup status.
  lcd.print("Starting...");

  //WIFI + MQTT CONNECTION
  //CONNECT TO WOKWI WIFI
  //Connect the ESP32 to Wokwi-GUEST
  //simulated Wi-Fi network.
  connectWiFi();

  //CONFIGURE MQTT
  //Tell the MQTT client which MQTT broker
  //ColdGuard will communicate with.
  //MQTT_SERVER = io.adafruit.com
  //MQTT_PORT = 8883
  mqttClient.setServer(
    MQTT_SERVER,
    MQTT_PORT
  );

  //CONFIGURE TLS CONNECTION
  //Allow the Wokwi ESP32 to establish the TLS
  //connection without manually installing the
  //Adafruit server certificate.
  //This is convenient for the simulation.
  wifiClient.setInsecure();

  //Connect the MQTT client to Adafruit IO
  //using the username and AIO key defined
  //near the top of the program.
  connectMQTT();


  //Print a startup message once when the ESP32 starts.
  Serial.println("Sensors, RGB LED, relay, buzzer and LCD initialised.");
  Serial.println();
}

void loop() {

//Connect to MQTT client for Adafruit.
if (!mqttClient.connected()) {
  connectMQTT();
}

mqttClient.loop();

//Read both temperature nd humidity from the DHT22.
//The results are stored together in a TempandHumidity structure
//called "data".
TempAndHumidity data = dht.getTempAndHumidity();

//Read the analogue value from the photoresistor.
//on ESP32, analogRead() normally gives a value
//between 0 and 4095.
//The exact value depends on how much light reaches
//the photoresistor.
int lightValue = analogRead(LDR_PIN);

// if LDR value is below threshold
//we assume light is entering the fridge
//and therefore the door is open.
bool doorOpen = lightValue < LDR_THRESHOLD;

//SENSOR VALIDITY AND PLAUSIBILITY CHECK
//ColdGuard enters FAILSAFE if: 
//1. The DHT22 returns NaN / invalid data.
//2. Temperature is below -10C.
//3. Temperature is above 30C.
bool sensorFault = 
  isnan(data.temperature) ||
  isnan(data.humidity) ||
  data.temperature < PLAUSIBLE_MIN_TEMP ||
  data.temperature > PLAUSIBLE_MAX_TEMP;

//DOOR OPEN TIMER
//Check whether the fridge door is currently open.
if (doorOpen) {

  //if the door has only just opened,
  //start the timer.
  if (!doorTimerRunning) {

    //millis() returns the number of milliseconds
    //since the ESP32 started running.
    doorOpenedAt = millis();

    //Remember that the door timer is now active.
    doorTimerRunning = true;

    Serial.println("Door timer started.");
  }
} else {

  //The door has been closed,
  //so reset the timer.
  doorTimerRunning = false;

  //Reset the stored start time.
  doorOpenedAt = 0;
}

//DOOR ALARM CONTROL
//Start with the assumption that the
//door alarm is not active.
bool doorAlarm = false;

//Only calculate elapsed time if:
//1. The door is currently open
//2. The timer has already started
if (doorOpen && doorTimerRunning) {

  //Calculate how long the door has been open.
  //Example:
  //Current millis() = 70000
  //doorOpenedAt = 10000
  //70000 - 10000 = 60000 ms
  unsigned long doorOpenTime =
      millis() - doorOpenedAt;

      //If the door has remained open for at least
      //60 seconds, activate the door alarm.
      if (doorOpenTime >= DOOR_OPEN_LIMIT) {

        doorAlarm = true;
      }
}

//DOOR REMINDER CHIRP CONTROL
//Once the door has been open long enough for
//doorAlarm to become active, ColdGuard produces
//short reminder chirps instead of one continous alarm.
//
//This uses millis() rather than delay(), so sensor
//monitoring, MQTT and the FSM continue running.
if (doorAlarm) {

  //START A NEW CHIRP
  //Start a chirp if: 
  //1.A chirp is not already active.
  //2. This is the first chirp OR at least
  //10 seconds have passed since the previous chirp.
  if (
    !doorChirpActive &&
  (
    lastDoorChirpAt == 0 ||
    millis() - lastDoorChirpAt >= DOOR_CHIRP_INTERVAL
  )
  ) {

    //Remember that the buzzer should now be sounding.
    doorChirpActive = true;

    //Remember exactly when this chirp sounded.
    doorChirpStartedAt = millis();

    //Remember when the most recent chirp occurred.
    lastDoorChirpAt = millis();

    Serial.println("Door reminder chirp started.");

  }

  //END THE CURRENT CHIP
  //
  //Once the chirp has lasted 150 milliseconds,
  //turn the chirp state back off.
  if (
    doorChirpActive &&
    millis() - doorChirpStartedAt >= DOOR_CHIRP_DURATION
  ) {

    doorChirpActive = false;

    Serial.println("Door reminder chirp ended.");
  }
} else{

  //The alarm condition has cleared.
  //
  //Reset all chirp information so the next time
  //the door is left open ColdGuard starts fresh.
  doorChirpActive = false;
  doorChirpStartedAt = 0;
  lastDoorChirpAt = 0;
}

//HIGH TEMPERATURE DWELL TIMER
//Only monitor temperature excursions while: 
//1. The sensor is valid.
//2. The fridge door is closed.
if (!sensorFault && !doorOpen) {

  //Start the high-temperature timer when
  //temperature rises above the safe maximum.
  if (data.temperature > SAFE_MAX_TEMP) {

    if (!highTempTimerRunning) {

      highTempStartedAt = millis();
      highTempTimerRunning = true;

      Serial.println("High temperature timer started.");
    }
  } else {

    //Temperature has returned to the safe range,
    //so reset the high-temperature timer.
    highTempTimerRunning = false;
    highTempStartedAt = 0;
  }
} else {

  //Door open or sensor fault
  //do not count this time toward a temperature excursion.
  highTempTimerRunning = false;
  highTempStartedAt = 0;
}

//CALCULATE HIGH TEMPERATURE DURATION
//Start at zero.
unsigned long highTempDuration = 0;

//If the high-temperature is running,
//calculate how long the temperature has been high.
if (highTempTimerRunning) {

  highTempDuration =
    millis() - highTempStartedAt;
}

//CLOSED-LOOP COOLING CONTROL
//Do not automatically control cooling if
//there is a sensor fault.
if (!sensorFault) {

  //Only perform normal cooling control
  //while the fridge door is closed.
  if (!doorOpen) {

    //Cooling may switch ON after the temperature
    //has remained above the safe maximum
    //for the required dwell period.
    if (!coolingOn &&
        data.temperature >= COOLING_ON_TEMP &&
        highTempDuration >= COOLING_DWELL_TIME) {

          coolingOn = true;

          Serial.println("Cooling activated after dwell.");
        }

      //Hysteresis
    //Once cooling is active, keep it running until
    //temperature falls to 7.5C or lower.
    else if (coolingOn &&
             data.temperature <= COOLING_OFF_TEMP) {

              coolingOn = false;

              Serial.println("Cooling deactivated.");
             } 
            } else {

              //For now, force cooling OFF while
              //the fridge door is open.
              coolingOn = false;
             }   
} else {

  //Sensor data cannot be trusted.
  //Keep cooling OFF until FAILSAFE behaviour
  //is defined more fully.
  coolingOn = false;
}

//Send the value stored in coolingOn to rhe relay.
//coolingOn = true > HIGH > relay ON
//coolingOn = false > LOW > relay OFF
digitalWrite(RELAY_PIN, coolingOn ? HIGH : LOW);

//FINITE STATE MACHINE - STATE DECISION
//The FSM decides the current operating state
//before the outputs are controlled. 

//Sensor fault has highest priority.
if (sensorFault) {

  currentState = FAILSAFE;
} else if (data.temperature < SAFE_MIN_TEMP) {

  //Temperature below 2C is immediately unsafe.
  currentState = EXCURSION;
} else if (

  //Temperature above 8C for the full excursion
  //dwell perios becomes an EXCURSION.
  highTempTimerRunning &&
  highTempDuration >= EXCURSION_DWELL_TIME
) {

  currentState = EXCURSION;
} else if (doorOpen) {

//Door-open condition takes priority over
//normal cooling state.
  currentState = DOOR_OPEN;
} else if (coolingOn) {
  
  //Cooling system is currently active.
  currentState = COOLING;
} else {

  //Everything else is normal.
  currentState = NORMAL;
}

//If ColdGuard is not currently in EXCURSION,
//reset the blinking LED state.
if (currentState != EXCURSION) {

  excursionLedOn = false;
  lastLedBlink = millis();
}

//FSM RGB STATUS CONTROL
//The RGB LED now follows the current FSM state
//instead of checking the door separately.
switch (currentState) {

  case NORMAL: 

    //NORMAL = solid GREEN
    setRGB(false, true, false);

    //Reset excursion blinking ready for the next time.
    excursionLedOn = false;
    lastLedBlink = millis();

    break;

  case DOOR_OPEN: 

    //DOOR_OPEN = solid AMBER
    //Red + Green creates amber/yellow.
    setRGB(true, true, false);

    //Reset excursion blinking.
    excursionLedOn = false;
    lastLedBlink = millis();

    break;

  case COOLING: 

  //COOLING = solid AMBER
  setRGB(true, true, false);

  //Reset excursion blinking.
  excursionLedOn = false;
  lastLedBlink = millis();

  break;

  case EXCURSION: 

    //EXCURSION = BLINKING RED
    //Do NOT use delay() here because that would stop
    //sensor monitoring, MQTT and control logic.
    if (millis() - lastLedBlink >= LED_BLINK_INTERVAL) {

      //Remember when the LED change occurred.
      lastLedBlink = millis();

      //Toggle the red LED state.
      //OFF -> ON
      //ON -> OFF
      excursionLedOn = !excursionLedOn;
    }

    //Turn only the red channel on/off.
    setRGB(excursionLedOn, false, false);

    break;

  case FAILSAFE: 

    //FAILSAFE = solid PURPLE
    //Red + Blue creates purple. 
    setRGB(true, false, true);

    //Reset excursion blinking.
    excursionLedOn = false;
    lastLedBlink = millis();

    break;
}

//FINAL BUZZER CONTROL
//
//The passive buzzer is controlled using
//the ESP32 PWM/LEDC hardware.
//
//Duty cycle 128 on an 8-bit PWM channel is
//approximately a 50% duty cycle and produces
//an audible 2000 Hz tone.
//
//EXCURSION = continous alarm.
//DOOR_OPEN = short reminder chirps.

if (currentState == EXCURSION) {

  //Critical temperature excursion: 
  //keep the buzzer continously sounding.
  ledcWrite(
    BUZZER_CHANNEL,
    128
  );
} else if (
  currentState == DOOR_OPEN &&
  doorChirpActive
) {

  //Door reminder: 
  //sound only during the short chirp.
  ledcWrite(
    BUZZER_CHANNEL,
    128
  );
} else {

  //No active alarm: 
  //stop the PWM signal.
  ledcWrite(
    BUZZER_CHANNEL,
    0
  );
}

//Recent EVENTS
//On the first loop, simply remember the current
//states without publishing an event.
if (!eventStateInitialised) {

  previousDoorOpen = doorOpen;
  previousCoolingOn = coolingOn;
  previousDoorAlarm = doorAlarm;

  eventStateInitialised = true;
} else {

  //DOOR EVENT
  //Only run this if the door state has changed.
  if (doorOpen != previousDoorOpen) {

    if (doorOpen) {

      publishEvent("Door opened");
    } else {

      publishEvent("Door closed");
    }

    //Remember the new door state.
    previousDoorOpen = doorOpen;
  }

//COOLING EVENT
//Only run this if cooling has changed ON/OFF.
if (coolingOn != previousCoolingOn) {

  if (coolingOn) {

    publishEvent("Cooling Activated");
  } else {

    publishEvent("Cooling deactivated");
  }

  //Remember the new cooling state.
  previousCoolingOn = coolingOn;
}

//DOOR ALARM EVENT
//Only run this if the door alarm state changes.
if (doorAlarm != previousDoorAlarm) {

  if (doorAlarm) {

    publishEvent("Door alarm activated");
  } else {

    publishEvent("Door alarm cleared");
  }

  //Remember the new alarm state.
  previousDoorAlarm = doorAlarm;
}
}

if (millis() - lastSensorUpdate >= SENSOR_INTERVAL) {

lastSensorUpdate = millis();

//EDGE INTELLIGENCE DATA COLLECTION
//Only valid/plausible temperature readings are allowed
//into the rolling history,
//
//FAILSAFE readings are excluded because an implausible
//sensor value should not contaminate the statistical model.
if (!sensorFault) {

  addTemperatureReading(data.temperature);
}

//Calculate the current rolling-window statistics.
float temperatureMean = calculateTemperatureMean();

float temperatureStdDev = 
  calculateTemperatureStdDev(temperatureMean);

//Calculate the temperature rate of change
//across the current rolling window.
float temperatureSlope = 
  calculateTemperatureSlope();

//Calculate how statistically unusual the
//current temperature is compared with the
//recent rolling remperature history.
float temperatureZScore = 
  calculateTemperatureZScore(
    data.temperature,
    temperatureMean,
    temperatureStdDev
  );

//EDGE INTELLIGENCE CLASSIFICATION
//The statistical model classifies current
//temperature behaviour as: 
//STABLE = normal recent behaviour. 
//DRIFTING = unusual statistical behaviour or
//a meaningful temperature trend. 
//CRITICAL = ColdGuard has entered a safety-critical 
//FSM state. 
//
//The edge-intelligence model needs a complete
//30-reading window before its statistical results
//are considered reliable.
//
//FAILSAFE and EXCURSION have highest priority.
//These represent safety-critical conditions that
//have already been identified by the main FSM.
if (
  currentState == EXCURSION ||
  currentState == FAILSAFE
) {

  edgeClassification = "CRITICAL";
} 

//Only perform statistical classification once
//the complete 30-reading rolling window is available.
else if (temperatureReadingCount < TEMP_WINDOW_SIZE) {

  edgeClassification = "WARMING_UP";
}

//Once the window is full, check whether the
//temperature behaviour is statistically unusual.
else if (

  //fabs() gives the absolute value of a decimal number.
  //This means both positive and negative changes matter.
  //For example: 
  //Z-score +2.3C = unusual increase
  //Z-score -2.3C = unusual decrease
  //
  //Slope +0.8 = temperature rising
  //Slope -0.8 = temperature falling
  //
  //Classify the temperature as DRIFTING when: 
  //1.The temperature is above the normal safe maximum,
  //but the FSM has not yet declared an EXCURSION.
  //
  //2.The Z-score shows statistcally unusual behaviour.
  //
  //3.The temperature is changing faster than our
  //configured slope threshold.
  data.temperature > SAFE_MAX_TEMP ||
  fabs(temperatureZScore) >= Z_SCORE_DRIFT_THRESHOLD ||
  fabs(temperatureSlope) >= SLOPE_DRIFT_THRESHOLD
) {

  edgeClassification = "DRIFTING";
} else {

  //If no abnormal behaviour was detected,
  //the recent temperature behaviour is considered stable.
  edgeClassification = "STABLE";
}

//IMMDEDIATE EDGE CLASSIFICATION UPDATE
//
//Normally ColdGuard sends dashboard telemtry every
//15 seconds.
//However, the edge classification is important enough
//that a change should appear on the dashboard immediately.
//
//Only publish when the value actually changes.
//This prevents unneccessary MQTT traffic.
if (
  mqttClient.connected() &&
  edgeClassification != lastPublishedEdgeClassification
) {

  mqttClient.publish(
    anomalyTopic.c_str(),
    edgeClassification.c_str()
  );

  //Remember what was sent so the same value
  //is not repeatedly published every 2 seconfs.
  lastPublishedEdgeClassification = edgeClassification;

  Serial.print(
    "MQTT Edge Classification change published: "
  );
  Serial.println(edgeClassification);
}

//UPDATE LCD DISPLAY
//Clear the LCD before writing new information.
//This is simple for now. 
//This can be improved later on to avoid flickering.
lcd.clear();

//LCD Row 1
//Start at column 0, row 0..
lcd.setCursor(0, 0);

//Display temperature.
lcd.print("T:");
lcd.print(data.temperature, 1);
lcd.print("C");

//Add some spacing.
lcd.print(" H:");

//Display humidity.
lcd.print(data.humidity, 0);
lcd.print("%");

//LCD ROW 2
// Start at column 0, row 1.
lcd.setCursor(0, 1);

//Display door state.
if (doorOpen) {

  lcd.print("Door:OPEN ");
} else {

  lcd.print("Door:CLOSED");
}

//Display cooling indicator.
if (coolingOn) {

  lcd.print(" C");
} else {

  lcd.print(" -");
}

Serial.print("Edge Window: ");
Serial.print(temperatureReadingCount);
Serial.print("/");
Serial.println(TEMP_WINDOW_SIZE);

//Display the rolling statistical calculations.
Serial.print("Rolling Mean: ");
Serial.print(temperatureMean, 2);
Serial.println(" C");

Serial.print("Standard Deviation: ");
Serial.print(temperatureStdDev, 2);
Serial.println(" C");

//Display the calculated temperature rate of change.
Serial.print("Temperature Slope: ");
Serial.print(temperatureSlope, 2);
Serial.println(" C/min");

//Display the current temperature Z-score.
Serial.print("Temperature Z-Score: ");
Serial.println(temperatureZScore, 2);

//Display the final edge-intelligence classification.
Serial.print("Edge Classification: ");
Serial.println(edgeClassification);

//print the temperature value.
//data.temperature contains the temperature in degrees Celsius.
// The 1 displays the value to one decimal place. 
Serial.print("Temperature: ");
Serial.print(data.temperature, 1);
Serial.println (" C");

//print the humidity unit and move to the next line.
Serial.print("Humidity: ");
Serial.print(data.humidity, 1);
Serial.println(" %");

//Print LDR Value
Serial.print("LDR value: ");

//Print the raw analogue value coming
//from GPIO 34.
Serial.println(lightValue);

//Print Door State
Serial.print("Door: ");

if (doorOpen) {

  //This runs when the LDR detects enough light
  //to indicate that the fridge door is open.
  Serial.println("OPEN");
} else {

  //This runs when the LDR is dark enough
  //to indicate that the fridge door is closed.
  Serial.println("CLOSED");
}

//PRINT DOOR TIMER STATUS
Serial.print("Door Alarm: ");

if (doorAlarm) {

  Serial.println("ACTIVE");
} else {

  Serial.println("OFF");
}

//PRINT COOLING STATUS
Serial.print("Cooling: ");

if (coolingOn) {

  Serial.println("ON");
} else {

  Serial.println("OFF");
}

//PRINT FSM SYSTEM STATE
Serial.print("System State: ");
Serial.println(getStateName(currentState));

//PRINT RGB LED STATE
Serial.print("Status LED: ");

switch (currentState) {

  case NORMAL: 

    Serial.println("GREEN");
    break;

  case DOOR_OPEN: 

    Serial.println("AMBER");
    break;

  case COOLING: 

    Serial.println("AMBER");
    break;

  case EXCURSION: 

    Serial.println("RED");
    break;

  case FAILSAFE: 

    Serial.println("PURPLE");
    break;
}

//Separator to make each sensor reading easier to see.
Serial.println("----------------");

} // CLOSES 2-SECOND SENSOR/LCD/SERIAL BLOCK

//MQTT DASHBOARD PUBLISH TIMER
//Publish dashboard telemetry every 15 seconds.
if (millis() - lastMqttPublish >= MQTT_PUBLISH_INTERVAL) {

  lastMqttPublish = millis();

  //Publish Temperature to Adafruit IO.
//Convert the temperature value into text.
//Example: 9.0 becomes "9.0"
char temperatureValue[8];

dtostrf(
  data.temperature,
  1,
  1,
  temperatureValue
);

//Publish the temperature to Adafruit
//temperature feed.
bool publishSuccess =
  mqttClient.publish(
    temperatureTopic.c_str(),
    temperatureValue
  );

//Print whether MQTT publishing worked.
if (publishSuccess) {

  Serial.print("MQTT Temperature published: ");
  Serial.println(temperatureValue);
} else {

  Serial.println("MQTT Temperature publish FAILED");
}

//Publish Humidity to Adafruit IO.
//Convert the humidity value into text.
//Example: 80.0 becomes "80.0"
char humidityValue[10];

dtostrf(
  data.humidity,
  1,
  1,
  humidityValue
);

//Publish humidity to the Adafruit
//humidity feed.
bool humidityPublishSuccess =
  mqttClient.publish(
    humidityTopic.c_str(),
    humidityValue
  );

//Print whether humidity publishing worked.
if (humidityPublishSuccess) {

  Serial.print("MQTT Humidity publish: ");
  Serial.println(humidityValue);
} else {

  Serial.println("MQTT Humidity publish FAILED");
}

//PUBLISH DOOR STATUS TO ADAFRUIT IO
//Convert the door state into a simple number
// 1 = door open
// 0 = door closed
const char* doorValue;

if (doorOpen) {

  doorValue = "1";

} else {

  doorValue = "0";
}

//Publish door status to Adafruit IO.
bool doorPublishSuccess = 
  mqttClient.publish(
    doorTopic.c_str(),
    doorValue
  );

//Print whether door status publishing worked.
if (doorPublishSuccess) {

  Serial.print("MQTT Door published: ");
  Serial.println(doorValue);
} else {

  Serial.println("MQTT door publish FAILED");
}

//PUBLISH COOLING / RELAY STATUS TO ADAFRUIT IO
//Convert cooling state into a simple number.
// 1 = cooling ON
// 0 = cooling OFF
const char* relayValue;

if (coolingOn) {

  relayValue = "1";
} else {

  relayValue = "0";
}

//Publish the actual cooling/relay state.
bool relayPublishSuccess = 
  mqttClient.publish(
    relayTopic.c_str(),
    relayValue
  );

//Print whether relay status publishing worked.
if (relayPublishSuccess) {

  Serial.print("MQTT Relay published: ");
  Serial.println(relayValue);
} else {

  Serial.println("MQTT Relay publish FAILED");
}

//Convert the FSM state into readable text
//for the Adafruit IO dashboard.
const char* systemState = 
  getStateName(currentState);

//PUBLISH SYSTEM STATE TO ADAFRUIT IO
bool statePublishSuccess = 
  mqttClient.publish(
    stateTopic.c_str(),
    systemState
  );

//Publish the latest edge-intelligence classfication
//to the Adafruit IO anomaly feed.
//Possible values: 
//WARMING_UP
//STABLE
//DRIFTING
//CRITICAL
mqttClient.publish(
  anomalyTopic.c_str(),
  edgeClassification.c_str()
);

//Keep the change-detection variable synchronized
//with the value sent during regular telemetry. 
lastPublishedEdgeClassification = edgeClassification;

Serial.print("MQTT Edge Classification published: ");
Serial.println(edgeClassification);

if (statePublishSuccess) {

  Serial.print("MQTT State published: ");
  Serial.println(systemState);
} else {
  Serial.println("MQTT State publish FAILED");
}

//SYSTEM FAULT STATUS
//The dashboard fault indicator follows
//the FSM sensor fault check.
bool systemFault = sensorFault;

//COVERT FAULT STATE FOR MQTT
// 1 = fault detected
// 0 = no fault
const char* faultValue;

if (systemFault) {

  faultValue = "1";
} else {

  faultValue = "0";
}

//PUBLISH SYSTEM FAULT to ADAFRUIT IO
bool faultPublishSuccess =
  mqttClient.publish(
    faultTopic.c_str(),
    faultValue
  );

if (faultPublishSuccess) {

  Serial.print("MQTT Fault published: ");
  Serial.println(faultValue);
} else {

  Serial.println("MQTT Fault publish FAILED");
}

//Seperator to make each sensor reading easier to see.
Serial.println("----------------");

}//Closes 15-second MQTT block

}//Closes loop()
