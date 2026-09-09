//include the DHTesp Library
//This library allows the ESP32 to communicate with the DHT22
//temperature and humidity sensor.
#include <DHTesp.h>

//Include the I2C LCD library.
//This allows the ESP32 to display information
//on the 16x2 LCD screen.
#include <LiquidCrystal_I2C.h>

//SENSOR UPDATE TIMER
//Stores the last time sensor information
//was printed to the Serial Monitor.
unsigned long lastSensorUpdate = 0;

//How often we want to print sensor information.
//2000 milliseconds = 2 seconds.
const unsigned long SENSOR_INTERVAL = 2000;

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

//DOOR TIMER SETTINGS
//Maximum time the fridge door can remain open
//before the buzzer alarm activates.
//60000 milliseconds = 60 seconds.
const unsigned long DOOR_OPEN_LIMIT = 60000;

//Stores the time when the door was first opened.
unsigned long doorOpenedAt = 0;

//Keeps track of whether the door timer is active.
bool doorTimerRunning = false;

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

void setup() {
  //Start serial communication
  Serial.begin(115200);

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

  //Configure the buzzer pin as an output.
  pinMode(BUZZER_PIN, OUTPUT);

  //Buzzer starts turned off.
  digitalWrite(BUZZER_PIN, LOW);

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

  //Print a startup message once when the ESP32 starts.
  Serial.println("ColdGuard starting...");
  Serial.println("Sensors, RGB LED, relay, buzzer and LCD initialised.");
  Serial.println();
}

void loop() {

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

//If the door alarm is active,
//turn the buzzer on.
if (doorAlarm) {

  digitalWrite(BUZZER_PIN, HIGH); 
} else {

  //Otherwise keep the buzzer off.
  digitalWrite(BUZZER_PIN, LOW);
}


//CLOSED-LOOP COOLING CONTROL
//Only perform automatic cooling control
//while fridge door is closed.
if (!doorOpen) {

  //If cooling is OFF and temperature reaches
  //8.5°C or higher, activate cooling.
  if (!coolingOn &&
      data.temperature >= COOLING_ON_TEMP) {

        coolingOn = true;

        Serial.println("Cooling activated.");
      }

      //If cooling is already ON and temperature
      //falls to 7.5°C or lower, deactivate cooling.
      else if (coolingOn &&
               data.temperature <= COOLING_OFF_TEMP) {

                coolingOn = false;

                Serial.println("Cooling deactivated.");
               }
}

//Send the value stored in coolingOn to rhe relay.
//coolingOn = true > HIGH > relay ON
//coolingOn = false > LOW > relay OFF
digitalWrite(RELAY_PIN, coolingOn ? HIGH : LOW);

//RGB Status LOGIC
if (doorOpen) {

  //Door open = AMBER.
  //Amber is created by turning on:
  //Red + Green. 
  //Blue remains off.
  setRGB(true, true, false);
} else {

  //Door closed / normal = GREEN.
  setRGB(false, true, false);
}

if (millis() - lastSensorUpdate >= SENSOR_INTERVAL) {

lastSensorUpdate = millis();
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

//PRINT LED STATE
Serial.print("Status LED: ");

if (doorOpen) {

  Serial.println("AMBER");
} else{

  Serial.println("GREEN");
}

//Seperator to make each sensor reading easier to see.
Serial.println("----------------");
}
}