// LCD library
#include <LiquidCrystal_I2C.h>

// DHT library
#include <DHT.h>

// required libraries for BMP280
#include <Wire.h>
#include <Adafruit_Sensor.h> 
#include <Adafruit_BMP280.h>

// Defining Pins
#define DHTPIN 9              // pin connected to the DHT sensor
#define DHTTYPE DHT22         // DHT sensor type 
#define RAIN_SENSOR_PIN_1 A1  // analog pin connected to the Rain Drop Sensor 1
#define RAIN_SENSOR_PIN_2 A2  // analog pin connected to the Rain Drop Sensor 2
#define MQ2_PIN A0            // analog pin connected to the MQ2 Sensor

const int buzzerPin = 13;     // buzzer pin
const int numSensors = 7;     // number of pins used for IR flame sensors 

int flamePins[numSensors] = {2, 3, 4, 5, 6, 7, 8}; // digital pins connected to the flame sensor modules

// declaration of variables and constants for Encoder module

const int sensorPin = 12;
double timeLap,step,rotation;
unsigned long startTime, endTime;
float windSpeed , showWindSpeed;

// creating objects 

LiquidCrystal_I2C lcd(0x3F, 16, 4);   // Create LCD object (4 rows, 16 chars in each)
DHT dht(DHTPIN, DHTTYPE);             // Create DHT22 object
Adafruit_BMP280 bmp;                  // Create BMP280 object

// function to calculate altitude using BMP280
double calculateAltitude (float pressure)
{
  double exponent = 1.0 / 5.255;
  float p0 = 1013.25;           // sea level pressure
  double altitude = 44330.0 * (1 - pow(pressure / p0, exponent));
  return altitude;
}

// function to calculate the wind speed
float windSpeedCal () 
{
  Serial.println("Funtion Calling");
  startTime = millis ();
  endTime = startTime + 1000;
  while (endTime >= millis ())
  {
    if (digitalRead(sensorPin) == 1) 
    {
      if (digitalRead(sensorPin) == 0 &&  millis () <= endTime)
      {
        step = step + 1;
        //print values in serial monitor for testing
        Serial.print("step: ");
        Serial.println(step);
        if (step==8)
        {
          rotation = rotation +1;
          //print values in serial monitor for testing
          Serial.print("rotation: ");
          Serial.println(rotation);
          windSpeed = ( 12.5 * rotation )*0.36;
          step = 0;
        }
      }
    }      
  }
  rotation  = 0;
  timeLap = timeLap + 1;
  //print values in serial monitor for testing
  Serial.print("time Lap : ");
  Serial.println(timeLap);
  step = 0;
  //print values in serial monitor for testing
  Serial.print("rotation: ");
  Serial.println(rotation);
  //print calculated wind speed in serial monitor for testing
  Serial.print("Wind  Speed : ");
  Serial.println(windSpeed);
  Serial.println("Funtion ending");
  return windSpeed;
}

// function to display values on the LCD
void display(float temperature, float humidity, float pressure, int altitude, float showWindSpeed, float mq2Percentage, String weather)
{
  //define a variable to alternate displaying THPA and WSR values
  //THPA - temperature,humidity,pressure,altitude
  //WSR - windspeed,smoke,rain

  bool showTHPA = true;

  if (showTHPA) 
  {
    //display temperature
    lcd.print("Temp: ");
    lcd.print(temperature);
    lcd.print(" C");

    //display humidity
    lcd.setCursor(0, 1);
    lcd.print("Hmdty:");
    lcd.print(humidity);
    lcd.print("%");

    //display pressure
    lcd.setCursor(-4, 2);
    lcd.print("Press:");
    lcd.print(pressure);
    lcd.print("hPa");

    //display altitude
    lcd.setCursor(-4,3);
    lcd.print("Altitude:");
    lcd.print(altitude);
    lcd.print("m");
  } 
  else 
  {
    //display wind speed
    lcd.print("Wind Spd:");
    lcd.print(showWindSpeed);
    lcd.print("km/h");

    //display smoke
    lcd.setCursor(0, 1);
    lcd.print("Smoke :");
    lcd.print(mq2Percentage);
    lcd.print("%");

    //display rain status
    lcd.setCursor(-4, 2);
    lcd.print("Rain: ");
    lcd.print(weather);
  }

  //Alternate displaying THPA and WSR values for every 3 seconds
  if ((millis() / 1000) % 3 == 0) 
  {
    showTHPA = !showTHPA; //change to showTHPA = false
  }
  windSpeed = 0;
}

void setup()
{
  Serial.begin(115200);
  Serial.print("setup");

  //Initialize the LCD
  lcd.backlight();        // Turn on the backlight
  lcd.init();             // start the lcd object
  lcd.clear();            
  lcd.setCursor(2,1);     //set cursor to top left corner
  lcd.print("LOADING ");  //print the text to the lcd

  //create a loading screen effect on the LCD display.
  for (int i = 0; i <= 100; i++)
  {
    lcd.setCursor(10,1);
    if (i<100) 
    {
      lcd.print(" ");   //print a space if the percentage < 100 (two digits)
    }
    if (i<10) 
    {
      lcd.print("  ");   //print 2 spaces if the percentage < 10 (one digit)
    }
    lcd.print(i);
    lcd.print("%");
    delay(100);           
  }

  //start and initiate the dht object
  dht.begin();

  //check whether BMP280 sensor is connected  
  Serial.println("BMP280 Sensor Test");
  if (!bmp.begin(0x76))   // Initialize with default address 0x76
  { 
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("BMP 280 is not working...");
  }

  // Default settings from BMP280 datasheet
  bmp.setSampling
  (
    Adafruit_BMP280::MODE_NORMAL,     // Operating Mode
    Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling 
    Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling 
    Adafruit_BMP280::FILTER_X16,      // Filtering
    Adafruit_BMP280::STANDBY_MS_500   // Standby time
  );

  pinMode(RAIN_SENSOR_PIN_1, INPUT); //configure A1 as an input pin 
  pinMode(RAIN_SENSOR_PIN_2, INPUT); //configure A2 as an input pin
  
  pinMode(buzzerPin, OUTPUT);        //configure D13 as an output pin

  pinMode(sensorPin, INPUT_PULLUP);  //configure D12 as an input pin

  //Loop to configure digital pins that connected to the IR Flame module as input pins
  for (int i = 0; i < numSensors-2; i++) 
  {
    pinMode(flamePins[i], INPUT);
  }

}

void loop()
{
  //DHT22 Readings
  float temperature = dht.readTemperature();  // Read temperature
  float humidity = dht.readHumidity();        // Read humidity

  //print values in serial monitor for testing
  Serial.print("temperature :");
  Serial.println(temperature);
  Serial.print("humidity :");
  Serial.println(humidity);

  //BMP280 Readings
  float bmpTemperature = bmp.readTemperature(); // Read temperature
  float pressure = bmp.readPressure()/100; // Read pressure in hPa

  //print values in serial monitor for testing
  Serial.print("BMP280 Temperature: ");
  Serial.print(bmpTemperature);
  Serial.println(" °C");
  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");

  //calculate altitude using the function
  double altitude = calculateAltitude(pressure);

  //Rain Drop sensor Readings
  int rainValue1 = analogRead(RAIN_SENSOR_PIN_1);
  int rainValue2 = analogRead(RAIN_SENSOR_PIN_2);

  //print values in serial monitor for testing
  Serial.print("rain 1: ");
  Serial.println(rainValue1);
  Serial.print("rain 2: ");
  Serial.println(rainValue2); 

  //the text to be displayed
  String weather;

  //control the buzzer according to the raindrop values
  if (rainValue1 < 700 && rainValue2 < 700) 
  {
    weather="RAINING";
    digitalWrite(buzzerPin, HIGH);  // Turn on the buzzer
    delay(500);                     // Delay for 500 milliseconds
    digitalWrite(buzzerPin, LOW);   // Turn off the buzzer
    delay(500);                     // Delay for 500 milliseconds
    digitalWrite(buzzerPin, HIGH);  // Turn on the buzzer
    delay(1000);                    // Delay for 1000 milliseconds
    digitalWrite(buzzerPin, LOW);   // Turn off the buzzer
  } 
  else 
  {
    weather="NORMAL";
    digitalWrite(buzzerPin, LOW);   // Turn off the buzzer
  }

  //MQ2 Reading
  int mq2Value = analogRead(MQ2_PIN);

  //Calculate MQ2 value as a percentage
  float mq2Percentage = (mq2Value / 1000.0) * 100.0;

  //print values in serial monitor for testing 
  Serial.print("MQ2 value :");
  Serial.println(mq2Value);
  Serial.print("MQ2 valve %:");
  Serial.println(mq2Percentage);

  //control the buzzer according to the MQ2 value (turns on if it exceeds the threshold)
  if (mq2Percentage > 17) 
  {
    digitalWrite(buzzerPin, HIGH);  // Turn on the buzzer
    delay(500);                     // Delay for 500 milliseconds
    digitalWrite(buzzerPin, LOW);   // Turn off the buzzer
    delay(500);                     // Delay for 500 milliseconds
    digitalWrite(buzzerPin, HIGH);  // Turn on the buzzer
    delay(1000);                    // Delay for 1000 milliseconds
    digitalWrite(buzzerPin, LOW);   // Turn off the buzzer
  }

  //control the buzzer according to the Readings of the 5 way IR Sensor
  for (int i = 0; i < numSensors -2; i++) 
  {
    int flameValue = digitalRead(flamePins[i]);
    if (flameValue == HIGH) 
    {
      //print the detected sensor in serial monitor for testing
      Serial.print("Flame detected at sensor ");
      Serial.println(i + 1);
      
      //Activate the buzzer
      digitalWrite(buzzerPin, HIGH);  // Turn on the buzzer
      delay(500);                     // Delay for 500 milliseconds
      digitalWrite(buzzerPin, LOW);   // Turn off the buzzer
      delay(500);                     // Delay for 500 milliseconds
      digitalWrite(buzzerPin, HIGH);  // Turn on the buzzer
      delay(1000);                    // Delay for 1000 milliseconds
      digitalWrite(buzzerPin, LOW);   // Turn off the buzzer
    } 
    else 
    {
      Serial.print("No flame detected at sensor ");
      Serial.println(i + 1);
    }
  }

  //control the buzzer according to the Readings of the 1 way IR Sensors
  for (int i = 5; i < 7 ; i++) 
  {
    int flameValue = digitalRead(flamePins[i]);
    if (flameValue == LOW) 
    {
      //print the detected sensor in serial monitor for testing
      Serial.print("Flame detected at sensor ");
      Serial.println(i + 1);

      //Activate the buzzer
      digitalWrite(buzzerPin, HIGH);  // Turn on the buzzer
      delay(500);                     // Delay for 500 milliseconds
      digitalWrite(buzzerPin, LOW);   // Turn off the buzzer
      delay(500);                     // Delay for 500 milliseconds
      digitalWrite(buzzerPin, HIGH);  // Turn on the buzzer
      delay(1000);                    // Delay for 1000 milliseconds
      digitalWrite(buzzerPin, LOW);   // Turn off the buzzer
    }
    else 
    {
      Serial.print("No flame detected at sensor ");
      Serial.println(i + 1);
    }
  }

  //calculate the wind speed using the function
  showWindSpeed = windSpeedCal ();

  //display values on the LCD using the function
  lcd.clear();
  lcd.setCursor(0, 0);
  display(temperature, humidity, pressure, altitude, showWindSpeed, mq2Percentage, weather);
}