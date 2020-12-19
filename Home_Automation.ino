#include "DHT.h"   // header file of temprature and humidity sensor
#include <ESP8266WiFi.h>  //header file of node mcu esp8266
#include "Adafruit_MQTT.h" // header file of adafruit
#include "Adafruit_MQTT_Client.h" //header file of mqtt client

#define DHTPIN D5  // dht sensor connected to pin D5
#define DHTTYPE DHT22  // if your sensor is DHT11,then change DHT22 to DHT11  

#define bulb1            D0
#define fan              D1
#define bulb2            D2
#define bulb3            D3

////////////////////// pins of ultrasonic sensor
#define echopin  D6 
#define trigpin  D7
//////////////////////

#define WLAN_SSID       "Python"             // Your SSID
#define WLAN_PASS       "python##123"        // Your password

#define AIO_SERVER      "io.adafruit.com"    //Adafruit Server
#define AIO_SERVERPORT  1883                   
#define AIO_USERNAME    "***********"            // Username,of adafruit
#define AIO_KEY         "************************"   // Auth Key, in adafruit


DHT dht(DHTPIN, DHTTYPE);

int maximumRange = 50;
int duration, dis;

//WIFI CLIENT
WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

 // publish means, Node MCU upload the reading to adafruit using MQTT protocol,in my case temperature, humidity, distance you can add more sensors to the project.
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish distance = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/distance");

 // suscribe means, adafruit give instruction to Node MCU or control the home appliances, you can also add more relays.
Adafruit_MQTT_Subscribe Bulb1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/bulb1"); 
Adafruit_MQTT_Subscribe Fan = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/fan");
Adafruit_MQTT_Subscribe Bulb2 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/bulb2");
Adafruit_MQTT_Subscribe Bulb3 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/bulb3");

// it display the message in serial monitor which is send by adafruit (basically it send by me through adafruit) 
Adafruit_MQTT_Subscribe Display1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Display");

void MQTT_connect();

void setup() {
  Serial.begin(115200);
  dht.begin();
  pinMode(bulb1, OUTPUT);
  pinMode(fan, OUTPUT);
  pinMode(bulb2, OUTPUT);
  pinMode(bulb3, OUTPUT);
  pinMode (trigpin, OUTPUT);
  pinMode (echopin, INPUT); 
  
  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); 
  Serial.println(WiFi.localIP());
 
  mqtt.subscribe(&Bulb1);
  mqtt.subscribe(&Fan);
  mqtt.subscribe(&Bulb2);
  mqtt.subscribe(&Bulb3);
  mqtt.subscribe(&Display1);
}

void loop() {
 
  MQTT_connect(); // it help to connect Node MCU to MQTT 
  
  float h = dht.readHumidity(); // read humidity from sensor
  float t = dht.readTemperature(); //read temperature from sensor

  ////////////////this block of code read the distance by ultrasonic sensor and publish to adafruit/////////////// 
  digitalWrite(trigpin,LOW);
  delayMicroseconds(2);   
  digitalWrite(trigpin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin, LOW);  
  duration=pulseIn (echopin,HIGH);
  dis= duration/58.2;
  ////////////////////////////////////////////////////////////////////////////
  if (! temperature.publish(t)) {                     //Publish temperature to Adafruit
      Serial.println(F("Failed"));
    } 
  if (! humidity.publish(h)) {                        //Publish humidity to Adafruit
      Serial.println(F("Failed"));
    }

  if (! distance.publish(dis)) {                      //Publish distance to Adafruit
      Serial.println(F("Failed"));
    }
    
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(20000))) {
    if (subscription == &Bulb1) {
      Serial.print(F("Got: "));
      Serial.println((char *)Bulb1.lastread);
      int Bulb1_State = atoi((char *)Bulb1.lastread);
      digitalWrite(bulb1, Bulb1_State); // turn on/off bulb through adafruit
      
    }
    if (subscription == &Fan) {
      Serial.print(F("Got: "));
      Serial.println((char *)Fan.lastread);
      int Fan_State = atoi((char *)Fan.lastread);
      digitalWrite(fan, Fan_State); // turn on/off fan through adafruit
    }
    if (subscription == &Bulb2) {
      Serial.print(F("Got: "));
      Serial.println((char *)Bulb2.lastread);
      int Bulb2_State = atoi((char *)Bulb2.lastread);
      digitalWrite(bulb2, Bulb2_State);  // turn on/off bulb through adafruit
    } 

    if (subscription == &Bulb3) {
      Serial.print(F("Got: "));
      Serial.println((char *)Bulb3.lastread);
      int Bulb3_State = atoi((char *)Bulb3.lastread);
      digitalWrite(bulb3, Bulb3_State);  // turn on/off bulb through adafruit
    } 

    if (subscription == &Display1) {
      Serial.print(F("Message: "));
      Serial.println((char *)Display1.lastread); // display message in serial monitor which is publish by adafruit
    }  
  }
}

void MQTT_connect() {
  int8_t ret;

  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000); 
    retries--;
    if (retries == 0) {
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}
