#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <speech-recognition-grow-tech_inferencing.h>
#include "driver/i2s.h"

  //WiFi Setup 
  WiFiClient client;
  char ssid[]="o2-WLAN17"; 
  char pass[]="EndlichWLAN";

  //MQTT Setup
  PubSubClient mqttClient(client);                                  //Initialize MQTT Client
  const char* server = "24.134.0.25";                              //Change this to current server  
  int port = 1883;                                    
  char mqttUserName[] = "user";                                     //Add if necessary 
  char mqttPass[] = "0152a729d42683a7fc8b461d2fc44a7e72786f61";     //Add if necessary 
  #define DEVICE_ID 4
  char subTopic[] = "growtech/control";
  char pubTopic[] = "growtech/data";  

  //Time
  unsigned long lastConnectionTime = 0;               //Time since last connection 
  const unsigned long postingInterval = 20L * 1000L;  //Post data every 20 secs 

  //Sensor setup 
  #define DHTPIN 16
  #define DHTTYPE DHT11
  DHT dht(DHTPIN, DHTTYPE); //DHT Initialize

int motorPin = 5;     //pin für den motor einsetzen 
int levelPin = 33;     //pin für den wasserstandssenor einfügen
int levelPower = 22;  
int soilPin = 34;     //Analog Read pin for Moisture Sensor
int soilPower = 12;
int LED = 21; 
int DHTpower = 13; 

//Speech Recognition + i2s Driver Setup 
int lastRec = 0; 
int16_t sampleBuffer[16000]; 
int16_t features[16000];  
const i2s_port_t I2S_PORT = I2S_NUM_0;
esp_err_t err;

// The I2S config as per the example
  const i2s_config_t i2s_config = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Receive, not transfer
      .sample_rate = 16000,                         // 16KHz
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // could only get it to work with 32bits
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // use right channel
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
      .dma_buf_count = 2,                           // number of buffers
      .dma_buf_len = 8                              // 8 samples per buffer (minimum)
  };

// The pin config as per the setup
  const i2s_pin_config_t pin_config = {
      .bck_io_num = 26,   // Serial Clock (SCK)
      .ws_io_num = 19,    // Word Select (WS)
      .data_out_num = I2S_PIN_NO_CHANGE, // not used (only for speakers)
      .data_in_num = 32   // Serial Data (SD)
  };

int I2SRead(){
  size_t bytesRead;
  
  digitalWrite(LED, HIGH);                     //Indicate start of recording
  Serial.println(" *** Recording Start ***"); 
  int count = 0; 
  lastRec = millis(); 
  while(1){
    i2s_read(I2S_PORT, (void*) sampleBuffer, 4, &bytesRead, portMAX_DELAY); //read from i2s until voice is detected 
    Serial.println(*sampleBuffer);
    if(*sampleBuffer < (-40) || millis() - lastRec >= 6000){
      for(int i = 0;i < 16000; i++){
        i2s_read(I2S_PORT, (void*)sampleBuffer, 4, &bytesRead, portMAX_DELAY); //when voice detected, read samples into features[] array 
        features[i] = sampleBuffer[0]; 
       }
       break; 
    }
  }
  Serial.println(" *** RECORDING ENDED *** "); 
  digitalWrite(LED, LOW); 
  return bytesRead; 
} 


int raw_get_data(size_t offset, size_t length, float *out_ptr) {
    return numpy::int16_to_float(features + offset, out_ptr, length);
}

//JSON 
DynamicJsonDocument doc(1024);

//Callback function controlls the motor 
void callback(char *topic, byte* payload, int len){
  char string[64];      //string to store incoming data
  int dt = 0;               //delay time 

  Serial.println("Message Arrived!"); 

  // for(int i = 0; payload[i] != 0; i++)
  // Serial.println((char)payload[i]); 

  if (strcmp(topic, "growtech/control") == 0) {
          deserializeJson(doc, payload);
          int device = doc["deviceID"];
          int action = doc["actionID"];
          int time = doc["watering"];
          
          if (device == DEVICE_ID) {
              switch (action) {
              case 4:
                  // Do whatever action 1 means
                  digitalWrite(motorPin, HIGH);
                  delay(1000);
                  digitalWrite(motorPin, LOW); 
                  delay(100); 
                  break;
              case 2:
                  // Do whatever action 2 means
                  digitalWrite(5, LOW);
                  break;
              }
          }
      }

}

//Hier werden alle Einstellungen durchgeführt
void setup() {
  Serial.begin(9600);

  WiFi.disconnect(false,true); 

  WiFiSetup();      //WLAN Einstellungen und Verbindung 

  MQTTSetup();     //MQTT Einstellungen.

  PinSetup();      //Elektronik Einstellungen.
}

//Hauptprogramm
void loop() {

  //Use Edge Impulse to classify data 
  edgeImpulse(); 
  
  if(!mqttClient.connected()) reconnect(); //Verbinde ESP32 mit dem MQTT Broker. 
 
  mqttClient.loop(); 
  
  /*
  digitalWrite(LED, HIGH); 
  delay(100); 
  digitalWrite(LED, LOW); 
  delay(1000); 
  */
  
  if(millis() - lastConnectionTime > postingInterval) mqttPublish();
}

void WiFiSetup(){

  int status = WL_IDLE_STATUS; //Set WiFi status temporarily to 'not connected'
  //Attempt to connect to WiFi
  while(status != WL_CONNECTED){
    Serial.println("Connecting..."); 
    status = WiFi.begin(ssid, pass); 
    delay(10000); 
  }
  Serial.println("Connected to WiFi!");

}

void MQTTSetup(){
 
  mqttClient.setServer(server, port);
  delay(1000);  

  mqttClient.setCallback(callback);
  Serial.println("MQTT Setup!");   
}

void PinSetup(){

  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  delay(100); 
  i2s_set_pin(I2S_PORT, &pin_config);
  delay(100); 

    
  pinMode(soilPin, INPUT);    //Bodenfeuchtigkeit
  pinMode(soilPower, OUTPUT); 
  pinMode(LED, OUTPUT);    //Benachrichtigungs LED 
  pinMode(levelPin, INPUT);   //Wasserstand 
  pinMode(motorPin, OUTPUT); 
  pinMode(DHTpower, OUTPUT);
  pinMode(26, OUTPUT); 
  pinMode(14, OUTPUT); 
  pinMode(32, INPUT); 
  digitalWrite(DHTpower, HIGH); 
  delay(100);  
  dht.begin();                //Luftfeuchtigkeit und Temperatur

  Serial.println("Pins Setup!"); 
}

void reconnect(){
  int i; 
  
  while(!mqttClient.connected()){
    Serial.print("Attempting MQTT connection...");

    String clientID = "ESP32Client-";
    clientID += String(random(0xffff), HEX);
      
     if(mqttClient.connect(clientID.c_str(), mqttUserName, mqttPass)) {
      Serial.println("Connected!"); //Connect to broker and check if successful
     }
     else{
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state()); //Print error Code. Check PubSub Doc for more info 
      Serial.println("Try again in 5 sec"); 
      delay(5000);  
     }
  }
    
  int substat = 0; 
  while(!substat){
    substat = mqttClient.subscribe("growtech/#"); 
    Serial.println("Subscribing..."); 
    delay(6000); 
  }
  Serial.println("Subscribe Successful!");

}

void mqttPublish() {

  float t = dht.readTemperature(); // Read temperature from DHT sensor.
  float h = dht.readHumidity();  // Read humidity from DHT sensor.
  float moisture = collectMoisture(); //Read soil moisture content
  int waterLevel = water_height(); 


  if(isnan(t) || isnan(h)){
    digitalWrite(LED, HIGH); 
    delay(4000); 
    digitalWrite(LED, LOW); 
    delay(100); 
  }  
  
  char payload[1024]; 
  sprintf(payload, "{ \"temperature\": %.2f, \"moisture\": %.2f, \"humidity\": %.2f, \"waterLevel\": %d, \"device\": %d  }", t, moisture, h, waterLevel, DEVICE_ID); 

  
  mqttClient.publish(pubTopic, payload);
  
  Serial.print("Soil Moisture: "); 
  Serial.print(moisture); 
  Serial.print(" Temperature: "); 
  Serial.print(t); 
  Serial.print(" Humidity: "); 
  Serial.println(h); 

  lastConnectionTime = millis();
}

float collectMoisture(){
  digitalWrite(soilPower, HIGH); 
  delay(1000);
  int moisture = 0; 

  for(int i = 0; i<100; i++) moisture += analogRead(soilPin);
  moisture = moisture/100; 

  float val = (((-50./1159.)*moisture) + 177.); 


  if(val > 100.) val = 100.; 
  if(val < 0.) val = 0.;

  delay(100); 
  digitalWrite(soilPower, LOW); 
  delay(10); 
  
  return val; 
}

int water_height(){
  int val = 0; 
  digitalWrite(levelPower, HIGH);
  delay(10);            
  val = analogRead(levelPin);   
  delay(100);
  digitalWrite(levelPower, LOW);  

  Serial.print("Level: "); 
  Serial.println(val); 

  return val;             
}

void ei_printf(const char *format, ...) {
    static char print_buf[1024] = { 0 };

    va_list args;
    va_start(args, format);
    int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
    va_end(args);

    if (r > 0) {
        Serial.write(print_buf);
    }
}

void edgeImpulse()
{
    ei_printf("Edge Impulse standalone inferencing (Arduino)\n");
    //Record Audio
    int bytesRead = I2SRead();
    //for(int i = 0; i < 50; i++)     //for debugging 
      //Serial.print(features[i]); 

/*
    if (sizeof(features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        ei_printf("The size of your 'features' array is not correct. Expected %lu items, but had %lu\n",
            EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(features) / sizeof(float));
        delay(1000);
        return;
    }
*/

    ei_impulse_result_t result = { 0 };

    // the features are stored into flash, and we don't want to load everything into RAM
    signal_t signal;
    signal.total_length = 16000; 
    signal.get_data = &raw_get_data; 


    // invoke the impulse
    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false /* debug */);
    ei_printf("run_classifier returned: %d\n", res);

    if (res != 0) return;

    // print the predictions
    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");
    ei_printf("[");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("%.5f", result.classification[ix].value);
#if EI_CLASSIFIER_HAS_ANOMALY == 1
        ei_printf(", ");
#else
        if (ix != EI_CLASSIFIER_LABEL_COUNT - 1) {
            ei_printf(", ");
        }
#endif
    }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("%.3f", result.anomaly);
#endif
    ei_printf("]\n");

    // human-readable predictions
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
    }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif

  Serial.print("grow-tech: "); 
  Serial.println(result.classification[2].value); 

  if(result.classification[2].value > 0.8){
    digitalWrite(motorPin, HIGH); 
    delay(4000); 
    digitalWrite(motorPin, LOW); 
    delay(4000); 
  }
}
