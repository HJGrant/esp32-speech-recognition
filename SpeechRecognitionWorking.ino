#include <speech-recognition-grow-tech_inferencing.h>
#include "driver/i2s.h"

//ESP and I2S Driver Setup
const i2s_port_t I2S_PORT = I2S_NUM_0;
esp_err_t err;
#define I2S_SAMPLE_RATE (16000)
int LED = 13; 
int rLED = 12; 
int lastRec = 0; 


// The I2S config as per the example
  const i2s_config_t i2s_config = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Receive, not transfer
      .sample_rate = I2S_SAMPLE_RATE,                         // 16KHz
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // could only get it to work with 32bits
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // use right channel
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
      .dma_buf_count = 4,                           // number of buffers
      .dma_buf_len = 8                              // 8 samples per buffer (minimum)
  };

// The pin config as per the setup
  const i2s_pin_config_t pin_config = {
      .bck_io_num = 26,   // Serial Clock (SCK)
      .ws_io_num = 14,    // Word Select (WS)
      .data_out_num = I2S_PIN_NO_CHANGE, // not used (only for speakers)
      .data_in_num = 32   // Serial Data (SD)
  };

int16_t sampleBuffer[16000]; 
int16_t features[16000];  

int I2SRead(){
  size_t bytesRead;
 
  digitalWrite(rLED, HIGH); 
  Serial.println(" *** Recording Start ***"); 
  int count = 0; 
  lastRec = millis(); 
  while(1){
    i2s_read(I2S_PORT, (void*) sampleBuffer, 4, &bytesRead, portMAX_DELAY);
    if(*sampleBuffer < (-40) || millis() - lastRec >= 6000){
      for(int i = 0;i < 16000; i++){
        i2s_read(I2S_PORT, (void*) sampleBuffer, 4, &bytesRead, portMAX_DELAY);
        features[i] = sampleBuffer[0]; 
       }
       break; 
    }
  }
  Serial.println(" *** RECORDING ENDED *** "); 
  digitalWrite(rLED, LOW); 
  return bytesRead; 
} 

int raw_get_data(size_t offset, size_t length, float *out_ptr) {
    return numpy::int16_to_float(features + offset, out_ptr, length);
}

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);

    Serial.println("Edge Impulse Inferencing Demo");

    err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_config);

    pinMode(33, OUTPUT); 
    pinMode(27, INPUT); 
    pinMode(25, OUTPUT); 
    pinMode(LED, OUTPUT); 
    pinMode(rLED, OUTPUT); 

}


void loop()
{
    ei_printf("Edge Impulse standalone inferencing (Arduino)\n");

    //Record Audio
    int bytesRead = I2SRead();
    //for(int i = 0; i < 50; i++)  
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
    digitalWrite(LED, HIGH); 
    delay(1000); 
    digitalWrite(LED, LOW); 
    delay(1000); 
  }


}

/**
 * @brief      Printf function uses vsnprintf and output using Arduino Serial
 *
 * @param[in]  format     Variable argument list
 */
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
