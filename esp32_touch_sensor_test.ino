//Touch sensor test for ESP32

// Include the touch sensor library for ESP32
#include <driver/touch_pad.h>

void setup() {

 
    // Initialize the serial port for debugging
    Serial.begin(115200);
    yield();
    delay(500); //Wait for serial port
    
    // Initialize the touch sensor
    touch_pad_init();
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    
    // Configure touch pad channels
    touch_pad_config(TOUCH_PAD_NUM0, 0);
    touch_pad_config(TOUCH_PAD_NUM1, 0);
    touch_pad_config(TOUCH_PAD_NUM2, 0);
    touch_pad_config(TOUCH_PAD_NUM3, 0);
    touch_pad_config(TOUCH_PAD_NUM4, 0);
    touch_pad_config(TOUCH_PAD_NUM5, 0);
    touch_pad_config(TOUCH_PAD_NUM6, 0);
    touch_pad_config(TOUCH_PAD_NUM7, 0);
    touch_pad_config(TOUCH_PAD_NUM8, 0);
    touch_pad_config(TOUCH_PAD_NUM9, 0);

    // Initialize touch pad peripheral.
    // The default fsm mode is software trigger mode.
    //ESP_ERROR_CHECK(touch_pad_init());
    
    // Set reference voltage for charging/discharging
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);

  #if TOUCH_FILTER_MODE_EN
      touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);
  #endif

    Serial.println("Touch pads configured.");
    //Serial.flush();
    yield();
    }//setup()

void loop() {
    // Initialize variables to store touch sensor readings
    uint16_t touch[10];

    // Read touch sensor values
    touch_pad_read(TOUCH_PAD_NUM0, &touch[0]); // OK
    touch_pad_read(TOUCH_PAD_NUM1, &touch[1]); // INOP (0)
    touch_pad_read(TOUCH_PAD_NUM2, &touch[2]); // OK
    touch_pad_read(TOUCH_PAD_NUM3, &touch[3]); // OK
    touch_pad_read(TOUCH_PAD_NUM4, &touch[4]); // OK
    touch_pad_read(TOUCH_PAD_NUM5, &touch[5]); // OK
    touch_pad_read(TOUCH_PAD_NUM6, &touch[6]); // OK
    touch_pad_read(TOUCH_PAD_NUM7, &touch[7]); // OK
    touch_pad_read(TOUCH_PAD_NUM8, &touch[8]); // OK
    touch_pad_read(TOUCH_PAD_NUM9, &touch[9]); // OK


    // Print touch sensor values to the serial port
    for(int n=0;n<10;n++){
      Serial.printf("%3d ",touch[n]);
      }//for n
    Serial.println();
    
    
    // Delay to limit the output to 10 lines per second
    delay(100);
    
    yield();
    }//loop()

    
