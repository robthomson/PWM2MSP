
#include <MSP.h>
#include "flt_modes.h"


HardwareSerial &mspSerial = Serial3;    //pin 8 tx pin 7 rx on teensy 3.2
byte PWM_PIN = 9;
int pwm_value;

MSP msp;

int mspArm;

uint32_t previousMillis_MSP = 0;
const uint32_t next_interval_MSP = 100;
uint32_t custom_mode = 0;             
uint32_t general_counter = 0;
uint32_t previousFlightMode = custom_mode;
uint32_t flightModeFlags = 0;



void setup()
{
    mspSerial.begin(115200);
    msp.begin(mspSerial);
    pinMode(PWM_PIN, INPUT);
}

void loop()
{

    pwm_value = pulseIn(PWM_PIN, HIGH);
    
    //send MSP data
    uint32_t currentMillis_MSP = millis();
    if ((uint32_t)(currentMillis_MSP - previousMillis_MSP) >= next_interval_MSP) {
        previousMillis_MSP = currentMillis_MSP;
        set_flight_mode_flags();
        send_msp_to_airunit();   
        general_counter += next_interval_MSP;
    }    
    
}


// MSP_STATUS reply customized for BF/DJI
struct msp_status_BF_t {
  uint16_t cycleTime;
  uint16_t i2cErrorCounter;
  uint16_t sensor;                    // MSP_STATUS_SENSOR_...
  uint32_t flightModeFlags;           // see getActiveModes()
  uint8_t  configProfileIndex;
  uint16_t averageSystemLoadPercent;  // 0...100
  uint16_t gyroCyleTime;
  uint8_t bytecount;    //0
  //uint8_t flagsData;  //nothing because bytecount == 0
  uint8_t armingDisableFlagsCount;
  uint32_t armingDisableFlags;
  uint8_t  rebootRequired;
} __attribute__ ((packed));



void set_flight_mode_flags()
{
    if((pwm_value >= 1950) && pwm_value <= 2050){
        flightModeFlags |= ARM_ACRO_BF;
        Serial.println("ARMED");
    }
    else{
        flightModeFlags &= ~ARM_ACRO_BF;
        Serial.println("DISARMED");
    }
}

msp_status_BF_t status_BF = {0};

void send_msp_to_airunit()
{
    //MSP_STATUS
    status_BF.flightModeFlags = flightModeFlags;
    msp.send(MSP_STATUS, &status_BF, sizeof(status_BF));

}
