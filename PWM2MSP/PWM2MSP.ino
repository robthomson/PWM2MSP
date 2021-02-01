
#include <MSP.h>
#include "flt_modes.h"
#include "MSP_OSD.h"
#include "OSD_positions_config.h"

HardwareSerial &mspSerial = Serial3;    //pin 8 tx pin 7 rx on teensy 3.2
byte PWM_PIN = 9;
byte PWMVOLTAGE_PIN = 10; //read the pwm pin to deliver a 'pwm/telemetry voltage scale to the dji unit'
int pwm_value;
double voltage_value;
int max_voltage = 5;  //5 volts
uint8_t vbat = 0;

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
    pinMode(PWMVOLTAGE_PIN, INPUT);
}

void loop()
{

    pwm_value = pulseIn(PWM_PIN, HIGH);
    voltage_value = pulseIn(PWMVOLTAGE_PIN , HIGH);

  Serial.println(voltage_value );
    
    //send MSP data
    uint32_t currentMillis_MSP = millis();
    if ((uint32_t)(currentMillis_MSP - previousMillis_MSP) >= next_interval_MSP) {
        previousMillis_MSP = currentMillis_MSP;
        set_flight_mode_flags();
        send_msp_to_airunit();   
        general_counter += next_interval_MSP;
    }    
    
}

/*
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

*/

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
msp_analog_t analog = {0};
msp_battery_state_t battery_state = {0};

void send_msp_to_airunit()
{
    //MSP_STATUS
    status_BF.flightModeFlags = flightModeFlags;
    msp.send(MSP_STATUS, &status_BF, sizeof(status_BF));

    vbat=map(voltage_value,1000,2000,0,max_voltage) * 10;

    //MSP_ANALOG
    analog.vbat = vbat;

    msp.send(MSP_ANALOG, &analog, sizeof(analog));

    
    send_osd_config();

}


msp_osd_config_t msp_osd_config = {0};

void send_osd_config()
{
  
#ifdef IMPERIAL_UNITS
    msp_osd_config.units = 0;
#else
    msp_osd_config.units = 1;
#endif

    msp_osd_config.osd_item_count = 1;
    msp_osd_config.osd_stat_count = 24;
    msp_osd_config.osd_timer_count = 2;
    msp_osd_config.osd_warning_count = 16;              // 16
    msp_osd_config.osd_profile_count = 1;              // 1
    msp_osd_config.osdprofileindex = 1;                // 1
    msp_osd_config.overlay_radio_mode = 0;             //  0

    
    msp_osd_config.osd_main_batt_voltage_pos = osd_main_batt_voltage_pos;
   
    /*
 msp_osd_config.osd_rssi_value_pos = osd_rssi_value_pos; 
    msp_osd_config.osd_crosshairs_pos = osd_crosshairs_pos;
    msp_osd_config.osd_artificial_horizon_pos = osd_artificial_horizon_pos;
    msp_osd_config.osd_horizon_sidebars_pos = osd_horizon_sidebars_pos;
    msp_osd_config.osd_item_timer_1_pos = osd_item_timer_1_pos;
    msp_osd_config.osd_item_timer_2_pos = osd_item_timer_2_pos;
    msp_osd_config.osd_flymode_pos = osd_flymode_pos;
    msp_osd_config.osd_craft_name_pos = osd_craft_name_pos;
    msp_osd_config.osd_throttle_pos_pos = osd_throttle_pos_pos;
    msp_osd_config.osd_vtx_channel_pos = osd_vtx_channel_pos;
    msp_osd_config.osd_current_draw_pos = osd_current_draw_pos;
    msp_osd_config.osd_mah_drawn_pos = osd_mah_drawn_pos;
    msp_osd_config.osd_gps_speed_pos = osd_gps_speed_pos;
    msp_osd_config.osd_gps_sats_pos = osd_gps_sats_pos;
    msp_osd_config.osd_altitude_pos = osd_altitude_pos;
    msp_osd_config.osd_roll_pids_pos = osd_roll_pids_pos;
    msp_osd_config.osd_pitch_pids_pos = osd_pitch_pids_pos;
    msp_osd_config.osd_yaw_pids_pos = osd_yaw_pids_pos;
    msp_osd_config.osd_power_pos = osd_power_pos;
    msp_osd_config.osd_pidrate_profile_pos = osd_pidrate_profile_pos;
    msp_osd_config.osd_warnings_pos = osd_warnings_pos;
    msp_osd_config.osd_avg_cell_voltage_pos = osd_avg_cell_voltage_pos;
    msp_osd_config.osd_gps_lon_pos = osd_gps_lon_pos;
    msp_osd_config.osd_gps_lat_pos = osd_gps_lat_pos;
    msp_osd_config.osd_debug_pos = osd_debug_pos;
    msp_osd_config.osd_pitch_angle_pos = osd_pitch_angle_pos;
    msp_osd_config.osd_roll_angle_pos = osd_roll_angle_pos;
    msp_osd_config.osd_main_batt_usage_pos = osd_main_batt_usage_pos;
    msp_osd_config.osd_disarmed_pos = osd_disarmed_pos;
    msp_osd_config.osd_home_dir_pos = osd_home_dir_pos;
    msp_osd_config.osd_home_dist_pos = osd_home_dist_pos;
    msp_osd_config.osd_numerical_heading_pos = osd_numerical_heading_pos;
    msp_osd_config.osd_numerical_vario_pos = osd_numerical_vario_pos;
    msp_osd_config.osd_compass_bar_pos = osd_compass_bar_pos;
    msp_osd_config.osd_esc_tmp_pos = osd_esc_tmp_pos;
    msp_osd_config.osd_esc_rpm_pos = osd_esc_rpm_pos;
    msp_osd_config.osd_remaining_time_estimate_pos = osd_remaining_time_estimate_pos;
    msp_osd_config.osd_rtc_datetime_pos = osd_rtc_datetime_pos;
    msp_osd_config.osd_adjustment_range_pos = osd_adjustment_range_pos;
    msp_osd_config.osd_core_temperature_pos = osd_core_temperature_pos;
    msp_osd_config.osd_anti_gravity_pos = osd_anti_gravity_pos;
    msp_osd_config.osd_g_force_pos = osd_g_force_pos;
    msp_osd_config.osd_motor_diag_pos = osd_motor_diag_pos;
    msp_osd_config.osd_log_status_pos = osd_log_status_pos;
    msp_osd_config.osd_flip_arrow_pos = osd_flip_arrow_pos;
    msp_osd_config.osd_link_quality_pos = osd_link_quality_pos;
    msp_osd_config.osd_flight_dist_pos = osd_flight_dist_pos;
    msp_osd_config.osd_stick_overlay_left_pos = osd_stick_overlay_left_pos;
    msp_osd_config.osd_stick_overlay_right_pos = osd_stick_overlay_right_pos;
    msp_osd_config.osd_display_name_pos = osd_display_name_pos;
    msp_osd_config.osd_esc_rpm_freq_pos = osd_esc_rpm_freq_pos;
    msp_osd_config.osd_rate_profile_name_pos = osd_rate_profile_name_pos;
    msp_osd_config.osd_pid_profile_name_pos = osd_pid_profile_name_pos;
    msp_osd_config.osd_profile_name_pos = osd_profile_name_pos;
    msp_osd_config.osd_rssi_dbm_value_pos = osd_rssi_dbm_value_pos;
    msp_osd_config.osd_rc_channels_pos = osd_rc_channels_pos;
  */

    msp.send(MSP_OSD_CONFIG, &msp_osd_config, sizeof(msp_osd_config));
}
