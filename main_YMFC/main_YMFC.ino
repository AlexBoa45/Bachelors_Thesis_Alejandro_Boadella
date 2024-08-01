/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//This code has been made and compiled by Alejandro Boadella (EETAC-UPC student). 
// Main source: 
// - Brokking.net Link: http://www.brokking.net/ymfc-32_auto_main.html#8.2
// Secondary sources:
// - Design and implementation of a new quadcopter drone prototype
// controlled through STM32 using the Arduino environment by León Enrique Prieto Bailo.
// - Arduproject.es
// It works only on stabilize mode. GPS mode doesn't work properly, little changes have to be made.
// This is due by the time factor on the development of this project. Some variables are not needed.
// I apologize for the inconvenience of any not mentioned source or any error committed 
// in the readaction of the code.
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

//Libraries used:
#include <Wire.h>                     // I2C bus library.
#include <SoftwareSerial.h>           // Serial bus for GPS "digital serial".

// Flight Mode Enumeration (i.e: FM_disabled==flight mode 0)
enum FlightMode{
  FM_disabled,                            // Disabled mode.
  FM_mounting,                            // Starts to be prepared for take off mode.
  FM_stable,                              // No balance fly mode.
  FM_stable_battery,                      // Stable mode + Battery corrector.
  FM_gps_hold                             // Mode that uses GPS to be stable.
};

 FlightMode fm = FM_disabled;             //Starting flight mode (fm)

// Serial connection to pins RX=PC2/Pin 12 and TX=PC3/Pin 11 for the GNSS receviver
SoftwareSerial ss(PC2, PC3);

// LED
volatile long led_timer;

// MOTORS
long time_motores_start, time_1, time_2, time_ON;

// FlightSky i6
#define pin_PPM PA4                                 // Pin radio PPM.
#define number_channels 8                           // 6 channels and 2 phantom channels. 
                                                    //if you increase the number of these channels, the phantom channels won't be there.
uint64_t pulse_instant[number_channels * 2 + 2];    // 8 channels that contains 8 rises and 8 falls in the PPM = 16 + 2 for the markers.
uint16_t remote_channel[number_channels];           // Vector of values of channels.
uint8_t flank_count = 1;                            // Counter. 
int16_t throttle;                                   
int32_t pid_roll_setpoint_base;
int32_t pid_pitch_setpoint_base;

// PID: Variables
float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

// Check:
float pid_t_control_error, pid_t_output, pid_rate_control_error, pid_i_mem_rate, pid_rate_error_prev, pid_output_rate;

// PID: Roll
float pid_p_gain_roll = 0.9;      // (By Default Brokking uses) p=1.3 i=0.04 d=18
float pid_i_gain_roll = 0.009;
float pid_d_gain_roll = 4;
int pid_max_roll = 400;

// PID: Pitch
float pid_p_gain_pitch = pid_p_gain_roll;
float pid_i_gain_pitch = pid_i_gain_roll;
float pid_d_gain_pitch = pid_d_gain_roll;
int pid_max_pitch = 400;

// PID: Yaw
float pid_p_gain_yaw = 0.75;
float pid_i_gain_yaw = 0.01;
float pid_d_gain_yaw = 0;
int pid_max_yaw = 400;

// MPU6050:        
#define MPU6050_ADDRESS 0x68                  
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_CONFIG 0x1A
int16_t manual_gyro_pitch_cal_value = 0;
int16_t manual_gyro_roll_cal_value = 0;
int16_t manual_gyro_yaw_cal_value = 0;
int16_t manual_x_cal_value = 0;
int16_t manual_y_cal_value = 0;
int16_t manual_z_cal_value = 0;
boolean auto_level = true;                      // This makes that it calibrates automatically the IMU every time the code starts.
uint8_t use_manual_calibration = false;         // No special use. Change if you want manual calibration.
int32_t acc_total_vector, acc_total_vector_at_start;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
int16_t acc_pitch_cal_value;
int16_t acc_roll_cal_value;
int16_t cal_int;
int16_t temperature;
int16_t acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;
int32_t acc_x_cal, acc_y_cal, acc_z_cal;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;

// ESC
#define pin_motor1 PC6                        // Pin motor 1  GPIO 6
#define pin_motor2 PC7                        // Pin motor 2  GPIO 5
#define pin_motor3 PB9                        // Pin motor 3  GPIO 10
#define pin_motor4 PB8                        // Pin motor 4  GPIO 9
int16_t esc_1, esc_2, esc_3, esc_4;           // Values for outputs.

TIM_TypeDef *TIM_DEF_M1_M2 = TIM3;            // This code defines 2 hardware timers that will take the role of send PWM pulses to the motors. 
TIM_TypeDef *TIM_DEF_M3_M4 = TIM4;

uint32_t channel_motor1 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_motor1), PinMap_PWM));  // Defines a PWM output.
uint32_t channel_motor2 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_motor2), PinMap_PWM));
uint32_t channel_motor3 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_motor3), PinMap_PWM));
uint32_t channel_motor4 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_motor4), PinMap_PWM));

HardwareTimer *TIM_M1_M2 = new HardwareTimer(TIM_DEF_M1_M2); // creation of the timers.
HardwareTimer *TIM_M3_M4 = new HardwareTimer(TIM_DEF_M3_M4);

// Loop timer
uint32_t loop_timer;

// Error signal
uint8_t error;

// GPS 
boolean gps_status = false;                // It start in false, then if the user wants to use GPS a previous calibration shall be done to use it.
float gps_p_gain = 2.7;                    //Gain setting for the GPS P-controller (default = 2.7).
float gps_d_gain = 6.5;                    //Gain setting for the GPS D-controller (default = 6.5).
float declination = 0.0;                   //Set the declination between the magnetic and geographic north. (Barcelona has 2 degress approx in 2024).

uint8_t read_serial_byte, incomming_message[100], number_used_sats, fix_type;
uint8_t waypoint_set = 0, latitude_north, longiude_east ;
uint16_t message_counter;
int16_t gps_add_counter;
int32_t l_lat_gps, l_lon_gps, lat_gps_previous, lon_gps_previous;
int32_t lat_gps_actual, lon_gps_actual, l_lat_waypoint, l_lon_waypoint;
float gps_pitch_adjust_north, gps_pitch_adjust, gps_roll_adjust_north, gps_roll_adjust;
float lat_gps_loop_add, lon_gps_loop_add, lat_gps_add, lon_gps_add;
uint8_t new_line_found, new_gps_data_available, new_gps_data_counter;
uint8_t gps_rotating_mem_location, return_to_home_step;
int32_t gps_lat_total_avarage, gps_lon_total_avarage;
int32_t gps_lat_rotating_mem[40], gps_lon_rotating_mem[40];
int32_t gps_lat_error, gps_lon_error;
int32_t gps_lat_error_previous, gps_lon_error_previous;
uint32_t gps_watchdog_timer;

float l_lon_gps_float_adjust, l_lat_gps_float_adjust, gps_man_adjust_heading;
float return_to_home_lat_factor, return_to_home_lon_factor, return_to_home_move_factor;
uint8_t home_point_recorded;
int32_t lat_gps_home, lon_gps_home;

// Compass HMC5883L: 
boolean hold_heading_status = false;      // It makes a first measure of the compass to set the heading and enables GPS.
uint8_t compass_address = 0x1E;            
uint8_t compass_calibration_on, heading_lock;
int16_t compass_x, compass_y, compass_z;
int16_t compass_cal_values[6];
float compass_x_horizontal, compass_y_horizontal, actual_compass_heading;
float compass_scale_y, compass_scale_z;
int16_t compass_offset_x, compass_offset_y, compass_offset_z;
float course_a, course_b, course_c, base_course_mirrored, actual_course_mirrored;
float course_lock_heading, heading_lock_course_deviation;


// Battery: (It needs to be a 3S battery or the code should be modified):
float low_battery_warning = 10.5;          //Set the battery warning at 10.5V (default = 10.5V).
float battery_voltage;
float battery_compensation = 40.0;       // Increment if the drone falls when the battery voltage falls.
                                         // Decrement if the drone rises when the battery voltage falls.
                                         
// Timer to print values via serial:
int long timp;            

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////       Setup routine       //////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);                     // Start serial communication at 115200 bauds.
  Wire.begin();                             // Start I2C bus at 400khz (high velocity).
  Wire.setClock(400000); 
  delay(1000);
  
  init_components();                        // Start basic components.
  read_rc();                                // Read radio.
  led_on();                                 // Show that the drone is activated.
  delay(250);

  if (remote_channel[5] < 1100){            // While and if the drone channel 5 is lower than 1100us, activate ESC calibration. 
    read_rc();
    while (remote_channel[5] < 1100){
        read_rc();
                                                                                          
         esc_1 = remote_channel[3];         // Read radio and see throttle channel.
         esc_2 = remote_channel[3];
         esc_3 = remote_channel[3];
         esc_4 = remote_channel[3];
      
          if (esc_1 < 1000) esc_1 = 1000;   // Enable lower and upper pulse limits.
          if (esc_2 < 1000) esc_2 = 1000;  
          if (esc_3 < 1000) esc_3 = 1000; 
          if (esc_4 < 1000) esc_4 = 1000; 
        
          if (esc_1 > 2000) esc_1 = 2000; 
          if (esc_2 > 2000) esc_2 = 2000;  
          if (esc_3 > 2000) esc_3 = 2000;  
          if (esc_4 > 2000) esc_4 = 2000; 
          
        TIM_M1_M2->setCaptureCompare(channel_motor1, esc_1, MICROSEC_COMPARE_FORMAT);   // Send output data to the motors.
        TIM_M1_M2->setCaptureCompare(channel_motor2, esc_2, MICROSEC_COMPARE_FORMAT);
        TIM_M3_M4->setCaptureCompare(channel_motor3, esc_3, MICROSEC_COMPARE_FORMAT);
        TIM_M3_M4->setCaptureCompare(channel_motor4, esc_4, MICROSEC_COMPARE_FORMAT);
    }
  }

  if (remote_channel[5] > 1900){    // if the channel 5 is higher than 1900us calibrate compass.
   led_off();                       // Show motors are not enable.
   delay(200);
   setup_compass();
   calibrate_compass();             // Initialize the compass and set the correct registers.
   read_compass();                  // Read and calculate the compass data.
   gps_status=true;                 // Enable GPS 
   delay(200);
   led_on();                        // Enable drone 
   delay(200);
  }
  
  led_off();
  delay(200);
  init_imu();                       // Calibrate IMU or MPU-6050.
  delay(200);
  led_on();
  loop_timer = micros();
  Serial.println("Setup finished");
  timp=millis();
  read_process_units();             // Make basic measurement and calculus.
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////       Main routine    ///////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  
  reference_computation();                                              // Seeking for fligh mode 
 
       if (fm == FM_gps_hold && hold_heading_status == false){          // If the GPS mode is set, enable it and get the heading angle. 
          gps_setup();                                                  // Set the baud rate and output refresh rate of the GPS module.     ¡¡¡ If the code stops probably read flush serial buffer !!!
          read_compass();                                               // Read and calculate the compass data.
          angle_yaw = actual_compass_heading;                           // Set the initial compass heading.
          course_lock_heading = angle_yaw;                              // Set the heading as the main angle in reference computation.
          hold_heading_status=true;
       }                                                                
       
   read_process_units();                                                // Process main units.
 
      if (fm == FM_gps_hold && hold_heading_status == true){            // This code enables GPS positionaing and also at the same time heading correction
        
          read_compass();                                                                  // Read and calculate the compass data.
          if (gps_add_counter >= 0)gps_add_counter --;
        
          read_gps();
          
          angle_yaw -= course_deviation(angle_yaw, actual_compass_heading) / 1200.0;       // Calculate the difference between the gyro and compass heading and make a small correction.
          if (angle_yaw < 0) angle_yaw += 360;                                             // If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
          else if (angle_yaw >= 360) angle_yaw -= 360;                                     // If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.
          
          gps_man_adjust_heading = angle_yaw;                                              // Send GPS heading correction.
          
          //When the heading_lock mode is activated the roll and pitch pid setpoints are heading dependent.
          //At startup the heading is registerd in the variable course_lock_heading.
          //First the course deviation is calculated between the current heading and the course_lock_heading.
          //Based on this deviation the pitch and roll controls are calculated so the responce is the same as on startup.
          
          if (hold_heading_status == true) {                                               // This code will only be activated if GPS at that instance isn't enable due loss of GPS information. Put false to desactivate!!!!!!!!!
                                                                                           // Put false if you dont want to use it. It doesn't correct the heading, only corrects the movement of the drone respect the pilot orientation.
            heading_lock_course_deviation = course_deviation(angle_yaw, course_lock_heading);
            remote_channel[1] = 1500 + ((float)(remote_channel[1] - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(remote_channel[2] - 1500) * cos((heading_lock_course_deviation - 90) * 0.017453));
            remote_channel[2] = 1500 + ((float)(remote_channel[2] - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(remote_channel[1] - 1500) * cos((heading_lock_course_deviation + 90) * 0.017453));
            gps_man_adjust_heading = course_lock_heading;
          }
      }

      
      if (fm == FM_gps_hold && waypoint_set == 1) {     // If the drone is in GPS mode and there's a waypoint.
        pid_roll_setpoint_base = 1500 + gps_roll_adjust;
        pid_pitch_setpoint_base = 1500 + gps_pitch_adjust;
      }
      else {
        pid_roll_setpoint_base = remote_channel[1];
        pid_pitch_setpoint_base = remote_channel[2];  
      }
    
      //Because we added the GPS adjust values we need to make sure that the control limits are not exceded.
      if (pid_roll_setpoint_base > 2000)pid_roll_setpoint_base = 2000;
      if (pid_roll_setpoint_base < 1000)pid_roll_setpoint_base = 1000;
      if (pid_pitch_setpoint_base > 2000)pid_pitch_setpoint_base = 2000;
      if (pid_pitch_setpoint_base < 1000)pid_pitch_setpoint_base = 1000;
      
  controllers();  // Apply PID corrections.

  actuators();    // Send signals to the motors.

  if (micros() - loop_timer > 4050) {     // It the loop is too slow, show it. 
    Serial.println("LOOP SLOW");
  }
  
  while (micros() - loop_timer < 4000);   //We wait until 4000us are passed.
  loop_timer = micros();

  /*if (millis() - timp > 1000) {           // Every second send information of anything the user want like, compass angle, GPS position, etc.

      Serial.print("FM:");
      Serial.println(fm);

      timp=millis();
  }*/

}
