
void init_components() {
  init_led();
  init_rc();
  init_esc();
}

// Led output declaration:
void init_led() {
  pinMode(PC1, OUTPUT);
}

// Radio control PPM read setup:
void init_rc() {
  pinMode(pin_PPM, INPUT);                                                    // Declare the pin as an input.
  attachInterrupt(digitalPinToInterrupt(pin_PPM), read_PPM, CHANGE);          // Declare that every time the signal changes read_PPM will be applied as an interruption.
}

// Reads the PPM signal: (IT DOESN'T PROCESSES)
void read_PPM() {
  if (micros() - pulse_instant[flank_count - 1] > 2500) flank_count = 0;      // If the changer of the signal timer is higher than 2500us the PPM whole signal restarts leaving a counter will 0 as a value.
  pulse_instant[flank_count] = micros();                                      // Safe in a vector the time when the signal rises or falls.
  flank_count++;                                                              // Counter increment when the signal rises or falls.
}

// Declaration motor PWM with the Timers:
void init_esc() { 
  TIM_M1_M2->setPWM(channel_motor1, pin_motor1, 250, 0);                      // Probably not needed, it sets PWM at 250 HZ with 0% duty cycle.
  TIM_M1_M2->setPWM(channel_motor2, pin_motor2, 250, 0);
  TIM_M3_M4->setPWM(channel_motor3, pin_motor3, 250, 0);
  TIM_M3_M4->setPWM(channel_motor4, pin_motor4, 250, 0);  
}

// GPS setup:
void gps_setup(void){

  ss.begin(57600);                                                           // GPS setup at 57600 bauds, it could be more for more efficient code.
  delay(250);
 
}

// Compass calibration:
void calibrate_compass(void) {
  compass_calibration_on = 1;                                                // Set the compass_calibration_on variable to disable the adjustment of the raw compass values.
  while (remote_channel[2] < 1900) {                                         // Stay in this loop until the pilot pulls up the pitch stick of the transmitter.
    delayMicroseconds(3700);                                                 // Simulate a 250Hz program loop.
    read_compass();                                                          // Read the raw compass values.
    read_rc();
    // In the following lines the maximum and minimum compass values are detected and stored.
    if (compass_x < compass_cal_values[0])compass_cal_values[0] = compass_x;
    if (compass_x > compass_cal_values[1])compass_cal_values[1] = compass_x;
    if (compass_y < compass_cal_values[2])compass_cal_values[2] = compass_y;
    if (compass_y > compass_cal_values[3])compass_cal_values[3] = compass_y;
    if (compass_z < compass_cal_values[4])compass_cal_values[4] = compass_z;
    if (compass_z > compass_cal_values[5])compass_cal_values[5] = compass_z;
  }
  compass_calibration_on = 0;                                                // Reset the compass_calibration_on variable.

  setup_compass();                                                           // Initiallize the compass and set the correct registers.
  read_compass();                                                            // Read and calculate the compass data.
  angle_yaw = actual_compass_heading;                                        // Set the initial compass heading.

  loop_timer = micros();                                                     //Set the timer for the next loop.
}

//At startup the registers of the compass need to be set. After that the calibration offset and scale values are calculated.
void setup_compass() {
  Wire.beginTransmission(compass_address);                     // Start communication with the compass.
  Wire.write(0x00);                                            // We want to write to the Configuration Register A (00 hex).
  Wire.write(0x78);                                            // Set the Configuration Regiser A bits as 01111000 to set sample rate (average of 8 at 75Hz).
  Wire.write(0x20);                                            // Set the Configuration Regiser B bits as 00100000 to set the gain at +/-1.3Ga.
  Wire.write(0x00);                                            // Set the Mode Regiser bits as 00000000 to set Continues-Measurement Mode.
  Wire.endTransmission();                                      // End the transmission with the compass.

  // Calculate the calibration offset and scale values
  compass_scale_y = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[3] - compass_cal_values[2]);
  compass_scale_z = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[5] - compass_cal_values[4]);

  compass_offset_x = (compass_cal_values[1] - compass_cal_values[0]) / 2 - compass_cal_values[1];
  compass_offset_y = (((float)compass_cal_values[3] - compass_cal_values[2]) / 2 - compass_cal_values[3]) * compass_scale_y;
  compass_offset_z = (((float)compass_cal_values[5] - compass_cal_values[4]) / 2 - compass_cal_values[5]) * compass_scale_z;
}

// Start the IMU/MPU-6050:
void init_imu(void) {                  
  Wire.beginTransmission(MPU6050_ADDRESS);              
  error = Wire.endTransmission();      
  while (error != 0) {                
    delay(4);
  }

  Wire.beginTransmission(MPU6050_ADDRESS);              // Start communication with the MPU-6050.
  Wire.write(MPU6050_PWR_MGMT_1);                       // We want to write to the PWR_MGMT_1 register (6B hex).
  Wire.write(0x00);                                     // Set the register bits as 00000000 to activate the gyro.
  Wire.endTransmission();                               // End the transmission with the gyro.

  Wire.beginTransmission(MPU6050_ADDRESS); 
  Wire.write(MPU6050_GYRO_CONFIG);                      // We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x08);                                     // Set the register bits as 00001000 (500dps full scale).
  Wire.endTransmission();            

  Wire.beginTransmission(MPU6050_ADDRESS);  
  Wire.write(MPU6050_ACCEL_CONFIG);                     // We want to write to the ACCEL_CONFIG register (1A hex).
  Wire.write(0x10);                                     // Set the register bits as 00010000 (+/- 8g full scale range).
  Wire.endTransmission();             

  Wire.beginTransmission(MPU6050_ADDRESS); 
  Wire.write(MPU6050_CONFIG);                            // We want to write to the CONFIG register (1A hex).
  Wire.write(0x03);                                      // Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).
  Wire.endTransmission();            

  if (use_manual_calibration) cal_int = 2000;  
  else {
    cal_int = 0;                     
    gyro_pitch_cal = 0; 
    gyro_roll_cal = 0;  
    gyro_yaw_cal = 0; 
  }
  
  // Let's take multiple gyro data samples so we can determine the calibration parameters.
  if (cal_int != 2000) {                                 // If it isn't 2000 value.
    for (cal_int = 0; cal_int < 2000; cal_int++) {       // Take 2000 readings for calibration.           
      
      read_imu();                                        // Read the gyro output.
                                           
      gyro_roll_cal += gyro_roll;                        // Ad roll value to gyro_roll_cal.          
      gyro_pitch_cal += gyro_pitch;                             
      gyro_yaw_cal += gyro_yaw; 
                                      
      acc_x_cal += acc_x;
      acc_y_cal += acc_y;
      acc_z_cal += acc_z;
      delay(4);  
    }
    gyro_roll_cal /= 2000;                                // Divide the roll total by 2000.
    gyro_pitch_cal /= 2000;  
    gyro_yaw_cal /= 2000; 
    acc_x_cal /= 2000;
    acc_y_cal /= 2000;
    acc_z_cal /= 2000;
    manual_gyro_pitch_cal_value = gyro_pitch_cal;         // Pass the data to the calibration values.
    manual_gyro_roll_cal_value = gyro_roll_cal;    
    manual_gyro_yaw_cal_value = gyro_yaw_cal;   
    manual_x_cal_value = acc_x_cal;
    manual_y_cal_value = acc_y_cal; 
    //manual_z_cal_value = acc_z_cal - 4096;                // Reduce value due error factor. (yaw not needed)
     
    read_imu(); 
    
    acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));    //Calculate the total accelerometer vector.
  
    if (abs(acc_y) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
      angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;              //Calculate the pitch angle.
    }
    if (abs(acc_x) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
      angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;               //Calculate the roll angle.
    }
    angle_pitch = angle_pitch_acc;                                                   //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;
  }
}
