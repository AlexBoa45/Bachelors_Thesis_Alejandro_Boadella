void read_process_units() {
  read_rc();
  read_imu();
  read_battery();
  process_imu();

}

// Process radio control data:
void read_rc() {
  if (flank_count == 18) {                                                                                  // If the count reaches 18 = 2*channels + 2, the whole signal has been read.
    for (int i = 1; i <= number_channels; i++) {
      remote_channel[i] = map(pulse_instant[2 * i] - pulse_instant[2 * i - 1], 600, 1600, 1000, 2000);      // It reads the fall and rise of the signal, does the variation and maps the variation into an interval from 1000-2000us.
    }
  }
}

// Reads battery voltage:
void read_battery() {
  
  // Load the battery voltage to the battery_voltage variable.
  // The STM32 uses a 12 bit analog to digital converter.
  // analogRead => 0 = 0V ..... 4095 = 3.3V
  // The voltage divider (1k & 10k) is 1:11.
  // analogRead => 0 = 0V ..... 4095 = 36.3V
  // 36.3 / 4095 = 112.81.
  // The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  battery_voltage = (float)analogRead(PA5) / 112.81;
  
  // The battery voltage is needed for compensation.
  // A complementary filter is used to reduce noise.
  // 1410.1 = 112.81 / 0.08.
  battery_voltage = battery_voltage * 0.92 + ((float)analogRead(PA5) / 1410.1);                           // If it gives a lower value, map the output to the real values!!!
  
  // Default setting is 10.5V 3S.
  if (battery_voltage > 6.0 && battery_voltage < low_battery_warning && error == 0)Serial.println("error bat");

}

// Reads IMU/MPU-6050 data:
void read_imu(void) {
  Wire.beginTransmission(MPU6050_ADDRESS);                      // Start communication with the gyro.
  Wire.write(MPU6050_ACCEL_XOUT_H);                             // Start reading @ register 43h and auto increment with every read.
  Wire.endTransmission();                                       // End the transmission.
  Wire.requestFrom(MPU6050_ADDRESS, 14);                        // Request 14 bytes from the MPU 6050.
  acc_y = Wire.read() << 8 | Wire.read();                   
  acc_x = Wire.read() << 8 | Wire.read();                       // Add the low and high byte to the acc variable.
  acc_z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();               
  gyro_roll = Wire.read() << 8 | Wire.read();                   // Read high and low part of the angular data.
  gyro_pitch = Wire.read() << 8 | Wire.read();
  gyro_yaw = Wire.read() << 8 | Wire.read();
  gyro_pitch *= -1;                                             // Invert the direction of the axis. See the angles, if they are not right, comment this.
  gyro_yaw *= -1;

  acc_x -= manual_x_cal_value;                                  // If it isn't in the calibration mode it won't have 0 as a value.
  acc_y -= manual_y_cal_value;
  acc_z -= manual_z_cal_value;
  gyro_roll -= manual_gyro_roll_cal_value;                      // Subtact the manual gyro roll calibration value.
  gyro_pitch -= manual_gyro_pitch_cal_value;
  gyro_yaw -= manual_gyro_yaw_cal_value;
}

// Process the IMU:
void process_imu() {
 // 65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);     // Gyro pid input is deg/sec. 
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);  
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);     
  
  // Gyro angle calculations
  // 0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += (float)gyro_pitch * 0.0000611;                                     // Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += (float)gyro_roll * 0.0000611;   
  angle_yaw += (float)gyro_yaw * 0.0000611;                                       
  if (angle_yaw < 0) angle_yaw += 360;                                             // If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
  else if (angle_yaw >= 360) angle_yaw -= 360;                                     // If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.

 // 0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);                   // If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);                   // If the IMU has yawed transfer the pitch angle to the roll angel.

  // Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));    // Calculate the total accelerometer vector.

  if (abs(acc_y) < acc_total_vector) {                                             // Prevent the asin function to produce a NaN.
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;              // Calculate the pitch angle.
  } 
  if (abs(acc_x) < acc_total_vector) {                                             // Prevent the asin function to produce a NaN.
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;               // Calculate the roll angle.
  }

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                   //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                      //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15;                                           //Calculate the pitch angle correction.
  roll_level_adjust = angle_roll * 15;                                             //Calculate the roll angle correction.

  if (!auto_level) {                                                               //If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0;                                                        //Set the pitch angle correction to zero.
    roll_level_adjust = 0;                                                         //Set the roll angle correcion to zero.
  }
   
    //Serial.println(angle_pitch);
    //Serial.println(angle_roll);
    //Serial.println(angle_yaw);
    //Serial.println(acc_y);
    //Serial.println(acc_x);
    
     
}


// The following subrouting calculates the smallest difference between two heading values:
float course_deviation(float course_b, float course_c) {
  course_a = course_b - course_c;
  if (course_a < -180 || course_a > 180) {
    if (course_c > 180)base_course_mirrored = course_c - 180;
    else base_course_mirrored = course_c + 180;
    if (course_b > 180)actual_course_mirrored = course_b - 180;
    else actual_course_mirrored = course_b + 180;
    course_a = actual_course_mirrored - base_course_mirrored;
  }
  return course_a;
}

// Reads the compass:
void read_compass() {
  Wire.beginTransmission(compass_address);                     // Start communication with the compass.
  Wire.write(0x03);                                            // We want to start reading at the hexadecimal location 0x03.
  Wire.endTransmission();                                      // End the transmission with the gyro.

  Wire.requestFrom(compass_address, 6);                        // Request 6 bytes from the compass.
  compass_y = Wire.read() << 8 | Wire.read();                  // Add the low and high byte to the compass_y variable.
  compass_y *= -1;                                             // Invert the direction of the axis.
  compass_z = Wire.read() << 8 | Wire.read();                  // Add the low and high byte to the compass_z variable.;
  compass_x = Wire.read() << 8 | Wire.read();                  // Add the low and high byte to the compass_x variable.;
  compass_x *= -1;                                             // Invert the direction of the axis.
  
  // Before the compass can give accurate measurements it needs to be calibrated. At startup the compass_offset and compass_scale
  // variables are calculated. The following part will adjust the raw compas values so they can be used for the calculation of the heading.
  
  if (compass_calibration_on == 0) {                            // When the compass is not beeing calibrated.
    compass_y += compass_offset_y;                              // Add the y-offset to the raw value.
    compass_y *= compass_scale_y;                               // Scale the y-value so it matches the other axis.
    compass_z += compass_offset_z;                              // Add the z-offset to the raw value.
    compass_z *= compass_scale_z;                               // Scale the z-value so it matches the other axis.
    compass_x += compass_offset_x;                              // Add the x-offset to the raw value.
  }

  // The compass values change when the roll and pitch angle of the quadcopter changes. That's the reason that the x and y values need to calculated for a virtual horizontal position.
  // The 0.0174533 value is phi/180 as the functions are in radians in stead of degrees.
  
  compass_x_horizontal = (float)compass_x * cos(angle_pitch * -0.0174533) + (float)compass_y * sin(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533) - (float)compass_z * cos(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533);
  compass_y_horizontal = (float)compass_y * cos(angle_roll * 0.0174533) + (float)compass_z * sin(angle_roll * 0.0174533);

  // Now that the horizontal values are known the heading can be calculated. With the following lines of code the heading is calculated in degrees.
  // Please note that the atan2 uses radians in stead of degrees. That is why the 180/3.14 is used.
  if (compass_y_horizontal < 0)actual_compass_heading = 180 + (180 + ((atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14)));
  else actual_compass_heading = (atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14);

  actual_compass_heading += declination;                                 // Add the declination to the magnetic compass heading to get the geographic north.
  if (actual_compass_heading < 0) actual_compass_heading += 360;         // If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
  else if (actual_compass_heading >= 360) actual_compass_heading -= 360; // If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.
}

// Reads GPS data:
void read_gps(void) {
  
  while (ss.available() && new_line_found == 0) {                                                        // Stay in this loop as long as there is serial information from the GPS available.
    char read_serial_byte = ss.read();                                                                   // Load a new serial byte in the read_serial_byte variable.
    if (read_serial_byte == '$') {                                                                       // If the new byte equals a $ character.
      for (message_counter = 0; message_counter <= 99; message_counter ++) {                             // Clear the old data from the incomming buffer array.
        incomming_message[message_counter] = '-';                                                        // Write a - at every position.
      }
      message_counter = 0;                                                                               // Reset the message_counter variable because we want to start writing at the begin of the array.
    }
    else if (message_counter <= 99)message_counter ++;                                                   // If the received byte does not equal a $ character, increase the message_counter variable.
    incomming_message[message_counter] = read_serial_byte;                                               // Write the new received byte to the new position in the incomming_message array.
    if (read_serial_byte == '*') new_line_found = 1;                                                     // Every NMEA line end with a *. If this character is detected the new_line_found variable is set to 1.
  }

  // If the software has detected a new NMEA line it will check if it's a valid line that can be used.
  if (new_line_found == 1) {                                                                             // If a new NMEA line is found.
    new_line_found = 0;                                                                                  // Reset the new_line_found variable for the next line.
    if (incomming_message[4] == 'L' && incomming_message[5] == 'L' && incomming_message[7] == ',') {     // When there is no GPS fix or latitude/longitude information available.
      // Set some variables to 0 if no valid information is found by the GPS module. This is needed for GPS lost when flying.
      l_lat_gps = 0;
      l_lon_gps = 0;
      lat_gps_previous = 0;
      lon_gps_previous = 0;
      number_used_sats = 0;
    }
    
    // If the line starts with GA and if there is a GPS fix we can scan the line for the latitude, longitude and number of satellites.
    if (incomming_message[4] == 'G' && incomming_message[5] == 'A' && (incomming_message[44] == '1' || incomming_message[44] == '2')) {
      lat_gps_actual = ((int)incomming_message[19] - 48) *  (long)10000000;                              // Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[20] - 48) * (long)1000000;                               // Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[22] - 48) * (long)100000;                                // Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[23] - 48) * (long)10000;                                 // Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[24] - 48) * (long)1000;                                  // Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[25] - 48) * (long)100;                                   // Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[26] - 48) * (long)10;                                    // Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual /= (long)6;                                                                         // To convert the minutes to degrees we need to divide the minutes by 6.
      lat_gps_actual += ((int)incomming_message[17] - 48) *  (long)100000000;                            // Add the degrees multiplied by 10.
      lat_gps_actual += ((int)incomming_message[18] - 48) *  (long)10000000;                             // Add the degrees multiplied by 10.
      lat_gps_actual /= 10;                                                                              // Divide everything by 10.

      lon_gps_actual = ((int)incomming_message[33] - 48) *  (long)10000000;                              // Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[34] - 48) * (long)1000000;                               // Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[36] - 48) * (long)100000;                                // Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[37] - 48) * (long)10000;                                 // Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[38] - 48) * (long)1000;                                  // Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[39] - 48) * (long)100;                                   // Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[40] - 48) * (long)10;                                    // Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual /= (long)6;                                                                         // To convert the minutes to degrees we need to divide the minutes by 6.
      lon_gps_actual += ((int)incomming_message[30] - 48) * (long)1000000000;                            // Add the degrees multiplied by 10.
      lon_gps_actual += ((int)incomming_message[31] - 48) * (long)100000000;                             // Add the degrees multiplied by 10.
      lon_gps_actual += ((int)incomming_message[32] - 48) * (long)10000000;                              // Add the degrees multiplied by 10.
      lon_gps_actual /= 10;                                                                              // Divide everything by 10.

      if (incomming_message[28] == 'N')latitude_north = 1;                                               // When flying north of the equator the latitude_north variable will be set to 1.
      else latitude_north = 0;                                                                           // When flying south of the equator the latitude_north variable will be set to 0.

      if (incomming_message[42] == 'E')longiude_east = 1;                                                // When flying east of the prime meridian the longiude_east variable will be set to 1.
      else longiude_east = 0;                                                                            // When flying west of the prime meridian the longiude_east variable will be set to 0.

      number_used_sats = ((int)incomming_message[46] - 48) * (long)10;                                   // Filter the number of satillites from the GGA line.
      number_used_sats += (int)incomming_message[47] - 48;                                               // Filter the number of satillites from the GGA line.

      if (lat_gps_previous == 0 && lon_gps_previous == 0) {                                              // If this is the first time the GPS code is used.
        lat_gps_previous = lat_gps_actual;                                                               // Set the lat_gps_previous variable to the lat_gps_actual variable.
        lon_gps_previous = lon_gps_actual;                                                               // Set the lon_gps_previous variable to the lon_gps_actual variable.
      }

      lat_gps_loop_add = (float)(lat_gps_actual - lat_gps_previous) / 10.0;                              // Divide the difference between the new and previous latitude by ten.
      lon_gps_loop_add = (float)(lon_gps_actual - lon_gps_previous) / 10.0;                              // Divide the difference between the new and previous longitude by ten.

      l_lat_gps = lat_gps_previous;                                                                      // Set the l_lat_gps variable to the previous latitude value.
      l_lon_gps = lon_gps_previous;                                                                      // Set the l_lon_gps variable to the previous longitude value.

      lat_gps_previous = lat_gps_actual;                                                                 // Remember the new latitude value in the lat_gps_previous variable for the next loop.
      lon_gps_previous = lon_gps_actual;                                                                 // Remember the new longitude value in the lat_gps_previous variable for the next loop.

      // The GPS is set to a 5Hz refresh rate. Between every 2 GPS measurments, 9 GPS values are simulated.
      gps_add_counter = 5;                                                                               // Set the gps_add_counter variable to 5 as a count down loop timer
      new_gps_data_counter = 9;                                                                          // Set the new_gps_data_counter to 9. This is the number of simulated values between 2 GPS measurements.
      lat_gps_add = 0;                                                                                   // Reset the lat_gps_add variable.
      lon_gps_add = 0;                                                                                   // Reset the lon_gps_add variable.
      new_gps_data_available = 1;                                                                        // Set the new_gps_data_available to indicate that there is new data available.
    }

    //If the line starts with SA and if there is a GPS fix we can scan the line for the fix type (none, 2D or 3D).
    if (incomming_message[4] == 'S' && incomming_message[5] == 'A')fix_type = (int)incomming_message[9] - 48;

  }

  // After 5 program loops 5 x 4ms = 20ms the gps_add_counter is 0.
  if (gps_add_counter == 0 && new_gps_data_counter > 0) {                                                 // If gps_add_counter is 0 and there are new GPS simulations needed.
    new_gps_data_available = 1;                                                                           // Set the new_gps_data_available to indicate that there is new data available.
    new_gps_data_counter --;                                                                              // Decrement the new_gps_data_counter so there will only be 9 simulations
    gps_add_counter = 5;                                                                                  // Set the gps_add_counter variable to 5 as a count down loop timer

    lat_gps_add += lat_gps_loop_add;                                                                      // Add the simulated part to a buffer float variable because the l_lat_gps can only hold integers.
    if (abs(lat_gps_add) >= 1) {                                                                          // If the absolute value of lat_gps_add is larger then 1.
      l_lat_gps += (int)lat_gps_add;                                                                      // Increment the lat_gps_add value with the lat_gps_add value as an integer. So no decimal part.
      lat_gps_add -= (int)lat_gps_add;                                                                    // Subtract the lat_gps_add value as an integer so the decimal value remains.
    }

    lon_gps_add += lon_gps_loop_add;                                                                      // Add the simulated part to a buffer float variable because the l_lat_gps can only hold integers.
    if (abs(lon_gps_add) >= 1) {                                                                          // If the absolute value of lat_gps_add is larger then 1.
      l_lon_gps += (int)lon_gps_add;                                                                      // Increment the lat_gps_add value with the lat_gps_add value as an integer. So no decimal part.
      lon_gps_add -= (int)lon_gps_add;                                                                    // Subtract the lat_gps_add value as an integer so the decimal value remains.
    }
  }

  if (new_gps_data_available) {                                                                           // If there is a new set of GPS data available.
    gps_watchdog_timer = millis();                                                                        // Reset the GPS watch dog timer.
    new_gps_data_available = 0;                                                                           // Reset the new_gps_data_available variable.

    if (fm == FM_gps_hold && waypoint_set == 0) {                                                         // If the flight mode is 3 (GPS hold) and no waypoints are set.
      waypoint_set = 1;                                                                                   // Indicate that the waypoints are set.
      l_lat_waypoint = l_lat_gps;                                                                         // Remember the current latitude as GPS hold waypoint.
      l_lon_waypoint = l_lon_gps;                                                                         // Remember the current longitude as GPS hold waypoint.
    }

    if (fm ==FM_gps_hold && waypoint_set == 1) {                                                          // If the GPS hold mode and the waypoints are stored and take off detected.
      // GPS stick move adjustments
      if (fm == FM_gps_hold && remote_channel[3] >= 1400) {                                               // ¡¡¡¡¡ 1400us is considered when the drone isn't touching the ground, consider changing !!!!
        if (!latitude_north) {
          l_lat_gps_float_adjust += 0.0015 * (((remote_channel[2] - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((remote_channel[1] - 1500) * cos((gps_man_adjust_heading - 90) * 0.017453))); //South correction
        }
        else {
          l_lat_gps_float_adjust -= 0.0015 * (((remote_channel[2] - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((remote_channel[1] - 1500) * cos((gps_man_adjust_heading - 90) * 0.017453))); //North correction
        }

        if (!longiude_east) {
          l_lon_gps_float_adjust -= (0.0015 * (((remote_channel[1] - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((remote_channel[2] - 1500) * cos((gps_man_adjust_heading + 90) * 0.017453)))) / cos(((float)l_lat_gps / 1000000.0) * 0.017453); //West correction
        }

        else {
          l_lon_gps_float_adjust += (0.0015 * (((remote_channel[1] - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((remote_channel[2] - 1500) * cos((gps_man_adjust_heading + 90) * 0.017453)))) / cos(((float)l_lat_gps / 1000000.0) * 0.017453); //East correction
        }
      }

      if (l_lat_gps_float_adjust > 1) {
        l_lat_waypoint ++;
        l_lat_gps_float_adjust --;
      }
      if (l_lat_gps_float_adjust < -1) {
        l_lat_waypoint --;
        l_lat_gps_float_adjust ++;
      }

      if (l_lon_gps_float_adjust > 1) {
        l_lon_waypoint ++;
        l_lon_gps_float_adjust --;
      }
      if (l_lon_gps_float_adjust < -1) {
        l_lon_waypoint --;
        l_lon_gps_float_adjust ++;
      }

      gps_lon_error = l_lon_waypoint - l_lon_gps;                                                         // Calculate the latitude error between waypoint and actual position.
      gps_lat_error = l_lat_gps - l_lat_waypoint;                                                         // Calculate the longitude error between waypoint and actual position.

      gps_lat_total_avarage -=  gps_lat_rotating_mem[ gps_rotating_mem_location];                         // Subtract the current memory position to make room for the new value.
      gps_lat_rotating_mem[ gps_rotating_mem_location] = gps_lat_error - gps_lat_error_previous;          // Calculate the new change between the actual pressure and the previous measurement.
      gps_lat_total_avarage +=  gps_lat_rotating_mem[ gps_rotating_mem_location];                         // Add the new value to the long term avarage value.

      gps_lon_total_avarage -=  gps_lon_rotating_mem[ gps_rotating_mem_location];                         // Subtract the current memory position to make room for the new value.
      gps_lon_rotating_mem[ gps_rotating_mem_location] = gps_lon_error - gps_lon_error_previous;          // Calculate the new change between the actual pressure and the previous measurement.
      gps_lon_total_avarage +=  gps_lon_rotating_mem[ gps_rotating_mem_location];                         // Add the new value to the long term avarage value.
      gps_rotating_mem_location++;                                                                        // Increase the rotating memory location.
      if ( gps_rotating_mem_location == 35) gps_rotating_mem_location = 0;                                // Start at 0 when the memory location 35 is reached.

      gps_lat_error_previous = gps_lat_error;                                                             // Remember the error for the next loop.
      gps_lon_error_previous = gps_lon_error;                                                             // Remember the error for the next loop.

      // Calculate the GPS pitch and roll correction as if the nose of the multicopter is facing north.
      // The Proportional part = (float)gps_lat_error * gps_p_gain.
      // The Derivative part = (float)gps_lat_total_avarage * gps_d_gain.
      gps_pitch_adjust_north = (float)gps_lat_error * gps_p_gain + (float)gps_lat_total_avarage * gps_d_gain;
      gps_roll_adjust_north = (float)gps_lon_error * gps_p_gain + (float)gps_lon_total_avarage * gps_d_gain;

      if (!latitude_north)gps_pitch_adjust_north *= -1;                                                   // Invert the pitch adjustmet because the quadcopter is flying south of the equator.
      if (!longiude_east)gps_roll_adjust_north *= -1;                                                     // Invert the roll adjustmet because the quadcopter is flying west of the prime meridian.

      // Because the correction is calculated as if the nose was facing north, we need to convert it for the current heading.
      gps_roll_adjust = ((float)gps_roll_adjust_north * cos(angle_yaw * 0.017453)) + ((float)gps_pitch_adjust_north * cos((angle_yaw - 90) * 0.017453));
      gps_pitch_adjust = ((float)gps_pitch_adjust_north * cos(angle_yaw * 0.017453)) + ((float)gps_roll_adjust_north * cos((angle_yaw + 90) * 0.017453));

      // Limit the maximum correction to 300. This way we still have full controll with the pitch and roll stick on the transmitter.
      if (gps_roll_adjust > 300) gps_roll_adjust = 300;
      if (gps_roll_adjust < -300) gps_roll_adjust = -300;
      if (gps_pitch_adjust > 300) gps_pitch_adjust = 300;
      if (gps_pitch_adjust < -300) gps_pitch_adjust = -300;
    }
  }

  if (gps_watchdog_timer + 1000 < millis()) {                                                             // If the watchdog timer is exceeded the GPS signal is missing.
    if (fm == FM_gps_hold) {                                                                              // If flight mode is set to (GPS hold).
      fm = FM_stable;                                                                                     // Set the flight mode to 2.
      //Serial.println("Error GPS");
    }
  }

  if (fm < 4 && waypoint_set > 0) {                                                                       // If the GPS hold mode is disabled and the waypoints are set.
    gps_roll_adjust = 0;                                                                                  // Reset the gps_roll_adjust variable to disable the correction.
    gps_pitch_adjust = 0;                                                                                 // Reset the gps_pitch_adjust variable to disable the correction.
    if (waypoint_set == 1) {                                                                              // If the waypoints are stored
      gps_rotating_mem_location = 0;                                                                      // Set the gps_rotating_mem_location to zero so we can empty the
      waypoint_set = 2;                                                                                   // Set the waypoint_set variable to 2 as an indication that the buffer is not cleared.
    }
    gps_lon_rotating_mem[ gps_rotating_mem_location] = 0;                                                 // Reset the current gps_lon_rotating_mem location.
    gps_lat_rotating_mem[ gps_rotating_mem_location] = 0;                                                 // Reset the current gps_lon_rotating_mem location.
    gps_rotating_mem_location++;                                                                          // Increment the gps_rotating_mem_location variable for the next loop.
    if (gps_rotating_mem_location == 36) {                                                                // If the gps_rotating_mem_location equals 36, all the buffer locations are cleared.
      waypoint_set = 0;                                                                                   // Reset the waypoint_set variable to 0.
      //Reset the variables that are used for the D-controller.
      gps_lat_error_previous = 0;
      gps_lon_error_previous = 0;
      gps_lat_total_avarage = 0;
      gps_lon_total_avarage = 0;
      gps_rotating_mem_location = 0;
      //Reset the waypoints.
      l_lat_waypoint = 0;
      l_lon_waypoint = 0;
    }
  }
}
