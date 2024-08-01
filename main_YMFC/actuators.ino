
void actuators() {
  act_esc_outputs();
  act_esc_PWM();
}

void act_esc_outputs() {
  if (fm >=2) {                                                      
    if (throttle > 1800) throttle = 1800;                                          // Limit max throttle.        
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 1 (front-right - CCW).
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 2 (rear-right - CW).
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 4 (front-left - CW).

    if (battery_voltage < 12.40 && battery_voltage > 6.0 && fm == FM_stable_battery) {                  //Is the battery connected and in the battery mode?
      esc_1 += (12.40 - battery_voltage) * battery_compensation;                   //Compensate the esc-1 pulse for voltage drop.
      esc_2 += (12.40 - battery_voltage) * battery_compensation;                   //Compensate the esc-2 pulse for voltage drop.
      esc_3 += (12.40 - battery_voltage) * battery_compensation;                   //Compensate the esc-3 pulse for voltage drop.
      esc_4 += (12.40 - battery_voltage) * battery_compensation;                   //Compensate the esc-4 pulse for voltage drop.
    }
  
  }
  else {
    esc_1 = 1000;                               
    esc_2 = 1000; 
    esc_3 = 1000; 
    esc_4 = 1000; 
  }

  if (esc_1 < 1000) esc_1 = 950;                //Limit the esc's pulse to 950us.
  if (esc_2 < 1000) esc_2 = 950;  
  if (esc_3 < 1000) esc_3 = 950; 
  if (esc_4 < 1000) esc_4 = 950; 

  if (esc_1 > 2000) esc_1 = 2000;               //Limit the esc's pulse to 2000us.
  if (esc_2 > 2000) esc_2 = 2000;  
  if (esc_3 > 2000) esc_3 = 2000;  
  if (esc_4 > 2000) esc_4 = 2000; 
}

void act_esc_PWM(){
  TIM_M1_M2->setCaptureCompare(channel_motor1, esc_1, MICROSEC_COMPARE_FORMAT);   // Send pulse to the motors.
  TIM_M1_M2->setCaptureCompare(channel_motor2, esc_2, MICROSEC_COMPARE_FORMAT);
  TIM_M3_M4->setCaptureCompare(channel_motor3, esc_3, MICROSEC_COMPARE_FORMAT);
  TIM_M3_M4->setCaptureCompare(channel_motor4, esc_4, MICROSEC_COMPARE_FORMAT);
}
