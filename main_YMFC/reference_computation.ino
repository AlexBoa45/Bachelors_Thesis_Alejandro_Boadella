void reference_computation(){
	ref_mode_management();
	ref_gen();
}

void ref_mode_management(){
    
  if (remote_channel[3] < 1100 && remote_channel[4] < 1100) fm = FM_mounting;                             // Enables mounting mode.
  if (fm == FM_mounting && remote_channel[3] < 1100 && remote_channel[4] > 1450) fm = FM_stable;          // Enables stable mode after mounting mode has been reached.
  if (fm >=2 && remote_channel[3] < 1050 && remote_channel[4] > 1950) fm = FM_disabled;                   // Enables disabled mode.
  if (fm >=2 && remote_channel[6] < 1100) fm = FM_stable;                                                 // Enables stable mode.
  if (fm >=2 && remote_channel[6] >= 1100 && remote_channel[6] <= 1900) fm = FM_stable_battery;           // Enables stable battery mode.
  if (fm >=2 && remote_channel[6] > 1900) fm = FM_gps_hold;                                               // Enables gps mode.
}

void ref_gen(){

	if(fm == FM_mounting){                        // Starts the drone parameters.
		  throttle = 950;
		  angle_pitch = angle_pitch_acc;
	    angle_roll = angle_roll_acc;
      pid_i_mem_roll = 0;
      pid_last_roll_d_error = 0;
      pid_output_roll = 0;
      pid_i_mem_pitch = 0;
      pid_last_pitch_d_error = 0;
      pid_output_pitch = 0;
      pid_i_mem_yaw = 0;
      pid_last_yaw_d_error = 0;
      pid_output_yaw = 0;  
	}

	if(fm >=2){                                   // Considers the throttle with the channel 3.
		throttle = remote_channel[3];
	}
}
