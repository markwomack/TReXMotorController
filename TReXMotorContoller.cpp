/*
 * Sets the speed for the motors. Values are expected
 * to be between -1 and 1. Values outside this range
 * will be pinned.
 */
void setMotors(double new_motor_left_speed, double new_motor_right_speed) {
  motor_left_speed = new_motor_left_speed;
  motor_right_speed = new_motor_right_speed;
  
  byte command_byte_left = 0xCC;
  if (motor_left_speed < 0) {
    command_byte_left = command_byte_left | 0x02;
  } else {
    command_byte_left = command_byte_left | 0x01;
  }
  int trex_left_speed = min(127, 127.0 * abs(motor_left_speed));

  byte command_byte_right = 0xC4;
  if (motor_right_speed < 0) {
    command_byte_right = command_byte_right | 0x01;
  } else {
    command_byte_right = command_byte_right | 0x02;
  }
  int trex_right_speed = min(127, 127.0 * abs(motor_right_speed));
  
  trex.write(command_byte_left);
  trex.write((byte)trex_left_speed);
  trex.write(command_byte_right);
  trex.write((byte)trex_right_speed);
}

/*
 * Returns the calculated angular velocity from all of
 * the samples. currently it just returns the average
 * of all the samples.
 */
double getAngularVelocityFromSamples(double* samples) {
  double total = 0;
  for (size_t count = 0; count < NUM_VEL_SAMPLES; count++) {
    total += samples[count];
  }
  return total / NUM_VEL_SAMPLES;
}

bool setTRexConfiguration(byte parameter, byte value) {
  trex.write(0xAF);      // Set configuration parameter command
  trex.write(parameter); // Set parameter to change
  trex.write(value);     // Set the value
  trex.write(0x55);      // constant format byte 1
  trex.write(0x2A);      // constant format byte 2
  while (!trex.available()) {}
  return (trex.read() != 0x00);
}

/*
 * Starts up the TReX motor controller. Returns false if
 * started without any error.
 */
bool startTRex() {
  // TODO(mwomack): Move all the TRex stuff into a helper class!
  static const byte SERIAL_TIMEOUT_PARAM(0x07);
  static const byte MOTOR_1_ACCELERATION_PARAM(0x0E);
  static const byte MOTOR_2_ACCELERATION_PARAM(0x0F);
  
  // Start serial port to TRex
  trex.begin(19200);
  while (!trex) { }

  // Set serial timeout to motor_controller_cmd_timeout
  byte timeout = max(127, motor_controller_cmd_timeout * 10);
  if (setTRexConfiguration(SERIAL_TIMEOUT_PARAM, timeout)) {
    return true;
  }

  // Set motor 1 and 2 acceleration
  byte motor_acceleration = max(127, motor_controller_acceleration);
  if (setTRexConfiguration(MOTOR_1_ACCELERATION_PARAM, motor_acceleration)) {
    return true;
  }
  if (setTRexConfiguration(MOTOR_2_ACCELERATION_PARAM, motor_acceleration)) {
    return true;
  }

  return false;
}
