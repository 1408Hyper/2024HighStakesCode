// Legacy drivetrain functions e.g. GPS funcs
// You can put these functions back in at any time and they should work just fine.

void BangGPS(double pos, int velocity = -200) {
			bool direction = pos > 0;

			int lastPos = 0;

			int difference = 0;
			if (direction) {
				lastPos = gps.get_position_x();
			} else {
				lastPos = gps.get_position_y();
			}
			float motorPos = 0;

			moveSingleVelocity(velocity);

			while (true) {
				if (direction) {
					motorPos = gps.get_position_x();
				} else {
					motorPos = gps.get_position_y();
				}

				difference += motorPos - lastPos;

				if (difference >= pos) {
					break;
				}

				lastPos = motorPos;
				pros::delay(20);
			}

			moveStop();
		}

		/// @brief Move to a specific position using PID with GPS
		/// @param pos Position to move to in inches (use negative for backward)
		// TODO: Tuning required
		// direction bool is for x, invert for y
		void PIDGps(double pos, bool direction = true, PIDOptions options = {
			0.1, 0.0, 0.0, 0.1, 6000
		}) {
			pos /= 1;
			//pos *= -1;

			float motorPos = 0;
			float derivative = 0;
			float integral = 0;
			float out = 0;

			float maxCycles = options.timeLimit / moveDelayMs;
			float cycles = 0;

			bool onFirstRun = true;
			bool targetPositive = true;

			if (direction) {
				pos += gps.get_position_y();
			} else {
				pos += gps.get_position_x();
			}

			float error = pos;

			int lastErrorTimes = 0;
			float lastError = error;
			derivative = error - lastError;
			integral = error;

			pros::lcd::print(2, ("GPS Pos: " + std::to_string(gps.get_position_x())).c_str());

			// with moving you just wanna move both MGs at the same speed

			while (true) {
				// get avg error
				if (direction) {
					motorPos = gps.get_position_y();
				} else {
					motorPos = gps.get_position_x();
				}

				error = pos - motorPos;

				integral += error;
				// Anti windup
				if (std::fabs(error) < options.errorThreshold) {
					integral = 0;
				}

				derivative = error - lastError;

				if (onFirstRun) {
					if (derivative > 0) {
						targetPositive = true;
					} else {
						targetPositive = false;
					}

					onFirstRun = false;
				}

				out = (options.kP * error) + (options.kI * integral) + (options.kD * derivative);
				lastError = error;

				out *= -0.01; // convert to mV
				out = std::clamp(out, maxVoltage, -maxVoltage);
				moveSingleVoltage(out);

				pros::lcd::print(4, ("GPS " + std::to_string(motorPos)).c_str());
				pros::lcd::print(7, ("PIDMove Out: " + std::to_string(out)).c_str());
				pros::lcd::print(5, ("PIDMove Error: " + std::to_string(error)).c_str());

				master->print(0, 0, ("error: " + std::to_string(error)).c_str());

				if (std::fabs(error) <= options.errorThreshold) {
					master->print(0, 0, "PIDGPS YAY %f", error);
					break;
				}

				if (std::fabs(error) >= std::fabs(lastError)) {
					lastErrorTimes++;
				}

				if (lastErrorTimes >= 3) {
					//pros::lcd::print(7, "PIDGPS ERROR LIMIT REACHED");
					//break;
				}

				if (integral >= pos) {
					//pros::lcd::print(7, "PIDGPS INTEGRAL LIMIT REACHED");
					//break;
				}

				if (cycles >= maxCycles) {
					pros::lcd::print(4, "PIDMove Time limit reached");
					break;
				}

				pros::delay(moveDelayMs);
				cycles++;
			}

			moveStop();
		}	
