// includes/usings are all in main.h
#include "main.h"

// nothing to see here, move along
																																																																																																									#define _HYPER_UNLEASH_HELL delete this, *(reinterpret_cast<int*>(this) + 1) = 0xDEADBEEF;
// uses ISO/C++20 standard
// added libraries/includes:
// pros

// ending in -mech classes are for pneumatics

// currently using legacy toggles (not using MechToggle class):
// mogomech - maybe try to upgrade to MechToggle?

// CONSIDER odom?

// WARNING: do NOT just put "Args" as the name of an args struct in any class
// instead, put the class name in front of it (e.g. DrivetrainArgs) for CLARITY
// in derived functions & then for factories just do using e.g. using ArgsType = DrivetrainArgs;

// for printing to brain and controller pls USE NEW LOG AND TELL FUNCS!!!

// TODO: seperate PID functions into a separate class for cleanliness

/// @brief Hyper namespace for all custom classes and functions
namespace hyper {
	// Function declarations
	template <typename T>
	string vectorToString(vector<T>& vec, string delimiter = ", ");

	std::int32_t prepareMoveVoltage(float raw);

	template <typename T>
	bool isNumBetween(T num, T min, T max);

	template <typename T>
	T normaliseAngle(T angle);

	template <typename T>
	T naiveNormaliseAngle(T angle);

	/*template <typename T>
	vector<T> getAllValues() {}

	template <typename E, typename V>
	void fillMapWithEnum(map<E, V>& map) {}*/
	
	template <typename T>
	T calcMeanFromVector(const vector<T>& vec);

	template <typename T>
	T calcMeanFromVector(const vector<T>& vec, int size);

	// Structs

	/// @brief Struct for motor group buttons (manual control)
	/// @param fwd Button for forward
	/// @param back Button for backward

	struct Buttons {
		pros::controller_digital_e_t fwd;
		pros::controller_digital_e_t back;
	};

	/// @brief Struct for motor move bounds
	struct MotorBounds {
		static constexpr std::int32_t MOVE_MIN = -127;
		static constexpr std::int32_t MOVE_MAX = 127;
		static constexpr std::int32_t MILLIVOLT_MAX = 12000;
	};

	// Class declarations

	/// @brief Abstract chassis class for if you want a custom chassis class
	class AbstractChassis {
	private:
	protected:
		pros::Controller master{pros::E_CONTROLLER_MASTER};
	public:
		/// @brief Creates abstract chassis object
		AbstractChassis() {
																																																																													// :) u know what this does
																																																																													#ifndef _HYPER_UNLOCK_66_75_63_6B
																																																																														_HYPER_UNLEASH_HELL
																																																																													#endif
		};

		virtual ~AbstractChassis() = default;

		/// @brief Gets the controller
		pros::Controller& getController() {
			return master;
		}

		virtual void opControl() = 0;
		virtual void auton() = 0;
		virtual void skillsPrep() = 0;
		virtual void skillsAuton() = 0;
		virtual void postAuton() = 0;
	}; // class AbstractChassis

	/// @brief Class for components of the chassis to derive from
	class AbstractComponent {
	private:
	public:
		static constexpr std::uint8_t MAX_BRAIN_LINES = 8;
		static constexpr std::uint8_t MAX_CONTROLLER_LINES = 2;
		static constexpr std::uint8_t CONTROLLER_TXT_START_COL = 0;
	protected:
		AbstractChassis* chassis;

		pros::Controller* master;
	public:
		/// @brief Args for AbstractComponent object
		/// @param chassis AbstractChassis derived object to be used for the component
		struct AbstractComponentArgs {
			AbstractChassis* chassis;
		};

		/// @brief Creates AbstractComponent object
		/// @param args Args AbstractComponent object (check args struct for more info)
		AbstractComponent(AbstractComponentArgs args) : 
		chassis(args.chassis),
		master(&args.chassis->getController()) {
			// :) u know what this does
																																																																																		#ifndef _HYPER_UNLOCK_66_75_63_6B
																																																																																			_HYPER_UNLEASH_HELL
																																																																																		#endif
		};

		/// @brief Log something to the brain safely
		/// @param line Line to print the message on (check class consts for max lines)
		/// @param message Message to print
		/// @param additional Additional arguments to print
		/// @return Success/fail state of the brain printing
		template <typename... T>
		bool log(const std::uint8_t line, const string& message, T&&... additional) {
			if (line > MAX_BRAIN_LINES) {
				return false;
			}

			pros::lcd::print(line, message.c_str(), additional...);
			return true;
		}

		/// @brief Tell the driver something via the controller safely
		/// @param line Line to print the message on (check class consts for max lines)
		/// @param message Message to print
		/// @param additional Additional arguments to print
		/// @return Success/fail state of the controller printing
		template <typename... T>
		bool tell(const std::uint8_t line, const string& message, T&&... additional) {
			if (line > MAX_CONTROLLER_LINES) {
				return false;
			}

			master->print(line, CONTROLLER_TXT_START_COL, message.c_str(), additional...);
			return true;
		}

		AbstractChassis& getChassis() {
			return *chassis;
		}

		pros::Controller& getMaster() {
			return *master;
		}

		virtual void opControl() = 0;

		virtual void postAuton() {}
		virtual void skillsPrep() {}

		virtual ~AbstractComponent() = default;
	}; // class ChassisComponent

	class AbstractMech : public AbstractComponent {
	private:
		bool engaged = false;

		pros::adi::DigitalOut piston;
	protected:
	public:
		/// @brief Args for abstract mech object
		/// @param abstractComponentArgs Args for AbstractComponent object
		/// @param pistonPort Port for piston
		struct AbstractMechArgs {
			AbstractComponentArgs abstractComponentArgs;
			char pistonPort;
		};

		/// @brief Creates abstract mech object
		/// @param args Args for abstract mech object (check args struct for more info)
		AbstractMech(AbstractMechArgs args) : 
			AbstractComponent(args.abstractComponentArgs),
			piston(args.pistonPort) {};

		/// @brief Sets actuation value of piston
		/// @param value Value to set the piston to
		void actuate(bool value) {
			piston.set_value(value);
			engaged = value;
		}

		/// @brief Gets the piston object
		/// @return PROS ADI DigitalOut object for piston
		pros::adi::DigitalOut& getPiston() {
			return piston;
		}

		/// @brief Gets the engaged state of the mech
		/// @return Engaged state of the mech
		bool getEngaged() {
			return engaged;
		}

		/// @brief Sets the engaged state of the mech (DO NOT USE UNLESS ABSOLUTELY NECESSARY!!)
		/// @param value Value to set the engaged state to (DO NOT USE UNLESS ABSOLUTELY NECESSARY!!)
		void doNotUseThisInYourLifeEver_ForceSetEngaged(bool value) {
			engaged = value;
		}

		virtual ~AbstractMech() = default;
	}; // class AbstractMech

	/// @brief Abstract motor group class for if you want a custom motor group class
	class AbstractMG : public AbstractComponent {
	private:		
	protected:
		const pros::MotorGroup mg;
	public:
		struct Speeds {
			int fwd = 10000;
			int back = -10000;
		};

		/// @brief Args for abstract motor group object
		/// @param abstractComponentArgs Args for AbstractComponent object
		/// @param ports Vector of ports for motor group
		struct AbstractMGArgs {
			AbstractComponentArgs abstractComponentArgs;
			MGPorts ports;
		};

		Speeds speeds = {};
		bool outputSpeeds;

		/// @brief Constructor for abstract motor group object
		/// @param args Args for abstract motor group object (check args struct for more info)
		AbstractMG(AbstractMGArgs args) : 
			AbstractComponent(args.abstractComponentArgs),
			mg(args.ports) {};

		/// @brief Move the motors in the specified direction according to speeds.
		/// @param on Whether to stop or start the motors.
		/// @param directionForward Direction to go.
		void move(bool on, bool directionForward = true) {
			on = canMove(on);

			if (on) {
				if (directionForward) {
					mg.move_velocity(speeds.fwd);
					if (outputSpeeds) {
						//pros::lcd::print(2, "motor going!!");
					}
				} else {
					mg.move_velocity(speeds.back);
					if (outputSpeeds) {
						//pros::lcd::print(2, "motor not going :(");
					}
				}
			} else {
				mg.move_velocity(0);
			}
		}

		virtual bool canMove(bool on) = 0;

		virtual ~AbstractMG() = default;
	}; // class AbstractMG

	/// @brief Abstract class for general PID which can be used as a template for specific PID functions.
	class AbstractPID {
	private:
		bool needSetupOptions = true;
	protected:
		pros::MotorGroup* left_mg;
		pros::MotorGroup* right_mg;

		struct PIDOptions {
			double kP;
			double kI;
			double kD;
			double errorThreshold;
			float timeLimit;
		};

		struct PIDRuntimeVars {
			float error = 0;
			float lastError = 0;
			float derivative = 0;
			float integral = 0;
			float out = 0;
			float cycles = 0;
			bool running = true;
			const float maxCycles = 0;
			
			PIDRuntimeVars(float timeLimitMs, float moveDelayMs) : 
				maxCycles(timeLimitMs / moveDelayMs) {}
		};

		// Setup constants for the PID
		virtual PIDOptions setupOptions() = 0;
		// Change the input so that it is suitable for the PID
		virtual std::int32_t preprocessInput(std::int32_t target) = 0;
		// Get the motor position/IMU position
		virtual std::int32_t getPos() = 0;

		// Change the output voltage to one which is suitable
		virtual std::int32_t postprocessOutput(std::int32_t output) = 0;
		// Add additional code to the main loop
		virtual void mainloopInject(PIDRuntimeVars& rv) {};
	private:
		PIDOptions options;
	public:
		struct AbstractPIDArgs {
			pros::MotorGroup* left_mg;
			pros::MotorGroup* right_mg;
		};

		AbstractPID(AbstractPIDArgs args) : 
			left_mg(args.left_mg),
			right_mg(args.right_mg) {};

		void move() {
			if (needSetupOptions) {
				options = setupOptions();
				needSetupOptions = false;
			}
		}
	}; // class AbstractPID

	/// @brief PID specifically for lateral drivetrain movement.
	class LateralPID : public AbstractPID {
	private:

	protected:

	public:

	}; // class LateralPID

	/// @brief PID specifically for turning drivetrain movement.
	class TurnPID : public AbstractPID {
	private:
	
	protected:

	public:

	}; // class TurnPID

	/// @brief Class which manages button presses (will run function on up, down and hold states of given button)
	class BtnManager : public AbstractComponent {
	private:
		bool lastPressed = false;

		void handleBtnPressed() {
			if (lastPressed) {
				for (VoidFunc& func: actionInfo.holdFuncs) {
					func();
				}
			} else {
				for (VoidFunc& func: actionInfo.downFuncs) {
					func();
				}
			}
		}
	protected:
	public:
		/// @brief Struct for action info for button manager object
		/// @param upFuncs Functions that are run once when up state is reached
		/// @param downFuncs Functions that are run once when down state is reached
		/// @param holdFuncs Functions to continuously run on hold state
		/// @param btn Button to manage
		struct ActionInfo {
			pros::controller_digital_e_t btn;
			VoidFuncVector downFuncs = {};
			VoidFuncVector upFuncs = {};
			VoidFuncVector holdFuncs = {};
		};

		ActionInfo actionInfo;

		/// @brief Args for button manager object
		/// @param abstractComponentArgs Args for AbstractComponent object
		/// @param actionInfo Action info for button manager object
		struct BtnManagerArgs {
			AbstractComponentArgs abstractComponentArgs;
			ActionInfo actionInfo;
		};

		/// @brief Creates button manager object
		/// @param args Args for button manager object (check args struct for more info)
		BtnManager(BtnManagerArgs args) : 
			AbstractComponent(args.abstractComponentArgs),
			actionInfo(args.actionInfo) {};

		void opControl() override {
			bool btnPressed = master->get_digital(actionInfo.btn);
			
			// down: !lastPressed && btnPressed
			// up: lastPressed && !btnPressed
			// hold: lastPressed && btnPressed

			if (btnPressed) {
				handleBtnPressed();
			} else if (lastPressed) {
				for (VoidFunc& func: actionInfo.upFuncs) {
					func();
				}
			}

			lastPressed = btnPressed;
		}

		bool getLastPressed() {
			return lastPressed;
		}
	};

	/// @brief Class for a toggle on the controller
	class MechToggle {
	private:
		struct ToggleFuncs {
			std::function<void()> offFunc;
			std::function<void()> onFunc;
		};

		bool lastPressed = false;

		pros::Controller* master;
		ToggleFuncs funcs;

		void toggle() {
			if (st.state) {
				funcs.offFunc();
				st.state = false;
			} else {
				funcs.onFunc();
				st.state = true;
			}
		}
	protected:
	public:
		/// @brief Struct for static functions for toggle object
		/// @param offFunc Function to toggle off (static)
		/// @param onFunc Function to toggle on (static)
		struct StaticFuncs {
			void (AbstractMech::*offFunc)();
			void (AbstractMech::*onFunc)();
		};

		/// @brief Static options for toggle object
		/// @param btn Button for toggle
		/// @param funcs Functions for toggle
		/// @param state Initial state for toggle
		struct StaticOptions {
			pros::controller_digital_e_t btn;
			bool state = false;
		};

		/// @brief Args for toggle object
		/// @param master Controller for robot
		/// @param st Static options for toggle object
		struct MechToggleArgs {
			pros::Controller* master;
			AbstractMech* component;
			StaticFuncs staticFuncs;
			StaticOptions st;
		};

		StaticOptions st;

		/// @brief Creates toggle object
		/// @param args Args for toggle object (check args struct for more info)
		MechToggle(MechToggleArgs args) : 
			master(args.master), 
			funcs({
				std::bind(args.staticFuncs.offFunc, args.component), 
				std::bind(args.staticFuncs.onFunc, args.component)
			}),
			st(args.st) {};

		/// @brief Run every single loop to check if the button has been pressed
		void opControl() {
			if (master->get_digital(st.btn)) {
				if (!lastPressed) {
					toggle();
				}
				lastPressed = true;
			} else {
				lastPressed = false;
			}
		}

		/// @brief Gets the toggle functions
		/// @return Toggle functions
		ToggleFuncs& getFuncs() {
			return funcs;
		}
	}; // class Toggle

	/// @brief Class for a toggle on the controller
	class BiToggle { // Don't need to derive from AbstractComponent because no need for Chassis pointer
	public:
		enum class State {
			OFF,
			FWD,
			BACK
		};
	private:
		AbstractMG* component;

		pros::Controller* master;
		
		State state = State::OFF;
		bool isNewPress = true;

		void moveState(State target) {
			if (!isNewPress) {
				return;
			}

			switch (target) {
				case State::OFF:
					component->move(false);
					break;
				case State::FWD:
					component->move(true);
					break;
				case State::BACK:
					component->move(true, false);
					break;
			}
			
			state = target;
		}

		void handleFwdBtn() {
			if (state == State::FWD) {
				moveState(State::OFF);
				pros::lcd::print(1, "Fwd pressed AND GOING OFF");
			} else {
				moveState(State::FWD);
				pros::lcd::print(1, "Fwd pressed AND GOING FWD");
			}
		}

		void handleBackBtn() {
			if (state == State::BACK) {
				moveState(State::OFF);
				pros::lcd::print(1, "Back pressed AND GOING OFF");
			} else {
				moveState(State::BACK);
				pros::lcd::print(1, "Back pressed AND GOING BACK");
			}
		}
	protected:
	public:
		/// @brief Struct for buttons for BiToggle object
		/// @param fwd Button for forward
		/// @param back Button for backward
		struct Buttons {
			pros::controller_digital_e_t fwd;
			pros::controller_digital_e_t back;
		};

		/// @brief Args for BiToggle object
		/// @param component Component to toggle
		/// @param btns Buttons for toggle
		struct BiToggleArgs {
			AbstractMG* component;
			Buttons btns;
		};

		Buttons btns;

		/// @brief Creates BiToggle object
		/// @param args Args for BiToggle object (check args struct for more info)
		BiToggle(BiToggleArgs args) : 
			component(args.component),
			btns(args.btns),
			master(&args.component->getMaster()) {};

		void opControl() {
			bool fwdPressed = master->get_digital(btns.fwd);
			bool backPressed = master->get_digital(btns.back);

			pros::lcd::print(3, ("FWD: " + std::to_string(fwdPressed)).c_str());
			pros::lcd::print(4, ("BACK: " + std::to_string(backPressed)).c_str());

			if (fwdPressed && backPressed) {
				// Don't do anything if both are pressed
				// TODO: test whether the return works
				// because we need it for backwards motor movement
				return;
			}

			if (fwdPressed) {
				pros::lcd::print(2, "Begin CONVEYER FWD");
				handleFwdBtn();
				isNewPress = false;
				return;
			}

			if (backPressed) {
				pros::lcd::print(2, "Begin CONVEYER BACK");
				handleBackBtn();
				isNewPress = false;
				return;
			}

			isNewPress = true;
		}

		void setState(State target) {
			state = target;
		}

		State getState() {
			return state;
		}
	}; // class BiToggle

	/// @brief Class for driver control
	class Drivetrain : public AbstractComponent {
	public:
		/// @brief Enum for different driver control modes
		enum class DriveControlMode {
			ARCADE

		};
	private:
		// Coefficients for turning in driver control
		struct TurnCoefficients {
			float left;
			float right;
		};

		pros::MotorGroup left_mg;
		pros::MotorGroup right_mg;
		int leftMgSize;
		int rightMgSize;

		// PID stuff
		// TODO: Move to separate class
		pros::IMU imu;
		pros::adi::Encoder odomEnc;
		pros::Gps gps;

		DriveControlMode driveControlMode;

		std::function<void()> driveControl;

		void bindDriveControl(void (Drivetrain::*driveFunc)()) {
			driveControl = std::bind(driveFunc, this);
		}
	protected:
	public:
		/// @brief Struct for different driver control speeds
		/// @param turnSpeed Multiplier for only turning
		/// @param forwardBackSpeed Multiplier for only forward/backward
		/// @param arcSpeed Multiplier of opposite turn for when turning and moving laterally at the same time
		// (higher value means less lateral movement)
		struct DriveControlSpeed {
			private:
				float forwardBackSpeed;
				float maxLateral;
			public:
				static constexpr float controllerMax = 127;

				float turnSpeed;
				float arcSpeed;

				/// @brief Sets the forward/backward speed
				/// @param speed Speed to set the forward/backward speed to
				// (Also prepares maxLateral for arc movement)
				void setForwardBackSpeed(float speed, float maxTolerance = 1) {
					forwardBackSpeed = speed;
					maxLateral = speed * controllerMax + maxTolerance;
				}

				/// @brief Gets the forward/backward speed
				/// @return Forward/backward speed
				float getForwardBackSpeed() {
					return forwardBackSpeed;
				}

				/// @brief Gets the max lateral movement
				/// @return Max lateral movement
				float getMaxLateral() {
					return maxLateral;
				}

				// lower arc speed is lower turning

				DriveControlSpeed(float turnSpeed = 1, float forwardBackSpeed = 1, float arcSpeed = 0.7) :
					turnSpeed(turnSpeed), 
					arcSpeed(arcSpeed) {
						setForwardBackSpeed(forwardBackSpeed);
				}
		};


		/// @brief Ports for the drivetrain
		/// @param leftPorts Vector of ports for left motors
		/// @param rightPorts Vector of ports for right motors
		struct DrivetrainPorts {
			vector<std::int8_t> left;
			vector<std::int8_t> right;
			std::int8_t imuPort;
			char odomPorts[2];
			std::int8_t gpsPort;
		};

		/// @brief Args for drivetrain object
		/// @param abstractComponentArgs Args for AbstractComponent object
		struct DrivetrainArgs {
			AbstractComponentArgs abstractComponentArgs;
			DrivetrainPorts ports;
		};

		using ArgsType = DrivetrainArgs;

		/// @brief Struct for PID options (self-explanatory - timeLimit in MS)
		struct PIDOptions {
			double kP;
			double kI;
			double kD;
			double errorThreshold;
			float timeLimit;
		};

		DriveControlSpeed driveControlSpeed = {};

		bool preventBackMove = false;

		std::int32_t defaultMoveVelocity = 1024;
		std::int8_t maxRelativeError = 5;

		std::int16_t maxTurnVelocity = 60;
		float minTurnThreshold = 5;

		float relativeMovementCoefficient = 14.2857;
		float voltMovementCoefficient = 1;

		float maxVoltage = 12000;

		// Motor builtin encoder inchesPerTick
		double inchesPerTick = 0.034034;
		//double inchesPerTick = 1.0;

		// Odom wheel encoder inchesPerTick
		// TODO: Calculate this
		double odomIPT = 0.0;

		uint32_t moveDelayMs = 2;

		// right is positive
		// left is negative
		int pidInvertTurn = 1;
		float pidReductionFactor = 1.9;

		float arcDeadband = 30;

		/// @brief Sets the brake mode for each motor group
		/// @param mode Brake mode to set the motors toS
		void setBrakeModes(pros::motor_brake_mode_e_t mode) {
			left_mg.set_brake_mode_all(mode);
			right_mg.set_brake_mode_all(mode);
		}

		Drivetrain(DrivetrainArgs args) : 
			AbstractComponent(args.abstractComponentArgs),
			left_mg(args.ports.left),
			right_mg(args.ports.right),
			imu(args.ports.imuPort),
			odomEnc(args.ports.odomPorts[0], args.ports.odomPorts[1]),
			gps(args.ports.gpsPort, 0.4, 0, 0.5, 1.3, 270),
			leftMgSize(args.ports.left.size()),
			rightMgSize(args.ports.right.size()) {
				setDriveControlMode();
				calibrateIMU();
			};
	private:
		void prepareArcadeLateral(float& lateral) {
			// Change to negative to invert
			lateral *= -1;

			// Clamp the range to above 0 only to remove back movement
			if (preventBackMove && (lateral < 0)) {
				lateral = 0;
			}
		}

		// Calculate the movement of the robot when turning and moving laterally at the same time
		void calculateArcMovement(TurnCoefficients& turnCoeffs, float lateral, float turn, float maxLateralTolerance = 1) {
			if (std::fabs(lateral) < arcDeadband) {
				return;
			}

			// 0-1 range of percentage of lateral movement against max possible lateral movement
			float lateralCompensation = lateral / driveControlSpeed.getMaxLateral();
			// Decrease the turn speed when moving laterally (higher turn should be higher turnDecrease)
			float dynamicArcSpeed = (lateral < 0) ? driveControlSpeed.arcSpeed : 1;

			float turnDecrease = 1 * turn * lateralCompensation * dynamicArcSpeed;

			if (lateral > 0) {
				turnDecrease *= turn * 0.0001;
			}

			if (turn > 0) { // Turning to right so we decrease the left MG
				if (lateral < 0) {
					turnCoeffs.left -= turnDecrease;
				} else {
					turnCoeffs.left += turnDecrease;
				}
			} else { // Turning to left so we decrease the right MG
				if (lateral > 0) {
					turnCoeffs.right -= turnDecrease;
				} else {
					turnCoeffs.right += turnDecrease;
				}
			}

			pros::lcd::print(6, ("TD, dAS:, lComp: " + std::to_string(turnDecrease) + ", " + std::to_string(dynamicArcSpeed) + ", " + std::to_string(lateralCompensation)).c_str());
		}

		TurnCoefficients calculateArcadeTurns(float turn, float lateral) {
			turn *= 1;

			TurnCoefficients turnCoeffs = {turn, turn};

			// Allow for arc movement
			calculateArcMovement(turnCoeffs, lateral, turn);

			return turnCoeffs;
		}
	public:
		void opControl() override {
			driveControl();
		}

		/// @brief Arcade control for drive control (recommended to use opControl instead)
		void arcadeControl() {
			float lateral = master->get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
			float turn = master->get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick

			prepareArcadeLateral(lateral);

			TurnCoefficients turnCoeffs = calculateArcadeTurns(turn, lateral);
			
			pros::lcd::print(1, ("T, L:" + std::to_string(turn) + ", " + std::to_string(lateral)).c_str());

			// Calculate speeds
			lateral *= driveControlSpeed.getForwardBackSpeed();

			turnCoeffs.left *= driveControlSpeed.turnSpeed;
			turnCoeffs.right *= driveControlSpeed.turnSpeed;

			// Ensure voltages are within correct ranges
			std::int32_t left_voltage = prepareMoveVoltage(lateral - turnCoeffs.left);
			std::int32_t right_voltage = prepareMoveVoltage(lateral + turnCoeffs.right);

			pros::lcd::print(2, ("L/R COEF: " + std::to_string(turnCoeffs.left) + ", " + std::to_string(turnCoeffs.right)).c_str());
			pros::lcd::print(7, ("LEFT/RIGHT: " + std::to_string(left_voltage) + ", " + std::to_string(right_voltage)).c_str());

			left_mg.move(left_voltage);
			right_mg.move(right_voltage);
		}

		/// @brief Fallback control that DriveControlMode switch statement defaults to.
		void fallbackControl() {
			arcadeControl();
		}

		/// @brief Calibrates the IMU
		void calibrateIMU(bool blocking = true) {
			imu.reset(blocking);
			imu.tare();
		}

		/// @brief Sets the driver control mode
		/// @param mode Mode to set the driver control to
		void setDriveControlMode(DriveControlMode mode = DriveControlMode::ARCADE) {
			driveControlMode = mode;

			switch (driveControlMode) {
				case DriveControlMode::ARCADE:
					bindDriveControl(&Drivetrain::arcadeControl);
					break;
				default:
					bindDriveControl(&Drivetrain::fallbackControl);
					break;
			}
		}

		/// @brief Moves the motors at a specific voltage
		/// @param leftVoltage Voltage for left motor
		/// @param rightVoltage Voltage for right motor
		void moveVoltage(std::int16_t leftVoltage, std::int16_t rightVoltage) {
			left_mg.move_voltage(leftVoltage);
			right_mg.move_voltage(rightVoltage);
			pros::lcd::print(0, ("Left Voltage: " + std::to_string(leftVoltage)).c_str());
			pros::lcd::print(1, ("Right Voltage: " + std::to_string(rightVoltage)).c_str());
		}

		/// @brief Moves the motors at a single voltage
		/// @param voltage Voltage to move the motors at
		void moveSingleVoltage(std::int16_t voltage) {
			moveVoltage(voltage, voltage);
		}

		/// @brief Sets movement velocity
		/// @param leftVoltage Voltage for left motor
		/// @param rightVoltage Voltage for right motor
		void moveVelocity(std::int32_t leftVoltage, std::int32_t rightVoltage) {
			left_mg.move_velocity(leftVoltage);
			right_mg.move_velocity(rightVoltage);
		}

		/// @brief Moves the motors at a single velocity
		/// @param voltage Voltage to move the motors at
		void moveSingleVelocity(std::int32_t voltage) {
			moveVelocity(voltage, voltage);
		}

		/// @brief Stops moving the motors
		void moveStop() {
			moveSingleVelocity(0);
		}

		/// @brief Tares the motors
		void tareMotors() {
			left_mg.tare_position();
			right_mg.tare_position();
		}

		/// @brief Move to relative position
		/// @param pos Position to move to in CM
		void moveRelPos(double pos) {
			tareMotors();

			pos *= relativeMovementCoefficient;

			left_mg.move_relative(pos, defaultMoveVelocity);
			right_mg.move_relative(pos, defaultMoveVelocity);

			double lowerError = pos - maxRelativeError;

			while ((
				!(left_mg.get_position() > lowerError)
			) && (
				!(right_mg.get_position() > lowerError)
			)) {
				pros::delay(moveDelayMs);
			}

			moveStop();
		}

		/// @brief Turn to a specific angle
		/// @param angle Angle to turn to
		void turnTo(double angle) {
			imu.tare();

			double currentHeading = imu.get_heading();
			double angleDifference = normaliseAngle(angle - currentHeading);

			std::int16_t turnDirection = (angleDifference > 0) ? maxTurnVelocity : -maxTurnVelocity;
			//std::int16_t turnDirection = maxTurnVelocity;
			
			left_mg.move_velocity(turnDirection);
			right_mg.move_velocity(-turnDirection);

			while (true) {
				currentHeading = imu.get_heading();
				angleDifference = normaliseAngle(angle - currentHeading);
				
				if (std::fabs(angleDifference) <= minTurnThreshold) {
					break;
				}
				
				pros::lcd::set_text(2, "Current heading: " + std::to_string(currentHeading));
				pros::delay(moveDelayMs);
			}

			moveStop();
		}

		/// @brief Turn to a specific angle with a delay
		/// @param angle Angle to turn to
		/// @param delayMs Delay in milliseconds
		void turnDelay(bool direction, std::uint32_t delayMs, std::int16_t delayTurnVelocity = 60) {
			std::int16_t turnDirection = (direction) ? delayTurnVelocity : -delayTurnVelocity;

			left_mg.move_velocity(turnDirection);
			right_mg.move_velocity(-turnDirection);

			pros::delay(delayMs);

			moveStop();
		}

		/// @brief Move forward for a certain number of milliseconds
		/// @param delayMs Number of milliseconds to move forward
		/// @param left Whether to move the left motor
		/// @param right Whether to move the right motor
		void moveDelay(std::uint32_t delayMs, bool forward = true, std::int16_t delayMoveVelocity = 60) {
			if (forward) {
				moveSingleVelocity(-delayMoveVelocity);
			} else {
				moveSingleVelocity(delayMoveVelocity);
			}

			pros::delay(delayMs);
			moveStop();
		}

		double getHeading() {
			return imu.get_heading();
		}

		double getAvgMotorPos() {
			/*vector<double> leftPos = left_mg.get_position_all();
			vector<double> rightPos = right_mg.get_position_all();

			double avgLeft = calcMeanFromVector(leftPos, leftMgSize);
			double avgRight = calcMeanFromVector(rightPos, rightMgSize);*/

			double avgPos = (left_mg.get_position() + right_mg.get_position()) / 2;

			return avgPos;
		}

		// TODO: Generic PID function that we can apply to PIDTurn and PIDMove
		// maybe make a class for this? if it gets too complicated
		// but that would also require refactoring Drivetrain to have an AbstractDrivetrain
		// parent to avoid cyclic dependencies

		// WARNING: do NOT use relativeMovementCoefficient for PID functions
		// as this does not account for acceleration/deceleration
		// it's only for simple movement (phased out by PID & PIDOptions struct)

		/// @brief Turn to a specific angle using PID
		/// @param angle Angle to move to (PASS IN THE RANGE OF -180 TO 180 for left and right)
		// TODO: Tuning required
		void PIDTurn(double angle, float reductionFactor = 2, PIDOptions options = {
			0.3, 0.0, 0.7, 1, 6000
		}) {
			imu.tare();
			angle = naiveNormaliseAngle(angle);

			angle *= pidInvertTurn;

			angle /= -1;

			bool anglePositive = angle > 0;
			bool turn180 = false;

			// IMU already tared so we don't need to get the current heading
			float error = angle;
			float lastError = 0;
			float derivative = 0;
			float integral = 0;

			float out = 0;
			float trueHeading = 0;

			float maxThreshold = 180 - options.errorThreshold;

			float maxCycles = options.timeLimit / moveDelayMs;
			float cycles = 0;

			if (std::fabs(angle) >= 180) {
				turn180 = true;
			}

			pros::lcd::print(3, "PIDTurn Start");

			// with turning you just wanna move the other MG at negative of the MG of the direction
			// which u wanna turn to

			while (true) {
				trueHeading = std::fmod((imu.get_heading() + 180), 360) - 180;
				error = angle - trueHeading;

				integral += error;
				// Anti windup
				if (std::fabs(error) < options.errorThreshold) {
					integral = 0;
				}

				derivative = error - lastError;
				out = (options.kP * error) + (options.kI * integral) + (options.kD * derivative);
				lastError = error;

				out *= 1000; // convert to mV
				out = std::clamp(out, -maxVoltage, maxVoltage);
				out /= pidReductionFactor;
				moveVoltage(-out, out);

				pros::lcd::print(5, ("PIDTurn Out: " + std::to_string(out)).c_str());
				pros::lcd::print(7, ("PIDTurn Error: " + std::to_string(error)).c_str());
				pros::lcd::print(6, ("PIDTurn True Heading: " + std::to_string(imu.get_heading())).c_str());

				if (std::fabs(error) <= options.errorThreshold) {
					break;
				}

				// 180 degree turning
				if (std::fabs(trueHeading) >= maxThreshold) {
					break;
				}

				// TODO: refactor checks in prod
				if (std::fabs(out) < 100) {
					pros::lcd::print(4, "PIDTurn Out too low");
				}

				if (cycles >= maxCycles) {
					pros::lcd::print(4, "PIDTurn Time limit reached");
					break;
				}

				pros::delay(moveDelayMs);
				cycles++;
			}

			pros::lcd::print(2, "PIDTurn End");
			moveStop();
		}

		// think about arc motion, odometry, etc.
		// the key thing is PID.
		// TUNING REQUIRED!!!

		/// @brief Move to a specific position using PID
		/// @param pos Position to move to in inches (use negative for backward)
		// TODO: Tuning required
		void PIDMove(double pos, float reductionFactor = 2, PIDOptions options = {
			0.19, 0.0, 0.4, 3, 6000
		}) {
			// TODO: Consider adding odometry wheels as the current motor encoders
			// can be unreliable for long distances or just dont tare the motors
			tareMotors();

			pos /= inchesPerTick;
			pos *= -1;

			float error = pos;
			float motorPos = 0;
			float lastError = 0;
			float derivative = 0;
			float integral = 0;
			float out = 0;

			float maxCycles = options.timeLimit / moveDelayMs;
			float cycles = 0;

			// with moving you just wanna move both MGsat the same speed

			while (true) {
				// get avg error
				motorPos = right_mg.get_position();
				error = pos - motorPos;

				integral += error;
				// Anti windup
				if (std::fabs(error) < options.errorThreshold) {
					integral = 0;
				}

				derivative = error - lastError;
				out = (options.kP * error) + (options.kI * integral) + (options.kD * derivative);
				lastError = error;

				out *= 1000; // convert to mV
				out = std::clamp(out, -maxVoltage, maxVoltage);
				out /= pidReductionFactor;
				moveSingleVoltage(out);

				if (std::fabs(error) <= options.errorThreshold) {
					break;
				}

				pros::lcd::print(4, ("PIDMove Motor Pos: " + std::to_string(motorPos)).c_str());
				pros::lcd::print(5, ("PIDMove Out: " + std::to_string(out)).c_str());
				pros::lcd::print(7, ("PIDMove Error: " + std::to_string(error)).c_str());

				if (cycles >= maxCycles) {
					pros::lcd::print(4, "PIDMove Time limit reached");
					break;
				}

				pros::delay(moveDelayMs);
				cycles++;
			}

			moveStop();
		}

		/// @brief Gets the left motor group
		pros::MotorGroup& getLeftMotorGroup() {
			return left_mg;
		}

		/// @brief Gets the right motor group
		pros::MotorGroup& getRightMotorGroup() {
			return right_mg;
		}		
	}; // class Drivetrain

	class MogoMech : public AbstractMech {
	public:
		/// @brief Struct for telling the driver info
		/// @param on Whether to tell the driver
		/// @param line Line to tell the driver on
		/// @param prefixMsg Prefix message to tell the driver
		/// @param engagedMsg Message to tell the driver when engaged
		/// @param disengagedMsg Message to tell the driver when disengaged
		struct TellDriverInfo {
			bool on = true;
			std::uint8_t line = 0;

			string prefixMsg = "Mogo engaged: ";
			string engagedMsg = "YES";
			string disengagedMsg = "NO";
		};

		TellDriverInfo tellDriverInfo = {};
	private:
		bool lastPressed = false;

		void processPress() {
			if (!lastPressed) {
				//pros::lcd::set_text(1, "A ENGAGED NOT PRESSED");
				bool engaged = getEngaged();

				if (engaged) {
					actuate(false);
				} else {
					actuate(true);
				}

				if (tellDriverInfo.on) {
					engaged = getEngaged();
					string suffix = (engaged) ? tellDriverInfo.engagedMsg : tellDriverInfo.disengagedMsg;

					tell(tellDriverInfo.line, tellDriverInfo.prefixMsg + suffix);
				}
			}
		}
	protected:
	public:
		pros::controller_digital_e_t btn = pros::E_CONTROLLER_DIGITAL_A;

		/// @brief Args for mogo mech object
		/// @param abstractMechArgs Args for AbstractMech object
		struct MogoMechArgs {
			AbstractMechArgs abstractMechArgs;
		};

		using ArgsType = MogoMechArgs;

		/// @brief Creates mogo mech object
		/// @param args Args for MogoMech object (check args struct for more info)
		MogoMech(MogoMechArgs args) : 
			AbstractMech(args.abstractMechArgs) {
				actuate(true);
			};

		/// @brief Runs every loop to check if the button has been pressed
		void opControl () override {
			// Perform the actuation if this is the button has JUST been pressed
			if (master->get_digital(btn)) {
				processPress();
				lastPressed = true;
				//pros::lcd::set_text(1, "L1 pressed");
			} else {
				lastPressed = false;
			}
		}

		void postAuton() override {
			//actuate(true);
		}

		void skillsPrep() override {
			//actuate(true);
		}
	}; // class MogoMech

	class Conveyer : public AbstractMG {
	private:
	protected:
	public:
		BiToggle toggle;

		bool allowController = true;
		// 0-1 range of percentage of maximum voltage allowed to be supplied to the conveyer
		// half motor
		std::int32_t halfMotorReduction = 0.75;

		/// @brief Args for conveyer object
		/// @param abstractMGArgs Args for AbstractMG object
		/// @param conveyerPorts Vector of ports for conveyer motors
		/// @param mogoMech Pointer to mogo mech object
		struct ConveyerArgs {
			AbstractMGArgs abstractMGArgs;
		};

		using ArgsType = ConveyerArgs;

		Conveyer(ConveyerArgs args) :
			AbstractMG(args.abstractMGArgs), 
			toggle({this, {
				pros::E_CONTROLLER_DIGITAL_R2,
				pros::E_CONTROLLER_DIGITAL_R1
			}}) {
				speeds = {10000, -10000};
				outputSpeeds = true;

				std::int32_t halfMotorMillivoltLimit = MotorBounds::MILLIVOLT_MAX * halfMotorReduction;
				mg.set_voltage_limit(halfMotorMillivoltLimit, 0);
				mg.set_gearing_all(pros::motor_gearset_e_t::E_MOTOR_GEAR_RED);
			};

		bool canMove(bool on) override {
			/* Dont need this for now because lift mech doesn't exist
			bool mogoMechMoving = reqPointers.mogoMech->getEngaged();
			bool liftMechMoving = reqPointers.liftMech->getEngaged();

			bool moveConveyer = (mogoMechMoving && on) || (liftMechMoving && on);

			// DISABLE THIS FOR NOW BECAUSE WE DONT HAVE A LIFT MECH
			return moveConveyer;*/

			return on;
		}

		void opControl() override {
			if (allowController) {
				toggle.opControl();
			}
		}

		void postAuton() override {
			move(true);
		}

		void skillsPrep() override {
			move(true);
		}
	}; // class Conveyer

	class MogoStopper : public AbstractComponent {
	private:
		pros::Optical sensor;
		MogoMech* mogoMech;
		BtnManager btn;

		bool doActuate = false;			
	protected:
	public:
		pros::c::optical_rgb_s_t mogoRGB = {233, 255, 8};
		float tolerance = 5;
	private:
		bool channelWithinTolerance(const float& channel, const float& target) {
			return std::fabs(channel - target) <= tolerance;
		}

		bool rgbIsMogo(const pros::c::optical_rgb_s_t& rgb) {
			bool redWithinTolerance = channelWithinTolerance(rgb.red, mogoRGB.red);
			bool greenWithinTolerance = channelWithinTolerance(rgb.green, mogoRGB.green);
			bool blueWithinTolerance = channelWithinTolerance(rgb.blue, mogoRGB.blue);

			bool withinTolerance = redWithinTolerance && greenWithinTolerance && blueWithinTolerance;

			return withinTolerance;
		}
	public:

		/// @brief Args for mogo stopper object
		/// @param abstractComponentArgs Args for AbstractComponent object
		/// @param stopperPort Port for stopper motor
		struct MogoStopperArgs {
			AbstractComponentArgs abstractComponentArgs;
			std::int8_t sensorPort;
			MogoMech* mogoMech;
		};

		using ArgsType = MogoStopperArgs;

		void toggleActuate() {
			doActuate = !doActuate;
		}

		/// @brief Constructor for mogo stopper object
		/// @param args Args for mogo stopper object (see args struct for more info)
		MogoStopper(MogoStopperArgs args) : 
			AbstractComponent(args.abstractComponentArgs),
			sensor(args.sensorPort),
			mogoMech(args.mogoMech),
			btn({args.abstractComponentArgs, {
				pros::E_CONTROLLER_DIGITAL_B, {std::bind(&MogoStopper::toggleActuate, this)}, {}, {}
			}}) {};
		
		///#DFFF08 - Mobile goal hex code
		///R-233 G-255 B-8 Mobile goal RGB

		void opControl() override {
			if (!doActuate) {
				return;
			}

			pros::c::optical_rgb_s_t rgb = sensor.get_rgb();

			if (rgbIsMogo(rgb)) {
				mogoMech->actuate(true);
			}
		}
	}; // class MogoStopper

	/// @brief Class which manages the Lady Brown mechanism
	class LadyBrown : public AbstractMG {
	private:
		int currentTarget = 0;
	protected:
	public:
		/// @brief Args for Lady Brown object
		/// @param abstractMGArgs Args for AbstractMG object
		struct LadyBrownArgs {
			AbstractMGArgs abstractMGArgs;
			std::int8_t sensorPort;
		};

		using ArgsType = LadyBrownArgs;

		// Target position to move to (start, halfway, 3/4, end)
		vector<double> targets = {0, 90, 180, -1};

		pros::Rotation sensor;

		BtnManager upBtn;
		BtnManager resetBtn;

		bool atManualControl = false;
		int resetTarget = 2;
		int halfTarget = 1;
		double limit = 32000;

		Buttons manualBtns = {
			pros::E_CONTROLLER_DIGITAL_UP,
			pros::E_CONTROLLER_DIGITAL_DOWN
		};

		/// @brief Changes the target by the amount specified by the change parameter
		/// @param change Amount to change the target by
		void changeTarget(int change) {
			int limit = targets.size() - 1;
			currentTarget += change;
			currentTarget = std::clamp(currentTarget, 0, limit);
		}

		void incrementTarget(int increment = 2) {
			changeTarget(increment);
		}

		void decrementTarget(int increment = 2) {
			changeTarget(-increment);
		}

		void setTargetHalf() {
			atManualControl = false;
			currentTarget = halfTarget;
		}
	private:
		void manualControl() {
			std::int32_t sensorPos = sensor.get_position();
			bool belowLimit = sensor.get_position() < limit;
			//bool belowLimit = true;

			if (master->get_digital(manualBtns.fwd) && belowLimit) {
				move(true);
			} else if (master->get_digital(manualBtns.back)) {
				move(true, false);
			} else {
				move(false);
			}

			// debug thingy telling us the sensor pos for limit
			/*tell(0, "LB Pos: " + std::to_string(sensorPos));
			log(7, "LB Pos: " + std::to_string(sensorPos));*/
		}

		void upBtnControl() {
			incrementTarget();

			if (targets[currentTarget] == -1) {
				atManualControl = true;
			}
		}
	public:
		void resetPos() {
			atManualControl = false;
			currentTarget = resetTarget;
		}
	
		/// @brief Constructor for Lady Brown object
		/// @param args Args for Lady Brown object (see args struct for more info)
		LadyBrown(LadyBrownArgs args) : 
			AbstractMG(args.abstractMGArgs),
			upBtn({args.abstractMGArgs.abstractComponentArgs, {
				pros::E_CONTROLLER_DIGITAL_UP, {std::bind(&LadyBrown::upBtnControl, this)}, {}, {} 
			}}),
			resetBtn({
				args.abstractMGArgs.abstractComponentArgs, {
					pros::E_CONTROLLER_DIGITAL_L1, {std::bind(&LadyBrown::resetPos, this)}, {}, {}
			}}),
			sensor(args.sensorPort) {
				mg.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
				sensor.reset();
			};

		bool canMove(bool on) override {
			return on;
		}

		void opControl() override {
			upBtn.opControl();
			resetBtn.opControl();

			// Auto control if not manual controol
			if (atManualControl) {
				manualControl();
			} else {
				mg.move_absolute(targets[currentTarget], speeds.fwd);
			}

			// TODO: implement moving the lady brown to targets with rotation sensor instead of motor ticks
			// for more accuracy
		}
	}; // class LadyBrown

	/// @brief Torus sensor to automatically reject a red/blue torus when detected by optical sensor
	class TorusSensor : public AbstractComponent {
	public:
		std::uint8_t requiredTicks = 50;
	protected:
	public:
		pros::Optical sensor;
		Conveyer* conveyer;

		bool rejectRed;
		bool doReject;

		/// @brief Args for torus sensor object
		/// @param abstractComponentArgs Args for AbstractComponent object
		/// @param sensorPort Port for sensor
		/// @param conveyer Pointer to conveyer object
		/// @param rejectRed Whether to reject red torus
		/// @param doReject Whether to reject torus
		struct TorusSensorArgs {
			AbstractComponentArgs abstractComponentArgs;
			std::uint8_t sensorPort;
			Conveyer* conveyer;
			bool rejectRed;
			bool doReject;
		};

		using ArgsType = TorusSensorArgs;
	
		struct TorusHues {
			static constexpr float BLUE[2] = {150, 170};
			static constexpr float RED[2] = {10, 30};
		};

		static constexpr float MAX_LED_PWM = 100;

		/// @brief Constructor for torus sensor object
		/// @param args Args for torus sensor object (see args struct for more info)
		TorusSensor(TorusSensorArgs args) : 
			AbstractComponent(args.abstractComponentArgs),
			sensor(args.sensorPort),
			rejectRed(args.rejectRed),
			conveyer(args.conveyer),
			doReject(args.doReject) {
				sensor.set_led_pwm(MAX_LED_PWM);
			};
	private:
		/*
		process of torus sensor:
		1. (on inital colour sensed) LB stg1
		2. LB stg0
		*/

		bool detected = false;
		bool lastDetected = detected;
		
		bool triggered = false;
		std::int8_t ticks = 0;
		// each tick is equal to MAINLOOP_DELAY_TIME_MS (should be 20)

		void triggerControl() {
			requiredTicks++;

			if (ticks >= requiredTicks) {
				ticks = 0;
				triggered = false;
				conveyer->move(true);
			}
		}

		void checkTrigger() {
			if (detected && !lastDetected) { // Run this on 0->1 transition
				//tell(0, "at stg0");
				triggered = true;
				conveyer->move(true, false);
			} 
			
			//if (tick) {tell(0, "AT color");}
			if (triggered) { triggerControl(); }
		}
	public:
		void opControl() override {
			float hue = sensor.get_hue();

			bool atRed = isNumBetween(hue, TorusHues::RED[0], TorusHues::RED[1]);
			bool atBlue = isNumBetween(hue, TorusHues::BLUE[0], TorusHues::BLUE[1]);
			//bool atBlue = false;

			detected = (rejectRed) ? atRed : atBlue;

			//tell(0, "red/blue: " + std::to_string(atRed) + ", " + std::to_string(atBlue));

			if (doReject) { checkTrigger(); }

			//tell(0, "tick: " + std::to_string(tick) + ", " + std::to_string(hue));

			// don't run anything past this point
			lastDetected = detected;
		}
	}; // class TorusSensor

	class Doinker : public AbstractMech {
	private:
	protected:
	public:
		/// @brief Args for doinker object
		/// @param abstractMechArgs Args for AbstractMech object
		struct DoinkerArgs {
			AbstractMechArgs abstractMechArgs;
		};

		using ArgsType = DoinkerArgs;

		BtnManager actuateBtn;

		void handleBtn() {
			actuate(!getEngaged());
			//master->set_text(1, 0, "Doinker state: " + std::to_string(state));
		}

		/// @brief Constructor for doinker object
		/// @param args Args for doinker object (see args struct for more info)
		Doinker(DoinkerArgs args) : 
			AbstractMech(args.abstractMechArgs),
			actuateBtn({args.abstractMechArgs.abstractComponentArgs, {
				pros::E_CONTROLLER_DIGITAL_X, {std::bind(&Doinker::handleBtn, this)}, {}, {}
			}
			}) {};

		void opControl() override {
			actuateBtn.opControl();
		}
	};

	/// @brief Class for timer object to control timings
	class Timer : public AbstractComponent {
		private:
			int waitTime = 20;

			void wait() {
				pros::delay(waitTime);
			}
		protected:
		public:
			/// @brief Args for timer object
			/// @param abstractComponentArgs Args for AbstractComponent object
			struct TimerArgs {
				AbstractComponentArgs abstractComponentArgs;
			};

			using ArgsType = TimerArgs;

			/// @brief Constructor for timer object
			/// @param args Args for timer object (see args struct for more info)
			Timer(TimerArgs args) : 
				AbstractComponent(args.abstractComponentArgs) {};

			/// @brief Gets the wait time for the timer
			/// @return Time to wait for in milliseconds
			int getWaitTime() {
				return waitTime;
			}

			void opControl() override {
				wait();
			}
	}; // class Timer

	/// @brief Class for hanging mechanism
	class HangingMech : public AbstractMech {
	private:
		BtnManager actuateBtn;
	protected:
	public:
		void handleBtn() {
			actuate(!getEngaged());
		}

		/// @brief Args for hanging mechanism object
		/// @param abstractMechArgs Args for AbstractMech object
		struct HangingMechArgs {
			AbstractMechArgs abstractMechArgs;
		};

		using ArgsType = HangingMechArgs;

		/// @brief Constructor for hanging mechanism object
		/// @param args Args for hanging mechanism object (see args struct for more info)
		HangingMech(HangingMechArgs args) : 
			AbstractMech(args.abstractMechArgs),
			actuateBtn({args.abstractMechArgs.abstractComponentArgs, {
				pros::E_CONTROLLER_DIGITAL_B, {std::bind(&HangingMech::handleBtn, this)}, {}, {}
			}}) {};

		void opControl() override {
			actuateBtn.opControl();
		}
	}; // class HangingMech

	/// @brief Class which manages all components
	class ComponentManager : public AbstractComponent {
	private:
	protected:
	public:
		Drivetrain dvt;

		MogoMech mogoMech;
		Doinker doinker;
		HangingMech hang;

		Conveyer conveyer;
		LadyBrown ladyBrown;

		MogoStopper mogoStopper;

		TorusSensor torusSensor;

		Timer timer;
		
		// All components are stored in this vector
		vector<AbstractComponent*> components;

		/// @brief Args for component manager object passed to the chassis, such as ports
		/// @param dvtPorts Ports for drivetrain
		/// @param mogoMechPort Port for mogo mech
		/// @param doinkerPort Port for doinker
		/// @param hangingPort Port for hanging mechanism
		/// @param conveyerPorts Ports for conveyer
		/// @param ladyBrownPorts Ports for lady brown
		/// @param ladyBrownRotSensorPort Port for lady brown rotation sensor
		/// @param mogoStopperPort Port for mogo stopper
		/// @param torusSensorPort Port for torus sensor
		/// @param rejectRedToruses Whether to reject red toruses
		struct ComponentManagerUserArgs {
			Drivetrain::DrivetrainPorts dvtPorts;
			char mogoMechPort;
			char doinkerPort;
			char hangingPort;
			MGPorts conveyerPorts;
			MGPorts ladyBrownPorts;
			std::int8_t ladyBrownRotSensorPort;
			std::int8_t mogoStopperPort;
			std::uint8_t torusSensorPort;
			bool rejectRedToruses;
			bool rejectToruses;
		};

		/// @brief Args for component manager object
		/// @param aca Args for AbstractComponent object
		/// @param user Args for component manager object passed to the chassis
		struct ComponentManagerArgs {
			AbstractComponentArgs aca;
			ComponentManagerUserArgs user;
		};

		/// @brief Constructor for component manager object
		/// @param args Args for component manager object (see args struct for more info)
		ComponentManager(ComponentManagerArgs args) : 
			AbstractComponent(args.aca),

			dvt({args.aca, args.user.dvtPorts}),
			mogoMech({args.aca, args.user.mogoMechPort}),
			conveyer({args.aca, args.user.conveyerPorts}),
			ladyBrown({{args.aca, args.user.ladyBrownPorts}, args.user.ladyBrownRotSensorPort}),
			doinker({args.aca, args.user.doinkerPort}),
			hang({args.aca, args.user.hangingPort}),
			mogoStopper({args.aca, args.user.mogoStopperPort, &mogoMech}),
			torusSensor({args.aca, args.user.torusSensorPort, &conveyer, args.user.rejectRedToruses, args.user.rejectToruses}),			
			timer({args.aca}) { 												// Add component pointers to vector
				// MUST BE DONE AFTER INITIALISATION not BEFORE because of pointer issues
				components = {
					&dvt,
					&mogoMech,
					&conveyer,
					&ladyBrown,
					&doinker,
					&mogoStopper,
					&torusSensor,
					&hang,
					&timer
				};
			};

		// Nice and simple :) definitely better than having to call each component individually
		void opControl() override {
			for (AbstractComponent* component : components) {
				component->opControl();
			}
		}

		void skillsPrep() override {
			for (AbstractComponent* component : components) {
				component->skillsPrep();
			}
		}

		void postAuton() override {
			for (AbstractComponent* component : components) {
				component->postAuton();
			}
		}
	}; // class ComponentManager

	/// @brief Abstract class for auton e.g. match or skills autonomous
	class AbstractAuton {
	private:
	protected:
		ComponentManager* cm;
	public:
		/// @brief Args for auton object
		/// @param cm Component manager object
		struct AutonArgs {
			ComponentManager* cm;
		};

		/// @brief Creates auton object
		/// @param args Args for auton object (check args struct for more info)
		AbstractAuton(AutonArgs args) : 
			cm(args.cm) {};

		/// @brief Runs the auton
		virtual void run() = 0;

		virtual ~AbstractAuton() = default;
	}; // class AbstractAuton

	class MatchAuton : public AbstractAuton {
	private:
		void linedAuton() {
			cm->dvt.PIDMove(30);
			cm->dvt.PIDTurn(-45);
			cm->dvt.moveDelay(500);
		}
		
		void calcCoefficientAuton()  {
			// 1 tile = 2 feet = 24 inches
			// 72 = 3 tiles = 3 feet
			// 96 = 4 tiles = 4 feet
			cm->dvt.PIDMove(72);
		}

		void testIMUAuton() {
			cm->dvt.moveVelocity(500, -500);
			pros::lcd::set_text(5, std::to_string(cm->dvt.getHeading()));
			pros::delay(10);
		}

		void calcTurnAuton() {
			cm->dvt.PIDTurn(-90);
		}

		void alliance_mogo_leftstart() {
			// Deposit preload on low wall stake
			// THIS IS THE LINE THAT CONTROLS HOW FAR FORWARD
			// TO GO TO THE WALL STAKE
			cm->dvt.PIDMove(17);
			//pros::lcd::print(2, "Initial phase complete");
			pros::delay(100);

			// Move to mogo
			cm->dvt.PIDTurn(90);
			cm->dvt.moveDelay(700, false);
			cm->conveyer.move(true);
			pros::delay(750);
			// stop it from hitting the wall
			cm->conveyer.move(false);
			cm->dvt.PIDMove(4);

			// Collect mogo

			cm->dvt.PIDTurn(-140);
			pros::delay(50);
			cm->mogoMech.actuate(true);
			cm->dvt.PIDMove(-47);
			cm->mogoMech.actuate(false);
			pros::delay(80);

			cm->dvt.PIDTurn(-110);
			pros::delay(100);
			cm->conveyer.move(true);
			cm->dvt.PIDMove(20);
			pros::delay(1000);
			cm->conveyer.move(false);
			// uncommnet later
			//cm->mogoMech.actuate(true);
			// Turn halfway through going to mogo
			// fix to turn 180 degrees
			//return;
			// for some reason 90 degrees has become 180 degrees for some reason
			cm->dvt.PIDTurn(-90);
			//dvt.PIDTurn(90);

			//return;

			
			//cm->dvt.PIDMove(-8);
			// uncommnet later

			// Collect mogo
			/*
			pros::delay(500);

			//return;
			// Turn, move and collect rings
			cm->dvt.PIDTurn(60);
			cm->conveyer.move(true);
			pros::delay(500);
			cm->mogoMech.actuate(true);
			// uncommnet later
			//cm->dvt.PIDMove(25);
			pros::delay(500);
			cm->conveyer.move(false);
			*/
			// Prepare for opcontrol
			//cm->conveyer.move(false);*/

			// OPTIONAL: Turn to face the wall
			/*
			cm->dvt.PIDTurn(150);
			cm->dvt.PIDMove(70);
			*/			
			
			pros::delay(2000);
		}
		void alliance_mogo_rightstart() {
			// Deposit preload on low wall stake
			// THIS IS THE LINE THAT CONTROLS HOW FAR FORWARD
			// TO GO TO THE WALL STAKE
			cm->dvt.PIDMove(15);
			//pros::lcd::print(2, "Initial phase complete");
			pros::delay(100);

			// Move to mogo
			cm->dvt.PIDTurn(-90);
			cm->dvt.moveDelay(700, false);
			cm->conveyer.move(true);
			pros::delay(750);
			// stop it from hitting the wall
			cm->conveyer.move(false);
			cm->dvt.PIDMove(4);

			// Collect mogo

			cm->dvt.PIDTurn(135);
			pros::delay(50);
			cm->mogoMech.actuate(true);
			cm->dvt.PIDMove(-47);
			cm->mogoMech.actuate(false);
			pros::delay(80);

			cm->dvt.PIDTurn(110);
			pros::delay(100);
			cm->conveyer.move(true);
			cm->dvt.PIDMove(20);
			pros::delay(1000);
			cm->conveyer.move(false);
			// uncommnet later
			//cm->mogoMech.actuate(true);
			// Turn halfway through going to mogo
			// fix to turn 180 degrees
			//return;
			// for some reason 90 degrees has become 180 degrees for some reason
			cm->dvt.PIDTurn(90);
			//dvt.PIDTurn(90);

			//return;

			
			//cm->dvt.PIDMove(-8);
			// uncommnet later

			// Collect mogo
			/*
			pros::delay(500);

			//return;
			// Turn, move and collect rings
			cm->dvt.PIDTurn(60);
			cm->conveyer.move(true);
			pros::delay(500);
			cm->mogoMech.actuate(true);
			// uncommnet later
			//cm->dvt.PIDMove(25);
			pros::delay(500);
			cm->conveyer.move(false);
			*/
			// Prepare for opcontrol
			//cm->conveyer.move(false);*/

			// OPTIONAL: Turn to face the wall
			/*
			cm->dvt.PIDTurn(150);
			cm->dvt.PIDMove(70);
			*/			
			
			pros::delay(2000);
		}
		void mogo_corner_left() {
			//goes to mogo and collects it
			cm->mogoMech.actuate(true);
			cm->dvt.PIDMove(-38);
			pros::delay(200);
			cm->mogoMech.actuate(false);
			pros::delay(200);
			//loads preload on mogo
			cm->conveyer.move(true);
			cm->dvt.PIDTurn(-90);
			//goes to left handside stack
			cm->dvt.PIDMove(22);
			pros::delay(1200);
			cm->doinker.actuate(true);
			cm->conveyer.move(true,false);
			cm->dvt.PIDTurn(-180);
			cm->dvt.PIDTurn(-100);
			//go to corner
			cm->dvt.PIDMove(40);
			cm->dvt.PIDTurn(180);
			pros::delay(200);
			//drop off mogo
			cm->dvt.PIDMove(-10);
			cm->mogoMech.actuate(true);
			cm->conveyer.move(true, false);

			cm->conveyer.move(false);
			cm->dvt.PIDMove(20);
		}
		void mogo_corner_right() {
			//goes to mogo and collects it
			cm->mogoMech.actuate(true);
			cm->dvt.PIDMove(-32);
			pros::delay(200);
			cm->mogoMech.actuate(false);
			pros::delay(200);
			//loads preload on mogo
			cm->conveyer.move(true);
			cm->dvt.PIDTurn(90);
			//goes to left handside stack
			cm->dvt.PIDMove(22);
			pros::delay(1200);
			cm->doinker.actuate(true);
			
			cm->dvt.PIDTurn(180);
			cm->dvt.PIDTurn(100);
			//go to corner
			cm->dvt.PIDMove(48);
			cm->dvt.PIDTurn(180);
			cm->dvt.PIDTurn(60);
			pros::delay(200);
			//drop off mogo
			cm->dvt.PIDMove(-10);
			cm->mogoMech.actuate(true);
			cm->conveyer.move(true, false);

			cm->conveyer.move(false);
			cm->dvt.PIDMove(20);
		}
	protected:
	public:
		/// @brief Args for match auton object
		/// @param autonArgs Args for auton object
		struct MatchAutonArgs {
			AutonArgs autonArgs;
		};

		/// @brief Creates match auton object
		/// @param args Args for match auton object (check args struct for more info)
		MatchAuton(MatchAutonArgs args) : 
			AbstractAuton(args.autonArgs) {};

		// TODO: Implement
		void run() override {
			// just comment out the auton function u dont want

			//calcCoefficientAuton();
			//calcTurnAuton();
			//testIMUAuton();
			//linedAuton();
			//alliance_mogo_rightstart();
			//alliance_mogo_leftstart();
			//mogo_corner_left();
			mogo_corner_right();
		}
	}; // class MatchAuton

	class SkillsAuton : public AbstractAuton {
	private:
		pros::adi::DigitalOut mogo;

		void sector1() {
			// horrible terrible but oh well
			// Preload onto wall stake and go forward to clamp mogo
			cm->dvt.setBrakeModes(pros::E_MOTOR_BRAKE_COAST);

			// wall stake
			cm->conveyer.move(true);
			pros::delay(500);
			
			cm->conveyer.move(false);
			// turn and get mogo
			cm->dvt.PIDMove(10);
			cm->dvt.PIDTurn(-90);
			mogo.set_value(true);
			
			cm->dvt.PIDMove(-28, 1.6);
			
			mogo.set_value(false);
			pros::delay(100);
							
			// Collect rings onto mogo
			cm->dvt.PIDTurn(-90);
			cm->dvt.PIDTurn(-90);
			cm->tell(0, "AFTER SECOND TURN");
			
			cm->conveyer.move(true);
			//cm->dvt.PIDTurn(3);
			cm->dvt.PIDMove(28);
			//pros::delay(100);
			cm->dvt.PIDMove(15);	
			
			
			
			cm->dvt.PIDMove(-5);
			
			cm->dvt.PIDTurn(110);

			
			cm->dvt.PIDMove(15);

			//pros::delay(100);
			
			cm->conveyer.move(true);
			cm->dvt.PIDTurn(-100);
			cm->conveyer.move(false);
			
			cm->dvt.PIDTurn(-110);
			
			
			cm->conveyer.move(true);
			cm->dvt.PIDMove(40);
			
			
			cm->dvt.PIDTurn(-30);
			
			
			cm->dvt.PIDMove(-45);
			
			cm->conveyer.move(true, false);

			//pros::delay(500);

			cm->conveyer.move(false);
			
			mogo.set_value(true);
			//pros::delay(200);
		}

		void sector2() {
			cm->dvt.PIDMove(10);

			// turn to the mogo on the other end
			// and go for it!
			cm->dvt.PIDTurn(80);
			cm->dvt.PIDMove(-117);
			//pros::delay(200);
			mogo.set_value(false);
			pros::delay(50);

			cm->dvt.PIDTurn(90);
			cm->tell(0, "AFTER FIRST TURN");
			cm->dvt.PIDTurn(80);
			cm->tell(0, "AFTER SECOND TURN");
			//pros::delay(200);

			cm->conveyer.move(true);
			//cm->dvt.PIDTurn(3);
			cm->dvt.PIDMove(30);
			//pros::delay(200);
			cm->dvt.PIDMove(-5);
			//pros::delay(200);
			cm->dvt.PIDTurn(-10);
			//pros::delay(200);
			//pros::delay(200);
			cm->dvt.PIDTurn(90);
			//pros::delay(200);
			cm->dvt.PIDTurn(60);
			//pros::delay(200);
			
			//pros::delay(200);
			cm->dvt.PIDMove(-15);
		
			
			cm->conveyer.move(true, false);

			//pros::delay(500);

			
			
			mogo.set_value(true);
			pros::delay(50);
			cm->dvt.PIDMove(20);
			cm->dvt.PIDTurn(-30);
			
			pros::delay(200);
		}

		void sector3() {
			mogo.set_value(true);
			cm->conveyer.move(true);
			cm->dvt.PIDMove(60);
			cm->doinker.actuate(true);
			cm->dvt.PIDTurn(60);
			cm->dvt.PIDMove(30);
			//pros::delay(200);
			mogo.set_value(true); //incase the mogo has not unactuated already
			cm->dvt.PIDTurn(90);
			cm->dvt.PIDTurn(55);
			cm->dvt.PIDMove(-25);
			mogo.set_value(false);

		}
		void sector4() {
			cm->doinker.actuate(false);
			cm->dvt.PIDTurn(-90);
			cm->dvt.PIDMove(-40);
			cm->conveyer.move(false);

			//pros::delay(500);

			cm->conveyer.move(false);
			
			mogo.set_value(true);

		}
		void sector5() {
			cm->dvt.PIDMove(110);
			cm->dvt.PIDTurn(-45);
			cm->dvt.PIDMove(30);
			cm->conveyer.move(true);
			cm->dvt.PIDMove(-10);
			cm->dvt.PIDMove(30);
			cm->dvt.PIDMove(-5);
			cm->doinker.actuate(true);
			cm->dvt.PIDTurn(720);
			cm->doinker.actuate(false);
			cm->dvt.PIDMove(35);
			cm->conveyer.move(false);
			cm->dvt.PIDMove(5);
			cm->dvt.PIDMove(-10);
		}
		
		
	protected:
	public:
		/// @brief Args for skills auton object
		/// @param autonArgs Args for auton object
		struct SkillsAutonArgs {
			AutonArgs autonArgs;
		};

		/// @brief Creates skills auton object
		/// @param args Args for skills auton object (check args struct for more info)
		SkillsAuton(SkillsAutonArgs args) : 
			AbstractAuton(args.autonArgs),
			// remove later because its horrible and sad and bad
			mogo(MOGO_MECH_PORT) {};

		void run() override {
			cm->tell(0, "Skills auton running");

			sector1();
			sector2();
			sector3();
			sector4();
			sector5();
		}
	}; // class SkillsAuton

	/// @brief Chassis class for controlling auton/driver control
	class Chassis : public AbstractChassis {
	private:
	protected:
	public:
		/// @brief Args for chassis object
		/// @param cmUserArgs Args for component manager object
		struct ChassisArgs {
			ComponentManager::ComponentManagerUserArgs cmUserArgs;
		};

		ComponentManager cm;

		MatchAuton matchAutonManager;
		SkillsAuton skillsAutonManager;

		/// @brief Creates chassis object
		/// @param args Args for chassis object (check args struct for more info)
		Chassis(ChassisArgs args) : 
			AbstractChassis(),
			cm({this, args.cmUserArgs}),
			matchAutonManager({&cm}),
			skillsAutonManager({&cm}) {};

		/// @brief Runs the opcontrol functions for each component
		void opControl() override {
			cm.opControl();
		}

		/// @brief Auton function for the chassis
		// 1000 = 70cm
		void auton() override {
			matchAutonManager.run();
		}

		/// @brief Skills auton function for the chassis
		void skillsAuton() override {
			skillsAutonManager.run();
		}

		/// @brief Skills preparation for opcontrol on the chassis
		void skillsPrep() override {
			// We need to run postAuton() first because these are what would prep for opcontrol normally
			cm.skillsPrep();
		}

		void postAuton() override {
			cm.postAuton();
		}
	}; // class Chassis

	/// @brief Convert vector of ints to string. For displaying on the LCD/debugging
	/// @param vec Vector to convert
	/// @param delimiter Delimiter to separate elements
	template <typename T>
	string vectorToString(vector<T>& vec, string delimiter) {
		int vecSize = vec.size();
		int vecSizeMinusOne = vecSize - 1;
		std::ostringstream oss;

		oss << "{";
		for (int i = 0; i < vecSize; i++) {
			oss << vec[i];
			if (i < vecSizeMinusOne) {
				oss << delimiter;
			}
		}
		oss << "}";

		return oss.str();
	}

	/// @brief Assert that a value is arithmetic
	/// @param val Value to assert
	template <typename T>
	void assertArithmetic(const T val) {
		static_assert(std::is_arithmetic<T>::value, "Value must be arithmetic");
	}

	/// @brief Checks whether a given color channel is within tolerance.
	/// @param channel Color to check
	/// @param target Target colour which the colour should be
	/// @return Whether the channel is within tolerance
	bool channelWithinTolerance(const float& channel, const float& target, const float& tolerance = 5) {
		return std::fabs(channel - target) <= tolerance;
	}

	std::int32_t prepareMoveVoltage(float raw) {
		// Round the number to the nearest integer
		raw = std::round(raw);

		std::int32_t voltage = static_cast<std::int32_t>(raw);
		voltage = std::clamp(voltage, MotorBounds::MOVE_MIN, MotorBounds::MOVE_MAX);

		return voltage;
	}

	/// @brief Assert that a number is between two values
	/// @param num Number to assert
	/// @param min Minimum value
	/// @param max Maximum value
	template <typename T>
	bool isNumBetween(T num, T min, T max) {
		assertArithmetic(num);

		return ((num >= min) && (num <= max));
	}

	/// @brief Normalise an angle to the range [-180, 180]
	/// @param angle Angle to normalise
	template <typename T>
	T normaliseAngle(T angle) {
		assertArithmetic(angle);

		if (angle > 180) {
			angle -= 360;
		} else if (angle < -180) {
			angle += 360;
		}

		return angle;
	}

	/// @brief Naively normalise an angle to the range [-180, 180] by simply clamping the value
	/// @param angle Angle to normalise
	template <typename T>
	T naiveNormaliseAngle(T angle) {
		assertArithmetic(angle);

		angle = std::clamp(angle, -180.0, 180.0);

		return angle;
	}

	/// @brief Calculate the mean of a vector
	/// @param vec Vector to calculate the mean of
	/// @param size Size of the vector
	/// @return Mean of the vector (type T)
	template <typename T>
	T calcMeanFromVector(const vector<T>& vec, int size) {
		T sum = std::accumulate(vec.begin(), vec.end(), 0);
		T mean = sum / size;

		return mean;
	}

	/// @brief Calculate the mean of a vector
	/// @param vec Vector to calculate the mean of
	/// @return Mean of the vector (type T)
	template <typename T>
	T calcMeanFromVector(const vector<T>& vec) {
		int size = vec.size();
		T sum = std::accumulate(vec.begin(), vec.end(), 0);
		T mean = sum / size;

		return mean;
	}

	/*/// @brief Get all the values of an enum class into a vector
	template <typename T>
	vector<T> getAllValues() {
		vector<T> values;
		constexpr int max = static_cast<int>(T::_MAX);
		values.reserve(max);

		for (int i = 0; i < max; i++) {
			values.push_back(static_cast<T>(i));
		}

		return values;
	}

	/// @brief Fill a map with default values for an enum class (see below function def for example use case)
	/// @param map Map to fill
	template <typename E, typename V>
	void fillMapWithEnum(map<E, V>& map) {
		vector<E> values = getAllValues<E>();
		E defaultValue = V();

		for (E value : values) {
			map[value] = defaultValue;
		}
	}*/
	// example use case
	//fillMapWithEnum<pros::controller_digital_e_t, bool>(map);
} // namespace hyper

// Global variables

// DONT say just "chassis" because certain class properties have the same name
hyper::AbstractChassis* currentChassis;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void initDefaultChassis() {
	static hyper::Chassis defaultChassis({
		{{LEFT_DRIVE_PORTS, RIGHT_DRIVE_PORTS, IMU_PORT, ODOM_ENC_PORTS, GPS_SENSOR_PORT}, // Drivetrain args
		MOGO_MECH_PORT, DOINKER_PORT, HANGING_MECH_PORT, // Mech args
		CONVEYER_PORTS, LADY_BROWN_PORTS, // MG args
		LADY_BROWN_ROT_SENSOR_PORT, MOGO_SENSOR_PORT, // Sensor args
		CONV_TORUS_PORT, REJECT_COLOR_RED, DO_REJECT_COLOR}}); // Torus sensor args
	
	currentChassis = &defaultChassis;
}

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();

	INIT_CHASSIS();

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	if (DO_MATCH_AUTON) {
		currentChassis->auton();
	}
}

// Not used anymore, used to be for pneumatics testing
void pneumaticstestcontrol () {
	pros::lcd::set_text(0, "In testcontrol");
	pros::Controller controller(pros::E_CONTROLLER_MASTER);
	while (true) {

		pros::delay(20); // Add a small delay to prevent overwhelming the CPU
	}
}

void testMatchAuton() {
	if (MATCH_AUTON_TEST) {
		autonomous();
	}
}

void preControl() {
	pros::lcd::set_text(0, "> 1408Hyper mainControl ready");

	bool inComp = pros::competition::is_connected();

	// competition auton test safeguard
	if (!inComp) {
		testMatchAuton();
	}

	if (DO_SKILLS_AUTON) {
		currentChassis->skillsAuton();
	}

	if (DO_SKILLS_PREP) {
		currentChassis->skillsPrep();
	}

	// only do post auton if we are not in skills prep
	if (DO_POST_AUTON && !DO_SKILLS_PREP) {
		currentChassis->postAuton();
	}
}

void mainloopControl() {
	bool opControlRunning = DO_OP_CONTROL;
	// Chassis control loop
	while (opControlRunning) {
		// Chassis opcontrol
		currentChassis->opControl();
	}
}

void mainControl() {
	preControl();
	mainloopControl();
}
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
	CURRENT_OPCONTROL();
}

// hello copilot how are you doing
// i am doing well thank you for asking
// what do you think of my code
// i think it is very good
// is there anything that you would add to my code?
// i would add more comments
// what is your favourite programming language
// i like c++ the most

// anti quick make nothing comment thingy
// aaaaa
