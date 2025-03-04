// Legacy motor group classes
// You can put these classes back in at any time and they should work just fine.


	class Intake : public AbstractMG {
	private:
		BiToggle toggle;
	protected:
	public:
		/// @brief Args for intake object
		/// @param abstractMGArgs Args for AbstractMG object
		/// @param intakePorts Vector of ports for intake motors
		struct IntakeArgs {
			AbstractMGArgs abstractMGArgs;
			vector<std::int8_t> intakePorts;
		};

		using ArgsType = IntakeArgs;

		/// @brief Constructor for intake object
		/// @param args Args for intake object (see args struct for more info)
		Intake(IntakeArgs args) :
			AbstractMG(args.abstractMGArgs),
			toggle({this, {
				pros::E_CONTROLLER_DIGITAL_L1,
				pros::E_CONTROLLER_DIGITAL_L2
			}}) {}

		bool canMove(bool on) override {
			return on;
		}

		void opControl() override {
			toggle.opControl();
		}
	}; // class Intake
