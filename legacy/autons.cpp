// Legacy autons
// You can put these functions back in at any time and they should work just fine.

		void testGpsAuton() {
			//cm->dvt.PIDGps(0.6096);
			cm->dvt.BangGPS(0.2096);
			pros::delay(20000);
		}

		void aadiAuton() {
			cm->mogoMech.actuate(true);
			cm->dvt.moveSingleVelocity(1);
			cm->dvt.PIDMove(-29);
			pros::delay(500);
			cm->mogoMech.actuate(false);
			cm->conveyer.move(true);
			pros::delay(300);
			cm->dvt.PIDTurn(-57);
			cm->conveyer.move(true);
			cm->dvt.PIDMove(30);
		}

		void testMogoAuton() {
			pros::delay(2000);
			cm->mogoMech.actuate(true);
			cm->tell(0, "Mogo actuated");
			pros::delay(2000);
			cm->mogoMech.actuate(false);
			cm->tell(0, "Mogo disactuated");
		}

        		void defaultAuton() {
			// destruction 100

			/*//cm->dvt.moveRelPos(300);
			//
			cm->dvt.turnDelay(true, 600);
			cm->dvt.moveRelPos(100);
			cm->dvt.turnDelay(false, 400);
			//cm->dvt.moveRelPos(150);
			cm->dvt.turnDelay(true, 300);*/

			/*cm->intake.move(true, false);
			cm->conveyer.move(true);*/

			// Because auton is only 15 secs no need to divide into sectors
			// Move and collect first rings/discombobulate first
			//cm->intake.move(true);
			cm->dvt.turnDelay(true, 600);
			//pros::delay(MAINLOOP_DELAY_TIME_MS);
			cm->dvt.moveRelPos(50);

			// Get the far ring and turn back onto main path
			// (no longer necessarily needed because we start with 1 ring already in the robot)
			cm->dvt.turnDelay(true, 330);
			cm->dvt.moveRelPos(105);
			//pros::delay(MAINLOOP_DELAY_TIME_MS);
			//cm->dvt.turnDelay(false, 1.5);

			// Get other stack knocked over
			// optional: increase speed to cm->intake if we have no harvester
			//cm->dvt.moveRelPos(130);
			//cm->dvt.moveDelay(600, false);
			cm->dvt.turnDelay(false, 450);
			//pros::delay(MAINLOOP_DELAY_TIME_MS);
			cm->dvt.moveRelPos(160);
			
			// Turn into high wall stake & deposit
			cm->dvt.turnDelay(false, 870);
			cm->dvt.moveDelay(800, false);
			//cm->intake.move(false);
			//liftMech.actuate(true);
			//pros::delay(MAINLOOP_DELAY_TIME_MS);

			// Deposit on high wall stake
			//cm->conveyer.move(true, false);
			pros::delay(2000);
			//cm->conveyer.move(false);
			//liftMech.actuate(false);
		}
