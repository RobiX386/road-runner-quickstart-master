package org.firstinspires.ftc.teamcode;


public class RobotConstants {
	public enum ArmLevels {
		Preintake_position(0.15),
		Intake_position(0.04),
		Scoring_position(0.84),
		Standby_position(0.54);

		public double position = 0.1;

		ArmLevels(double position) { this.position = position; }
	}

    public enum LiftLevels {
        Auto_position(270),
        Down_position(0),
        First_position(320),
        Second_position(530),
        Third_position(650),
        Fourth_position(770),
        Fifth_position(890),
        Sixth_position(1010),
        Seventh_position(1130),
        Eight_position(1250);


        public int position = 0;

        LiftLevels(int value) {
            this.position = value;
        }
    }

    public static LiftLevels state;

}
