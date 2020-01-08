package frc.robot;

public class RobotMap
{
	// Robot Stat Values
	public static final double ROBOT_TIME_STEP = 0.02; // 20ms
	public static final double ROBOT_MAX_VELOCITY = 3.2624; // meters per second
	public static final double ROBOT_MAX_ACCELERATION = 0.2;
	public static final double ROBOT_MAX_JERK = 0.5;
	public static final int ROBOT_WHEEL_DIAMETER = 2;

	// Drivetrain ID Values
	public static final int DRIVETRAIN_LEFT1 = 1;
	public static final int DRIVETRAIN_LEFT2 = 2;
	public static final int DRIVETRAIN_LEFT3 = 3;
	public static final int DRIVETRAIN_RIGHT1 = 4;
	public static final int DRIVETRAIN_RIGHT2 = 5;
	public static final int DRIVETRAIN_RIGHT3 = 6;
	public static final double DRIVETRAIN_LEFT_PGAIN = .986;

	// BallMech ID Values
	public static final int BALLMECH_LEFTARM = 3;
	public static final int BALLMECH_RIGHTARM = 10;
	public static final int BALLMECH_INTAKE = 11;

	//climbMech ID Values
	public static final int CLIMBMECH_MOTOR_LEFT = 2;
	public static final int CLIMBMECH_MOTOR_RIGHT = 1;

	// TestMech Constants
	public static final int TESTMECH_TEST_MOTOR = 55;

	// Vision System Constants
	public static final double HATCH_VISION_TURN_PGAIN = -0.008;
	public static final double HATCH_VISION_DISTANCE_PGAIN = 0.3;
	public static final double HATCH_VISION_MIN_AIM_COMMAND = 0.0;
	public static final double HATCH_VISION_DESIRED_TARGET_AREA = 4.0;

	// Pneumatics ID Values
	public static final int PCM1 = 0;
	public static final int PCM2 = 14;
	public static final int DRIVETRAIN_SHIFTER_FWD = 5; //correct
	public static final int DRIVETRAIN_SHIFTER_REV = 2;
	public static final int HATCH_CLAMP_FWD = 4;
	public static final int HATCH_CLAMP_REV = 3;
	public static final int HATCH_PUSHER_FWD = 7; //correct
	public static final int HATCH_PUSHER_REV = 0;
	public static final int ARM_CLUTCH_REV = 1;
	public static final int ARM_CLUTCH_FWD = 6;
	public static final int HABLIFT_PISTON_FWD = 7;
	public static final int HABLIFT_PISTON_REV = 0;
	public static final int SMALL_HABLIFT_PISTON_FWD = 2;
	public static final int SMALL_HABLIFT_PISTON_REV = 5;
	

	// ENUMS
	public enum ArmPosition 
	{
		STARTING, LOADING_FLOOR, LOADING_WALL, SCORING, CLIMBING;
	}
}