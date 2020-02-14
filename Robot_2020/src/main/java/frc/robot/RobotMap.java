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
	public static final int DRIVETRAIN_RIGHT1 = 4;
	public static final int DRIVETRAIN_RIGHT2 = 5;
	public static final int DRIVETRAIN_RIGHT3 = 6;
	public static final int DRIVETRAIN_LEFT1 = 1;
	public static final int DRIVETRAIN_LEFT2 = 2;
	public static final int DRIVETRAIN_LEFT3 = 3;

	// BallMech ID Values
	public static final int BALLMECH_INTAKE = 1;
	public static final int BALLMECH_ROLLER = 2;

	//climbMech ID Values
	public static final int CLIMBMECH_MOTOR1 = 3;
	public static final int CLIMBMECH_MOTOR2 = 4;

	// Vision System Constants
	public static final double HATCH_VISION_TURN_PGAIN = -0.02;
	public static final double HATCH_VISION_DISTANCE_PGAIN = 0.3;
	public static final double HATCH_VISION_MIN_AIM_COMMAND = 0.0;
	public static final double HATCH_VISION_DESIRED_TARGET_AREA = 4.0;	

	public static final int kPIDLoopIdx = 0;
	public static final double kF = 0.05;
	public static final double kP = 0.008;
	public static final double kI = 0;
	public static final double kD = 0;
	public static final int kTimeoutMs = 0;

	//Pneumatics
	public static final int PCM1 = 0;
	public static final int PCM2 = 1;
	//gray
	public static final int DRIVETRAIN_SHIFTER_FWD = 6;
	public static final int DRIVETRAIN_SHIFTER_REV = 1;
	//purple
	public static final int INTAKE_EXTENDER_FWD = 2;
	public static final int INTAKE_EXTENDER_REV = 5;
	//white
	public static final int HANGER_FWD = 4;
	public static final int HANGER_REV = 3;
}