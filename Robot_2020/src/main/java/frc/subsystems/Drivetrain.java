package frc.subsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import java.util.List;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Robot;

import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Drivetrain extends SubsystemBase
{
	// Motor Controller Declarations
	private TalonFX rightTalon1  = new TalonFX(RobotMap.DRIVETRAIN_RIGHT1);
	private TalonFX rightTalon2  = new TalonFX(RobotMap.DRIVETRAIN_RIGHT2);
	private TalonFX rightTalon3  = new TalonFX(RobotMap.DRIVETRAIN_RIGHT3);
	private TalonFX leftTalon1  = new TalonFX(RobotMap.DRIVETRAIN_LEFT1);
	private TalonFX leftTalon2  = new TalonFX(RobotMap.DRIVETRAIN_LEFT2);
	private TalonFX leftTalon3  = new TalonFX(RobotMap.DRIVETRAIN_LEFT3);
	

	// Odometry class for tracking robot pose
	private DifferentialDriveOdometry m_odometry;
	
	// private static DoubleSolenoid gearShifter = new DoubleSolenoid(RobotMap.PCM1, RobotMap.DRIVETRAIN_SHIFTER_FWD, RobotMap.DRIVETRAIN_SHIFTER_REV);

	private double angleError, turnSpeed, targetEncVal = 0;
	private double turn_Kp = 1/360, desiredAngle;
	public int moveState = 0;
	public int turnState = 0;
	private final double ENC_TICKS_PER_INCH = 58.4 / 60;

	public Drivetrain()
	{
		rightTalon2.follow(rightTalon1);
		rightTalon3.follow(rightTalon1);
		leftTalon2.follow(leftTalon1);
		leftTalon3.follow(leftTalon1);

		rightTalon1.setInverted(true);
		rightTalon2.setInverted(InvertType.FollowMaster);
		rightTalon3.setInverted(InvertType.FollowMaster);
	}

	/**
	 * A tank drive function that accounts for the joystick inversion and drivetrain drift
	 * 
	 * @param leftSpeed
	 * @param rightSpeed
	 */
	public void sigmaDrive(double leftSpeed, double rightSpeed)
	{
		rightTalon1.set(ControlMode.PercentOutput, rightSpeed);
		leftTalon1.set(ControlMode.PercentOutput, leftSpeed);
	}

	/**
	 * Enables / disables the drivetrain's highGear mode
	 * 
	 * @param gearState True to enable highGear, and False to disable it
	 */
	public void highGear(boolean gearState)
	{
		// if(gearState)
		// 	gearShifter.set(Value.kReverse);
		// else
		// 	gearShifter.set(Value.kForward);
	}

	/**
	 * Drives the robot a set distance in a straight line
	 * Does not use the gyro to account for angle
	 * 
	 * @param inches
	 * @param speed
	 */
	public boolean driveStraight(int inches)
	{

		switch(moveState)
		{
			case 0:
			targetEncVal = rightTalon1.getSelectedSensorPosition() + (inches * ENC_TICKS_PER_INCH);
			moveState = 1;
			break;
			
			case 1:
			sigmaDrive(-0.35, -0.35);
			if(rightTalon1.getSelectedSensorPosition() > targetEncVal - 5)
			{
				moveState = 2;
			}
			break;

			case 2:
			sigmaDrive(0.0, 0.0);
			moveState = 0;
			return true;
		}

		return false;
	}

	public void update()
	{

	}

	// public void talonTest(double speed)
	// {
	// 	talonFX1.set(ControlMode.PercentOutput, speed);
	// }

	/**
	 * Turns the robot to a desired angle
	 * 
	 * @param robotHeading The robot's current heading
	 * @param desiredAngle Angle that the robot will rotate towards
	 * @param rotationRate Rate at which the robot will turn
	 * @param tolerance Angle deadzone to prevent overshooting
	 * @return Lets the code know when the robot is within the target range
	 */
	public boolean toAngle(double desiredAngle, double rotationRate, double tolerance)
	{
		double output;
		double currentAngle = -Robot.navX.yaw;
		double angleDifference = desiredAngle - currentAngle;
		boolean turnFinished;

		if(angleDifference > tolerance)
		{
			turnFinished = false;
			
			if(angleDifference > 10 || angleDifference < -10)
			{
				output = 0.7*rotationRate;
			}
			else
			{
				output = 0.5*rotationRate;
			}

			if(angleDifference > 0)
			{
				sigmaDrive(-output, output);
			}
			else
			{
				sigmaDrive(output, -output);
			}
		}
		else
		{
			turnFinished = true;
		}

		return turnFinished;
	}

	/**
	 * *Broken for the time being*
	 * 
	 * @param angle
	 * @return
	 */
	public boolean turnAngle(double angle)
	{
		switch(turnState)
		{
			case 0:
			
			System.out.println("Running this state: " + turnState);

			desiredAngle =  Robot.navX.angle + angle;
			System.out.println("Current angle: " + angle + "     Desired Angle: " + desiredAngle);
			turnState++;
			
			case 1:
			angleError = desiredAngle - Robot.navX.angle;
			turnSpeed = angleError * turn_Kp;
			sigmaDrive(turnSpeed, -turnSpeed);
			if(Math.abs(angleError) < 5)
			{
				turnState++;		
				sigmaDrive(0.0, 0.0);
				return true;
			}
			break;
		}

		return false;
	}

	public double getLeftEncoderVel()
	{
		return rightTalon1.getSelectedSensorVelocity();
	}
	public double getRightEncoderVel()
	{
		return rightTalon1.getSelectedSensorVelocity();
	}

	public double getLeftEncoder()
	{
		return rightTalon1.getSelectedSensorVelocity();
	}
	public double getRightEncoder()
	{
		return rightTalon1.getSelectedSensorVelocity();
	}

	public void tankDriveVolts(double leftVolts, double rightVolts) 
	{
		// leftSparkMax1.setVoltage(leftVolts);
		// rightSparkMax1.setVoltage(rightVolts);
	}
	  
    public Pose2d getPose() 
    {
        return m_odometry.getPoseMeters();
    }

	public DifferentialDriveWheelSpeeds getWheelSpeeds() 
    {
        return new DifferentialDriveWheelSpeeds(Robot.drivetrain.getLeftEncoderVel(), Robot.drivetrain.getRightEncoderVel());
	}

	public static final double ksVolts = 0.195;
	public static final double kvVoltSecondsPerMeter = 0.0486;
	public static final double kaVoltSecondsSquaredPerMeter = 0.00531;

	// Example value only - as above, this must be tuned for your drive!
	public static final double kPDriveVel = 0.507;
	public static final double kTrackwidthMeters = 1.1232503390357302;
	public static final DifferentialDriveKinematics kDriveKinematics =
		new DifferentialDriveKinematics(kTrackwidthMeters);
	public static final double kMaxSpeedMetersPerSecond = 8;
	public static final double kMaxAccelerationMetersPerSecondSquared = 3;

	// Reasonable baseline values for a RAMSETE follower in units of meters and seconds
	public static final double kRamseteB = 2;
	public static final double kRamseteZeta = 0.7;

	public Command getAutonomousCommand() 
	{
		var autoVoltageConstraint =
		new DifferentialDriveVoltageConstraint(
			new SimpleMotorFeedforward(ksVolts,
									kvVoltSecondsPerMeter,
									kaVoltSecondsSquaredPerMeter),
			kDriveKinematics,
			10);

		// Create config for trajectory
		TrajectoryConfig config =
			new TrajectoryConfig(kMaxSpeedMetersPerSecond,
								kMaxAccelerationMetersPerSecondSquared)
				// Add kinematics to ensure max speed is actually obeyed
				.setKinematics(kDriveKinematics)
				// Apply the voltage constraint
				.addConstraint(autoVoltageConstraint);

		// An example trajectory to follow.  All units in meters.
		Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, new Rotation2d(0)),
			// Pass through these two interior waypoints, making an 's' curve path
			List.of(
				new Translation2d(1, 1),
				new Translation2d(2, -1)
			),
			// End 3 meters straight ahead of where we started, facing forward
			new Pose2d(3, 0, new Rotation2d(0)),
			// Pass config
			config
		);
		// final Drivetrain Robot.drivetrain = new Drivetrain();

		RamseteCommand ramseteCommand = new RamseteCommand(
			exampleTrajectory,
			Robot.drivetrain::getPose,
			new RamseteController(kRamseteB, kRamseteZeta),
			new SimpleMotorFeedforward(ksVolts,
									kvVoltSecondsPerMeter,
									kaVoltSecondsSquaredPerMeter),
			kDriveKinematics,
			Robot.drivetrain::getWheelSpeeds,
			new PIDController(kPDriveVel, 0, 0),
			new PIDController(kPDriveVel, 0, 0),
			// RamseteCommand passes volts to the callback
			Robot.drivetrain::tankDriveVolts,
			Robot.drivetrain);

		// Run path following command, then stop at the end.
		return ramseteCommand.andThen(() -> Robot.drivetrain.tankDriveVolts(0, 0));
	}
}