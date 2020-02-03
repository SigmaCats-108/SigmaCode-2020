package frc.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class Drivetrain extends SubsystemBase
{
	// Motor Controller Declarations
	private static CANSparkMax leftSparkMax1 = new CANSparkMax(RobotMap.DRIVETRAIN_LEFT1, MotorType.kBrushless);
	private static CANSparkMax leftSparkMax2 = new CANSparkMax(RobotMap.DRIVETRAIN_LEFT2, MotorType.kBrushless);
	private static CANSparkMax leftSparkMax3 = new CANSparkMax(RobotMap.DRIVETRAIN_LEFT3, MotorType.kBrushless);
	private static CANSparkMax rightSparkMax1 = new CANSparkMax(RobotMap.DRIVETRAIN_RIGHT1, MotorType.kBrushless);
	private static CANSparkMax rightSparkMax2 = new CANSparkMax(RobotMap.DRIVETRAIN_RIGHT2, MotorType.kBrushless);
	private static CANSparkMax rightSparkMax3 = new CANSparkMax(RobotMap.DRIVETRAIN_RIGHT3, MotorType.kBrushless);

	// private static TalonFX talonFX1 = new TalonFX(1);
	// private static TalonFX talonFX2 = new TalonFX(2);
	// private static TalonFX talonFX3 = new TalonFX(3);

	private static CANEncoder leftEncoder = leftSparkMax1.getEncoder();
	private static CANEncoder rightEncoder = rightSparkMax1.getEncoder();

	// Odometry class for tracking robot pose
	private DifferentialDriveOdometry m_odometry;
	
	private DifferentialDrive drive;

	// private static DoubleSolenoid gearShifter = new DoubleSolenoid(RobotMap.PCM1, RobotMap.DRIVETRAIN_SHIFTER_FWD, RobotMap.DRIVETRAIN_SHIFTER_REV);

	private double angleError, turnSpeed, targetEncVal = 0;
	private double turn_Kp = 1/360, desiredAngle;
	public int moveState = 0;
	public int turnState = 0;
	private final double ENC_TICKS_PER_INCH = 58.4 / 60;


	public Drivetrain()
	{
		// Set drivetrain motors to coast
		leftSparkMax1.setIdleMode(IdleMode.kCoast);
		leftSparkMax2.setIdleMode(IdleMode.kCoast);
		leftSparkMax3.setIdleMode(IdleMode.kCoast);
		rightSparkMax1.setIdleMode(IdleMode.kCoast);
		rightSparkMax2.setIdleMode(IdleMode.kCoast);
		rightSparkMax3.setIdleMode(IdleMode.kCoast);

		// Set up followers
		leftSparkMax2.follow(leftSparkMax1);
		leftSparkMax3.follow(leftSparkMax1);
		rightSparkMax2.follow(rightSparkMax1);
		rightSparkMax3.follow(rightSparkMax1);

		// talonFX2.follow(talonFX1);
		// talonFX3.follow(talonFX1);

		// Assign DifferentialDrive Motors
		drive = new DifferentialDrive(leftSparkMax1, rightSparkMax1);

		// Sets drivetrain deadband, default is 0.02
		drive.setDeadband(0.03);

		// talonFX1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
	}

	/**
	 * A tank drive function that accounts for the joystick inversion and drivetrain drift
	 * 
	 * @param leftSpeed
	 * @param rightSpeed
	 */
	public void sigmaDrive(double leftSpeed, double rightSpeed)
	{
		drive.tankDrive(leftSpeed /** RobotMap.DRIVETRAIN_LEFT_PGAIN*/, rightSpeed, false);
		// talonFX.set(ControlMode.PercentOutput, leftSpeed);
		// rightSparkMax1.set(rightSpeed);
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
			targetEncVal = leftEncoder.getPosition() + (inches * ENC_TICKS_PER_INCH);
			moveState = 1;
			break;
			
			case 1:
			sigmaDrive(-0.35, -0.35);
			if(leftEncoder.getPosition() > targetEncVal - 5)
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
		return leftEncoder.getVelocity();
	}
	public double getRightEncoderVel()
	{
		return rightEncoder.getVelocity();
	}

	public double getLeftEncoder()
	{
		return leftEncoder.getPosition();
	}
	public double getRightEncoder()
	{
		return rightEncoder.getPosition();
	}

	public void tankDriveVolts(double leftVolts, double rightVolts) 
	{
		leftSparkMax1.setVoltage(leftVolts);
		rightSparkMax1.setVoltage(rightVolts);
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