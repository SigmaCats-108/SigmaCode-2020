package frc.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.RobotMap;
import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Drivetrain
{
	// Motor Controller Declarations
	private static CANSparkMax leftSparkMax1 = new CANSparkMax(RobotMap.DRIVETRAIN_LEFT1, MotorType.kBrushless);
	private static CANSparkMax leftSparkMax2 = new CANSparkMax(RobotMap.DRIVETRAIN_LEFT2, MotorType.kBrushless);
	private static CANSparkMax leftSparkMax3 = new CANSparkMax(RobotMap.DRIVETRAIN_LEFT3, MotorType.kBrushless);
	private static CANSparkMax rightSparkMax1 = new CANSparkMax(RobotMap.DRIVETRAIN_RIGHT1, MotorType.kBrushless);
	private static CANSparkMax rightSparkMax2 = new CANSparkMax(RobotMap.DRIVETRAIN_RIGHT2, MotorType.kBrushless);
	private static CANSparkMax rightSparkMax3 = new CANSparkMax(RobotMap.DRIVETRAIN_RIGHT3, MotorType.kBrushless);

	private static TalonFX talonFX = new TalonFX(0);

	private static CANEncoder leftEncoder = leftSparkMax1.getEncoder();
	private static CANEncoder rightEncoder = rightSparkMax1.getEncoder();
	
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
		// leftSparkMax2.follow(leftSparkMax1);
		// leftSparkMax3.follow(leftSparkMax1);
		// rightSparkMax2.follow(rightSparkMax1);
		// rightSparkMax3.follow(rightSparkMax1);

		// Assign DifferentialDrive Motors
		drive = new DifferentialDrive(leftSparkMax1, rightSparkMax1);

		// Sets drivetrain deadband, default is 0.02
		drive.setDeadband(0.03);
	}

	/**
	 * A tank drive function that accounts for the joystick inversion and drivetrain drift
	 * 
	 * @param leftSpeed
	 * @param rightSpeed
	 */
	public void sigmaDrive(double leftSpeed, double rightSpeed)
	{
		// drive.tankDrive(-leftSpeed /** RobotMap.DRIVETRAIN_LEFT_PGAIN*/, -rightSpeed, false);
		// talonFX.set(ControlMode.PercentOutput, leftSpeed);
		rightSparkMax1.set(rightSpeed);
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

		/*
		double kp = 0.03; 
		switch(moveState)
		{
			case 0:
			targetEncVal = leftEncoder.getPosition() + (inches * ENC_TICKS_PER_INCH);
			moveState = 1;
			break;
			
			case 1:
			double displacement = targetEncVal - leftEncoder.getPosition();
			double move_command = displacement * kp;
			sigmaDrive(move_command * -1, move_command * -1);
			if(leftEncoder.getPosition() > targetEncVal - 5)
			{
				moveState = 2;
			}
			break;

			case 2:
			sigmaDrive(0.0, 0.0);
		
			return true;
			
		}
		//System.out.println("moveState: " + moveState);
		return false;
		*/
	}

	public void update()
	{
		SmartDashboard.putNumber("LeftEncoder", leftEncoder.getPosition());
	}

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
}