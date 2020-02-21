package frc.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Drivetrain
{
	// Motor Controller Declarations
	private TalonFX rightTalon1  = new TalonFX(RobotMap.DRIVETRAIN_RIGHT1);
	private TalonFX rightTalon2  = new TalonFX(RobotMap.DRIVETRAIN_RIGHT2);
	private TalonFX rightTalon3  = new TalonFX(RobotMap.DRIVETRAIN_RIGHT3);
	private TalonFX leftTalon1  = new TalonFX(RobotMap.DRIVETRAIN_LEFT1);
	private TalonFX leftTalon2  = new TalonFX(RobotMap.DRIVETRAIN_LEFT2);
	private TalonFX leftTalon3  = new TalonFX(RobotMap.DRIVETRAIN_LEFT3);
		
	private static DoubleSolenoid gearShifter = new DoubleSolenoid(RobotMap.PCM2, RobotMap.DRIVETRAIN_SHIFTER_FWD, RobotMap.DRIVETRAIN_SHIFTER_REV);

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

		rightTalon1.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
		leftTalon1.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
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
		if(gearState)
		{
			gearShifter.set(Value.kReverse);
		}
		else
		{
			gearShifter.set(Value.kForward);
		}
	}

	double turnKP = 0.1;
	double distanceKP = 0.1;
	public void driveToAngle(double distance_inches, double endPose)
	{
		double averageEncoderPosition = (leftTalon1.getSelectedSensorPosition() + rightTalon1.getSelectedSensorPosition()) / 2;
		double desiredPosition = averageEncoderPosition + (distance_inches * ENC_TICKS_PER_INCH);
		double distance_adjust = (desiredPosition - averageEncoderPosition) * distanceKP;
		double angle_adjust = (endPose - Robot.navX.angle) * turnKP;
		double left_command = angle_adjust + distance_adjust;
		double right_command = -angle_adjust + distance_adjust;
		sigmaDrive(left_command, right_command);
	}

	public void update()
	{
		SmartDashboard.putNumber("encoder", (leftTalon1.getSelectedSensorPosition() + rightTalon1.getSelectedSensorPosition()) / 2);
	}
}