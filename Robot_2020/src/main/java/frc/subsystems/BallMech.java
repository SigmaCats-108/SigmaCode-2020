package frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANEncoder;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.IO;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The robot mechanism for the picking up and scoring of balls.
 */
public class BallMech
{
	private CANSparkMax shooterMotor1 = new CANSparkMax(8, MotorType.kBrushless);
	private CANSparkMax shooterMotor2 = new CANSparkMax(9, MotorType.kBrushless);
	private CANSparkMax intakeMotor = new CANSparkMax(10, MotorType.kBrushless);

	private CANEncoder shooterEncoder = shooterMotor1.getEncoder();

	// private static TalonFX talonFX = new TalonFX(0);
	
	public BallMech()
	{
		// talonFX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
		// talonFX.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
	}

	public void update()
	{
		SmartDashboard.putNumber("Shooter velocity", shooterEncoder.getVelocity());
	}

	public void intake(double speed)
	{
		intakeMotor.set(speed);
	}

	public void stopIntake()
	{
		intakeMotor.set(0);
	}

	public void talonTest(double speed)
	{
		// talonFX.set(ControlMode.PercentOutput, speed);
	}

	public void bangBangShooter()
	{
		double desired_RPM = 4000;
		// double actual_RPM = talonFX.getSelectedSensorVelocity();
		double actual_RPM = shooterEncoder.getVelocity();
		if(actual_RPM <= desired_RPM)
		{
			setShooterMotors(1.0);
		}
		else
		{	
			setShooterMotors(0);
		}
	}

	public void proportionalShooter()
	{
		double desired_RPM = 3000;
		double RPM_error = desired_RPM - shooterEncoder.getVelocity();
		double shootKP = 0.0003;
		double speed = 0;
		speed += shootKP * RPM_error;
		setShooterMotors(speed);
	}

	public void setShooterMotors(double speed)
	{
		// talonFX.set(ControlMode.PercentOutput, speed);
		shooterMotor1.set(speed);
		shooterMotor2.set(-speed);
	}

	public void stopShooter()
	{
		setShooterMotors(0);
	}

	// public void extendIntake()
	// {
	// 	intakeCylinder.set(Value.kForward);
	// }
}