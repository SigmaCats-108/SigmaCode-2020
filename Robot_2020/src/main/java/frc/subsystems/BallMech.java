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

/**
 * The robot mechanism for the picking up and scoring of balls.
 */
public class BallMech
{
	public CANSparkMax shooterMotor1 = new CANSparkMax(8, MotorType.kBrushless);
	public CANSparkMax shooterMotor2 = new CANSparkMax(10, MotorType.kBrushless);
	public CANSparkMax intakeMotor = new CANSparkMax(12, MotorType.kBrushless);

	private static TalonFX talonFX = new TalonFX(0);
	
	public BallMech()
	{
		talonFX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
		talonFX.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
	}

	public void update()
	{
		System.out.println(talonFX.getSelectedSensorVelocity());
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
		talonFX.set(ControlMode.PercentOutput, speed);
	}

	public void bangBangShooter()
	{
		double desired_RPM = 10000;
		double actual_RPM = talonFX.getSelectedSensorVelocity();
		if(actual_RPM <= desired_RPM)
		{
			setShooterMotors(0.6);
		}
		else
		{	
			setShooterMotors(0.59);
		}
	}

	public void setShooterMotors(double speed)
	{
		talonFX.set(ControlMode.PercentOutput, speed);
	}

	public void stopShooter()
	{
		talonFX.set(ControlMode.PercentOutput, 0);
	}

	// public void extendIntake()
	// {
	// 	intakeCylinder.set(Value.kForward);
	// }
}