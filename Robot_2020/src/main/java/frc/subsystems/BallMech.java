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
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The robot mechanism for the picking up and scoring of balls.
 */
public class BallMech
{
	private TalonFX shooterMotor1 = new TalonFX(8);
	private CANSparkMax intakeMotor = new CANSparkMax(16, MotorType.kBrushless);
	private CANSparkMax rollerMotor = new CANSparkMax(11, MotorType.kBrushed);
	private CANSparkMax indexMotor = new CANSparkMax(10, MotorType.kBrushless);
	private DoubleSolenoid intakeCylinder = new DoubleSolenoid(2,3);
	private AnalogInput ballSensor_intake = new AnalogInput(0);
	private AnalogInput ballSensor_indexer = new AnalogInput(1);
	
	public BallMech()
	{
		shooterMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
		shooterMotor1.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
	}

	public void update()
	{

	}

	public void intake(double speed)
	{
		intakeMotor.set(speed);
	}

	public void stopIntake()
	{
		intakeMotor.set(0);
	}

	private void setShooterMotors(double speed)
	{
		shooterMotor1.set(ControlMode.PercentOutput, speed);
	}

	public void stopShooter()
	{
		setShooterMotors(0);
		indexMotor.set(0);
	}

	private void runRoller()
	{
		 
	}

	int state = 0;
	private int ballCount()
	{
		int ballCount = 0;
		switch(state)
		{
			case 0:
			if(ballSensor_intake.getVoltage() < 5)
			{
				ballCount++;
				state = 1;
			}
			break;

			case 1:
			if(ballSensor_intake.getVoltage() > 10)
			{
				state = 0;
			}
			break;
		}
		return ballCount;
	}

	public void extendIntake()
	{
		if(intakeCylinder.get() == Value.kForward)
		{	
			intakeCylinder.set(Value.kReverse);
		}
		else
		{
			intakeCylinder.set(Value.kForward);
		}
	}
	public int shooterState = 0;
	private double required_RPM = 0;
	public boolean shootSequence()
	{
		switch(shooterState)
		{
			case 0:
			if (Robot.sigmaSight.lineUpToShoot())
			{
				shooterState = 1;
			}
			break;

			case 1:

			break;

			case 2:

			break;
		}

		return false;
	}
}