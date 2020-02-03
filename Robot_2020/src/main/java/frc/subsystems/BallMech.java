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
import edu.wpi.first.wpilibj.Ultrasonic;

/**
 * The robot mechanism for the picking up and scoring of balls.
 */
public class BallMech
{
	private CANSparkMax shooterMotor1 = new CANSparkMax(8, MotorType.kBrushless);
	private CANSparkMax shooterMotor2 = new CANSparkMax(9, MotorType.kBrushless);
	private CANSparkMax intakeMotor = new CANSparkMax(16, MotorType.kBrushless);
	private CANSparkMax rollerMotor = new CANSparkMax(11, MotorType.kBrushed);
	private CANSparkMax indexMotor = new CANSparkMax(10, MotorType.kBrushless);

	private CANEncoder shooterEncoder = shooterMotor1.getEncoder();

	private AnalogInput test = new AnalogInput(3);
	private Ultrasonic ultrasonic = new Ultrasonic(0,1);

	// public static TalonFX talonFX = new TalonFX(3);
	
	public BallMech()
	{
		// talonFX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
		// talonFX.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
		// test.setOversampleBits(4);

		// talonFX.config_kF(RobotMap.kPIDLoopIdx, RobotMap.kF, RobotMap.kTimeoutMs);
		// talonFX.config_kP(RobotMap.kPIDLoopIdx, RobotMap.kP, RobotMap.kTimeoutMs);
		// talonFX.config_kI(RobotMap.kPIDLoopIdx, RobotMap.kI, RobotMap.kTimeoutMs);
		// talonFX.config_kD(RobotMap.kPIDLoopIdx, RobotMap.kD, RobotMap.kTimeoutMs);
		// ultrasonic.setAutomaticMode(true);

	}

	public void update()
	{
		// SmartDashboard.putNumber("sensor", test.getValue());
		// SmartDashboard.putNumber("ultrasonic", ultrasonic.getRangeInches());
		SmartDashboard.putNumber("Velocity", shooterEncoder.getVelocity());

		// System.out.println("OpenRampRate " + shooterMotor1.getOpenLoopRampRate());
		// System.out.println("ClosedRampRate " + shooterMotor1.getClosedLoopRampRate());
		// talonFX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
	}

	public void intake(double speed)
	{
		intakeMotor.set(speed);
		rollerMotor.set(speed);
	}

	public void stopIntake()
	{
		intakeMotor.set(0);
		rollerMotor.set(0);
	}

	public void talonTest(double speed)
	{
		// talonFX.set(ControlMode.PercentOutput, speed);
	}

	double shooterSpeed = 1.0;
	int count = 0;
	public void variableDistanceShooter(double desired_RPM )
	{
		count++;
		double actual_RPM = shooterEncoder.getVelocity();
		System.out.println("actualRPM " + actual_RPM);
		if(actual_RPM==0)
			setShooterMotors(1);
	
		if(count == 10)
		{
			double RPM_error = actual_RPM - desired_RPM ;
			double percentDifference = RPM_error / desired_RPM ;
			shooterSpeed -= percentDifference * 0.35;
			if(shooterSpeed > 1)
			shooterSpeed = 1;

			if(shooterSpeed < -1)
			shooterSpeed = -1;
			setShooterMotors(shooterSpeed);
			count = 0;
			System.out.println("CHANGING SPEED");
			System.out.println("RPM_error " + RPM_error);
			System.out.println("percentDifference " + percentDifference);
			System.out.println("---------------");
		}
		System.out.println("shooterSpeed " + shooterSpeed);
	}


	// double shooterSpeed = 1.0;
	// int count = 0;
	// public void shoot(double desired_RPM )
	// {
	// 	count++;
	// 	double actual_RPM = talonFX.getSelectedSensorVelocity();
	// 	System.out.println("actualRPM " + actual_RPM);
	// 	if(actual_RPM==0)
	// 		setShooterMotors(1);
	
	// 	if(count == 10)
	// 	{
	// 		double RPM_error = actual_RPM - desired_RPM ;
	// 		double percentDifference = RPM_error / desired_RPM ;
	// 		shooterSpeed -= percentDifference * 0.35;
	// 		if(shooterSpeed > 1)
	// 		shooterSpeed = 1;

	// 		if(shooterSpeed < -1)
	// 		shooterSpeed = -1;
	// 		setShooterMotors(shooterSpeed);
	// 		count = 0;
	// 		System.out.println("CHANGING SPEED");
	// 		System.out.println("RPM_error " + RPM_error);
	// 		System.out.println("percentDifference " + percentDifference);
	// 		System.out.println("---------------");
	// 	}
	// 	System.out.println("shooterSpeed " + shooterSpeed);
	// }

	public boolean runIndexer(double RPM)
	{
		if(shooterEncoder.getVelocity() > RPM - 100 && shooterEncoder.getVelocity() < RPM + 100)
		{
			indexMotor.set(0.5);
			return true;
		}
		indexMotor.set(0);
		return false;
	}

	public void setShooterMotors(double speed)
	{
		// talonFX.set(ControlMode.Velocity, speed);
		shooterMotor1.set(speed);
		shooterMotor2.set(-speed);
	}

	public void stopShooter()
	{
		setShooterMotors(0);
		indexMotor.set(0);
	}

	// public void extendIntake()
	// {
	// 	intakeCylinder.set(Value.kForward);
	// }

	// public int shooterState = 0;
	// private double required_RPM = 0;
	// public boolean shootSequence()
	// {
	// 	switch(shooterState)
	// 	{
	// 		case 0:
	// 		// if (Robot.sigmaSight.lineUpToShoot())
	// 		// {
	// 		// 	shooterState = 1;
	// 		// }
	// 		shooterState = 1;
	// 		break;

	// 		case 1:
	// 		required_RPM = Robot.sigmaSight.area * 600;
	// 		shoot(3000);
	// 		shooterState = 2;
	// 		break;

	// 		case 2:
	// 		shoot(3000);
	// 		if(runIndexer(3000))
	// 		{
	// 			shooterState = 1;
	// 		}
	// 		break;
	// 	}

	// 	return false;
	// }


}