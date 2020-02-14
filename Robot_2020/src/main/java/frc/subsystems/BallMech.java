package frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * The robot mechanism for the picking up and scoring of balls.
 */
public class BallMech
{
	private TalonFX shooterMotor1 = new TalonFX(7);
	private TalonFX shooterMotor2 = new TalonFX(8);
	private CANSparkMax intakeMotor = new CANSparkMax(1, MotorType.kBrushless);
	private CANSparkMax rollerMotor = new CANSparkMax(2, MotorType.kBrushed);
	private DoubleSolenoid intakeCylinder = new DoubleSolenoid(RobotMap.PCM2 ,RobotMap.INTAKE_EXTENDER_FWD , RobotMap.INTAKE_EXTENDER_REV);
	private Ultrasonic ballSensor_intake = new Ultrasonic(0, 1);
	private Ultrasonic ballSensor_shooter = new Ultrasonic(2, 3);
	
	public BallMech()
	{
		shooterMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
		shooterMotor1.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);

		shooterMotor1.config_kF(RobotMap.kPIDLoopIdx, RobotMap.kF, RobotMap.kTimeoutMs);
		shooterMotor1.config_kP(RobotMap.kPIDLoopIdx, RobotMap.kP, RobotMap.kTimeoutMs);
		shooterMotor1.config_kI(RobotMap.kPIDLoopIdx, RobotMap.kI, RobotMap.kTimeoutMs);
		shooterMotor1.config_kD(RobotMap.kPIDLoopIdx, RobotMap.kD, RobotMap.kTimeoutMs);
		
		shooterMotor2.follow(shooterMotor1);
		shooterMotor2.setInverted(InvertType.OpposeMaster);
	}

	public void update()
	{
		SmartDashboard.putNumber("velocity", shooterMotor1.getSelectedSensorVelocity());
	}

	public void intake(double speed)
	{
		intakeMotor.set(speed);
		extendIntake(true);
	}

	public void stopIntake()
	{
		intakeMotor.set(0);
		extendIntake(false);
	}

	public boolean setShooterMotors(double speed)
	{
		shooterMotor1.set(ControlMode.Velocity, speed);
		return Math.abs(shooterMotor1.getSelectedSensorVelocity() - speed) < 500;
	}

	public void stopShooter()
	{
		shooterMotor1.set(ControlMode.PercentOutput, 0);
	}

	public void runRoller(double speed)
	{
		rollerMotor.set(speed);
	}

	int state = 0;
	private int ballCount()
	{
		int ballCount = 0;
		switch(state)
		{
			case 0:
			if(ballSensor_intake.getRangeInches() < 70)
			{
				ballCount++;
				state = 1;
			}
			break;

			case 1:
			if(ballSensor_intake.getRangeInches() > 80)
			{
				state = 0;
			}
			break;
		}
		return ballCount;
	}

	public void moveBalls()
	{
		if(ballCount() >= 3 && ballSensor_shooter.getRangeInches() > 30)
		{
			runRoller(0.5);
		}
		else
		{
			runRoller(0);
		}
	}

	public void extendIntake(boolean intakeState)
	{
		if(intakeState)
		{
			intakeCylinder.set(Value.kReverse);
		}
		else
		{
			intakeCylinder.set(Value.kForward);
		}
	}

	public int shooterState = 0;
	public void variableDistanceShooter()
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
			if(setShooterMotors(Robot.sigmaSight.desiredSpeed()))
			{
				runRoller(-1);
			}
			else
			{
				runRoller(0);
			}
			break;
		}
	}
}