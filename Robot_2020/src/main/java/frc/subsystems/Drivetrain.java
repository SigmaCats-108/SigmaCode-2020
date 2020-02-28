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
	public TalonFX rightTalon1  = new TalonFX(RobotMap.DRIVETRAIN_RIGHT1);
	public TalonFX rightTalon2  = new TalonFX(RobotMap.DRIVETRAIN_RIGHT2);
	//publicate TalonFX rightTalon3  = new TalonFX(RobotMap.DRIVETRAIN_RIGHT3); //bottom motor
	public TalonFX leftTalon1  = new TalonFX(RobotMap.DRIVETRAIN_LEFT1);
	//publicate TalonFX leftTalon2  = new TalonFX(RobotMap.DRIVETRAIN_LEFT2); //bottom motor
	public TalonFX leftTalon3  = new TalonFX(RobotMap.DRIVETRAIN_LEFT3);
		
	private static DoubleSolenoid gearShifter = new DoubleSolenoid(RobotMap.PCM2, RobotMap.DRIVETRAIN_SHIFTER_FWD, RobotMap.DRIVETRAIN_SHIFTER_REV);
	
	public int moveState = 0;
	public int turnState = 0;
	private final double ENC_TICKS_PER_INCH = 131113.5 / 120; //measured value of encoder ticks per inch

	public Drivetrain()
	{
		rightTalon2.follow(rightTalon1);
		// rightTalon3.follow(rightTalon1);
		// leftTalon2.follow(leftTalon1);
		leftTalon3.follow(leftTalon1);

		rightTalon1.setInverted(true);
		rightTalon2.setInverted(InvertType.FollowMaster);
		// rightTalon3.setInverted(InvertType.FollowMaster);

		

		rightTalon1.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
		leftTalon1.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);

		// rightTalon1.configGetStatorCurrentLimit(currentLimit);
		// rightTalon2.configGetStatorCurrentLimit(currentLimit);
		// // rightTalon3.configGetStatorCurrentLimit(currentLimit);
		// leftTalon1.configGetStatorCurrentLimit(currentLimit);
		// // leftTalon2.configGetStatorCurrentLimit(currentLimit);
		// leftTalon3.configGetStatorCurrentLimit(currentLimit);
	}

	/**
	 * A tank drive function that accounts for the joystick inversion and drivetrain drift
	 * 
	 * @param leftSpeed
	 * @param rightSpeed
	 */
	public void sigmaDrive(double leftSpeed, double rightSpeed)
	{
		rightTalon1.set(ControlMode.PercentOutput, rightSpeed); //motor broken
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

	double turnKP = 0.01;
	double distanceKP = 0.000003;
	public boolean driveToAngle(double distance_inches, double endPose)
	{
		double averageEncoderPosition = -rightTalon1.getSelectedSensorPosition();
		double desiredPosition = averageEncoderPosition + (distance_inches * ENC_TICKS_PER_INCH);
		double distance_adjust = (desiredPosition - averageEncoderPosition) * distanceKP;
		double angle_adjust = (endPose - Robot.navX.angle) * turnKP;
		double left_command = -angle_adjust + distance_adjust;
		double right_command = angle_adjust + distance_adjust;
		sigmaDrive(left_command, right_command);
		return averageEncoderPosition > desiredPosition - 100 && averageEncoderPosition < desiredPosition + 100 
		&& Robot.navX.angle > endPose - 2 && Robot.navX.angle < endPose + 2;
	}

	public boolean turnAngle(double angle)
	{
		double speed = (angle - Robot.navX.angle) * 0.006;
		sigmaDrive(-speed, speed);
		if(Math.abs(angle - Robot.navX.angle) < 5)
		{
			sigmaDrive(0, 0);
			return true;
		}
		return false;
	}

	public int driveStraightState = 0;
	double desiredPosition = 0, averageEncoderPosition = 0;
	double distance_adjust = 0;
	double currentAngle = 0;
	public boolean driveStraight(double distance_inches)
	{
		switch(driveStraightState)
		{
			case 0:
			averageEncoderPosition = -rightTalon1.getSelectedSensorPosition();
			desiredPosition = averageEncoderPosition + (distance_inches * ENC_TICKS_PER_INCH);
			driveStraightState = 1;
			currentAngle = Robot.navX.angle;
			break;

			case 1:
			double angle_adjust = (currentAngle - Robot.navX.angle) * 0.028;
			distance_adjust = (desiredPosition - (-rightTalon1.getSelectedSensorPosition())) * distanceKP;
			sigmaDrive(-distance_adjust - angle_adjust, -distance_adjust + angle_adjust);
			// System.out.println("speed" + distance_adjust);
			// System.out.println("desired" + desiredPosition);
			if(Math.abs(distance_adjust) < 0.2)
			{
				driveStraightState = 2;
			}
			break;

			case 2:
			sigmaDrive(0, 0);
			// System.out.println("FINISHED");
			return true;
		}
		return false;
	}

	// public int driveCounter = 0;
	// public int shootCounter = 0;
	// public void autoDrive()
	// {
	// 	sigmaDrive(0.2, 0.2);
	// 	if(driveCounter > 125)
	// 	{
	// 		sigmaDrive(0, 0);
	// 		if(shootCounter < 200)
	// 		{
	// 			Robot.ballMech.variableDistanceShooter();
	// 		}
	// 		else
	// 		{
	// 			Robot.ballMech.stopShooter();
	// 			Robot.ballMech.runRoller(0);
	// 			Robot.sigmaSight.counter = 0;
	// 			Robot.ballMech.counter = 0;
	// 			Robot.ballMech.shooterState = 0;
	// 		}
	// 		shootCounter++;
	// 	}
	// 	driveCounter++;
	// }

	public int autoState = 0;
	int counter = 0;
	public void autonomous()
	{
		switch(autoState)
		{
			case 0:
			if(driveToAngle(100, 20))
			{
				autoState = 1;
			}
			break;

			case 1:
			Robot.ballMech.intake(-0.9);
			if(Robot.ballMech.ballIsInRobot())
			{
				autoState = 2;
			}
			break;

			case 2:
			Robot.ballMech.stopIntake();
			if(driveToAngle(0, 180))
			{
				autoState = 3;
			}
			break;

			case 3:
			Robot.ballMech.variableDistanceShooter();
			if(counter > 150)
			{
				autoState = 4;
			}
			counter++;
			break;

			case 4:
			if(driveToAngle(50, 0))
			{
				autoState = 5;
			}
			break;

			case 5:
			Robot.ballMech.intake(-0.9);
			if(driveToAngle(100, 0))
			{
				autoState = 6;
			}
			break;
			
			case 6:
			Robot.ballMech.stopIntake();
			if(driveToAngle(0, 180))
			{
				autoState = 7;
			}
			break;

			case 7:
			Robot.ballMech.variableDistanceShooter();
			if(counter > 150)
			{
				autoState = 4;
			}
			counter++;
			break;
		}
	}

	int autoCounter = 0;
	public void sixBallAuto()
	{
		switch(autoState)
		{
			case 0:
			Robot.ballMech.variableDistanceShooter();
			if(autoCounter > 200)
			{
				Robot.ballMech.stopShooter();
				Robot.ballMech.runRoller(0);
				Robot.sigmaSight.counter = 0;
				Robot.ballMech.counter = 0;
				Robot.ballMech.shooterState = 0;
				autoState = 1;
			}
			autoCounter++;
			break;

			case 1:
			if(turnAngle(-181))
			{
				autoState = 2;
			}
			break;

			case 2:
			Robot.ballMech.intake(-1);
			// System.out.println("driving straight");
			if(driveStraight(140))
			{
				autoState = 3;
			}
			break;

			case 3:
			Robot.ballMech.stopIntake();
			if(turnAngle(0))
			{
				autoState = 4;
			}
			break;

			case 4:
			if(driveStraight(75))
			{
				autoState = 5;
			}

			case 5:
			Robot.ballMech.variableDistanceShooter();
			break;
		}
	}

	public void eightBallAuto(double distance_inches, double endPose)
	{
		switch(autoState)
		{
			case 0:
			Robot.ballMech.variableDistanceShooter();
			if(autoCounter > 200)
			{
				Robot.ballMech.stopShooter();
				Robot.ballMech.runRoller(0);
				Robot.sigmaSight.counter = 0;
				Robot.ballMech.counter = 0;
				Robot.ballMech.shooterState = 0;
				autoState = 1;
			}
			autoCounter++;
			break;

			case 1:
			if(turnAngle(-176))
			{
				autoState = 2;
			}
			break;

			case 2:
			// Robot.ballMech.intake(-1);
			// System.out.println("driving straight");
			if(driveStraight(250))
			{
				autoState = 3;
			}
			break;

			case 3:
			Robot.ballMech.stopIntake();
			if(driveStraight(-150))
			{
				autoState = 4;
			}
			break;

			case 4:
			if(turnAngle(0))
			{
				autoState = 5;
			}
			break;

			case 5:
			Robot.ballMech.variableDistanceShooter();
			break;
		}
	}

	public void threeBallAuto()
	{
		switch(autoState)
		{
			case 0:
			Robot.ballMech.variableDistanceShooter();
			if(autoCounter > 250)
			{
				Robot.ballMech.stopShooter();
				Robot.ballMech.runRoller(0);
				Robot.sigmaSight.counter = 0;
				Robot.ballMech.counter = 0;
				Robot.ballMech.shooterState = 0;
				autoState = 1;
			}
			autoCounter++;
			break;

			case 1:
			driveStraight(-70);
			break;
		}
	}

	public void update()
	{
		// SmartDashboard.putNumber("encoder", (leftTalon1.getSelectedSensorPosition() + rightTalon1.getSelectedSensorPosition()) / 2);
		// SmartDashboard.putNumber("leftenc", leftTalon1.getSelectedSensorPosition());
		// SmartDashboard.putNumber("rightenc", rightTalon1.getSelectedSensorPosition());
		// SmartDashboard.putNumber("angle", Robot.navX.angle);
		SmartDashboard.putNumber("outputcurrent", rightTalon1.getSupplyCurrent());
	}
}