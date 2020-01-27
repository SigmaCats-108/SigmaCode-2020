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
		// System.out.println("OpenRampRate " + shooterMotor1.getOpenLoopRampRate());
		// System.out.println("ClosedRampRate " + shooterMotor1.getClosedLoopRampRate());
		// talonFX.set(TalonFXControlMode.Velocity,3000);
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

	double speed = 1.0;
	float kp = 0.0008f; 				// proportional konstant
	float ki = .0f;   			//konstant of integration
	float kd =  .5f; 			//konstant of derivation
	float current = 0;
	float integralActiveZone = 2;	// zone of error values in which the total error for the                    integral term accumulates
	float errorT;						// total error accumulated
	float lastError;					// last error recorded by the controller
	float proportion;						// the proportional term
	float integral;							// the integral term
	float derivative;
	public void shoot()
	{
		float error = 3000 - (float)shooterEncoder.getVelocity(); // calculates difference between current velocity and target velocity

		if(error <integralActiveZone && errorT != 0)// total error only accumulates where        ///there is error, and when the error is 
								//within the integral active zone
		{
			errorT += error;// adds error to the total each time through the loop
		}
		else{
			errorT = 0;// if error = zero or error is not withing the active zone, total       ///error is set to zero
		}

		if(errorT > 50/ki) //caps total error at 50
		{
			errorT = 50/ki;
		}
		if(error == 0){
			derivative = 0; // if error is zero derivative term is zero
		}
		proportion 	 =     error             * kp; // sets proportion term
		integral 		 =     errorT            * ki;// sets integral term
		derivative   =    (error - lastError)* kd;// sets derivative term

		lastError = error; // sets the last error to current error so we can use it in the next loop

		current =  proportion + integral + derivative;// sets value current as total of all terms

		setShooterMotors(current); // sets motors to the calculated value
	}
	
	
	int count = 0;
	public void bangBangShooter()
	{
		count++;
		double desired_RPM = 3000;
		// double actual_RPM = talonFX.getSelectedSensorVelocity();
		double actual_RPM = shooterEncoder.getVelocity();
		System.out.println("actualRPM " + actual_RPM);
		if(actual_RPM==0)
			setShooterMotors(1);
			
		// if(actual_RPM <= desired_RPM)
		// {
		// 	setShooterMotors(speed);
		// }
		// else
		// {	
			if(count == 25)
			{
				double RPM_error = actual_RPM - desired_RPM;
				double percentDifference = RPM_error / desired_RPM;
				speed -= percentDifference * 0.35;
				if(speed > 1)
					speed = 1;

				if(speed < -1)
					speed = -1;
				setShooterMotors(speed);
				count = 0;
				System.out.println("CHANGINE SPEED");
				System.out.println("RPM_error " + RPM_error);
				System.out.println("percentDifference " + percentDifference);
				System.out.println("---------------");
			}
			System.out.println("speed " + speed);

			
		}

	public void proportionalShooter()
	{
		double desired_RPM = 3000;
		double RPM_error = desired_RPM - shooterEncoder.getVelocity();
		double shootKP = 0.0003;
		double speed = 0;
		speed += shootKP * RPM_error;
		double speedError = (1 - speed) * .002;
		setShooterMotors(speed - speedError);
		System.out.println("RPM_error " + RPM_error);
		System.out.println("speed " + speed);
		System.out.println("speedError " + speedError);
		System.out.println("Velocity " + shooterEncoder.getVelocity());
		System.out.println("Speed - speedError " + (speed - speedError));
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