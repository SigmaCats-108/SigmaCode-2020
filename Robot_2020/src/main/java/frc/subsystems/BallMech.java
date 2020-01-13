package frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANEncoder;
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
	public CANSparkMax shooterMotor1 = new CANSparkMax(7, MotorType.kBrushless);
	public CANSparkMax shooterMotor2 = new CANSparkMax(8, MotorType.kBrushless);
	// static CANEncoder encoder1 = shooterMotor1.getEncoder();
	//private static CANEncoder encoder2 = shooterMotor2.getEncoder();
	
	public BallMech()
	{

	}





}