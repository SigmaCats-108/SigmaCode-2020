package frc.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ClimbMech
{
    private CANSparkMax climbMotor1 = new CANSparkMax(RobotMap.CLIMBMECH_MOTOR1, MotorType.kBrushless);
    private CANSparkMax climbMotor2 = new CANSparkMax(RobotMap.CLIMBMECH_MOTOR2, MotorType.kBrushless);
    public CANSparkMax weirdClimbMotor = new CANSparkMax(RobotMap.WEIRD_CLIMBMECH_MOTOR2, MotorType.kBrushed);
    private CANEncoder climbEncoder = new CANEncoder(climbMotor1);

    public DoubleSolenoid hangerSolenoid = new DoubleSolenoid(RobotMap.PCM2 ,RobotMap.HANGER_FWD , RobotMap.HANGER_REV);
    
    // private DigitalInput limitSwitch = new DigitalInput(0);
    // private AnalogInput sens = new AnalogInput(0);

    public ClimbMech()
    {
        climbMotor2.follow(climbMotor1, true);
        climbMotor1.setIdleMode(IdleMode.kBrake);
        climbMotor2.setIdleMode(IdleMode.kBrake);
        climbEncoder.setPosition(0);

        // hangerSolenoid.set(Value.kOff);
    }

    public void update()
    {
        SmartDashboard.putNumber("climb encoder", climbEncoder.getPosition());
        if(hangerSolenoid.get() == Value.kForward)
        {
            SmartDashboard.putNumber("kjjkug", 1);
        }
        else
        {
            SmartDashboard.putNumber("ihiugbi", 2);
        }
        // SmartDashboard.putNumber("ultrasonic srgf", sens.getValue());
    }

    public void extendHanger()
    {
        if(hangerSolenoid.get() == Value.kReverse)
        {
            hangerSolenoid.set(Value.kForward);
        }
        else
        {
            hangerSolenoid.set(Value.kReverse);
        }
    }

    public void setClimbMotors(double speed)
    {
        climbMotor1.set(speed);
    }

    public void climb()
	{
		if(climbEncoder.getPosition() < 106)
		{
			setClimbMotors(0.5);
        }
        else
        {
            setClimbMotors(0);
        }
    }    
}