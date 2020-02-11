package frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.RobotMap;


public class ClimbMech
{
    private CANSparkMax climbMotor1 = new CANSparkMax(RobotMap.CLIMBMECH_MOTOR1, MotorType.kBrushed);
    private CANSparkMax climbMotor2 = new CANSparkMax(RobotMap.CLIMBMECH_MOTOR2, MotorType.kBrushed);

    private DoubleSolenoid hangerSolenoid = new DoubleSolenoid(RobotMap.PCM2 ,RobotMap.HANGER_FWD , RobotMap.HANGER_REV);
    
    private DigitalInput limitSwitch = new DigitalInput(0);

    public ClimbMech()
    {
        climbMotor2.follow(climbMotor1, true);
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

    public void liftRobot()
    {
        if(!limitSwitch.get())
        {
            setClimbMotors(0.5);
        }
        else
        {
            setClimbMotors(0);
        }
    }
    
}