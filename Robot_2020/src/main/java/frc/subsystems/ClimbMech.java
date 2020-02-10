package frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Robot;
import frc.robot.RobotMap;


public class ClimbMech
{

	private DoubleSolenoid hangerSolenoid = new DoubleSolenoid(RobotMap.PCM2 ,RobotMap.HANGER_FWD , RobotMap.HANGER_REV);

    public ClimbMech()
    {

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
}