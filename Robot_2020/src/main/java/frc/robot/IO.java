package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class IO
{
    public static XboxController mainController = new XboxController(0);
    public static XboxController operatorController = new XboxController(1);

    // Main Controller Variables
    public static boolean m_buttonA, m_buttonB, m_buttonX, m_buttonXRaw, m_buttonY, m_leftBumper, m_leftBumperReleased, m_rightBumper, m_leftStick, m_rightStick, o_buttonBReleased, m_pauseButton;
    public static double m_leftTrigger, m_rightTrigger, m_leftAnalogX, m_rightAnalogX, m_leftAnalogY, m_rightAnalogY;
    public static boolean o_buttonA, o_buttonB, o_buttonX, o_buttonXReleased, o_buttonY, o_leftBumper, o_rightBumper, o_leftStick, o_rightStick;
    public static double o_leftTrigger, o_rightTrigger, o_leftAnalogX, o_rightAnalogX, o_leftAnalogY, o_rightAnalogY;
    static double  rpm = 15000;
    public static boolean WOF_Running = false;
    public static int m_DPad;

    public static void UpdateControllers()
    {
        m_buttonA = mainController.getRawButton(1);
        m_buttonB = mainController.getRawButton(2);
        m_buttonX = mainController.getRawButton(3);
        m_buttonXRaw = mainController.getRawButton(3);
        m_buttonY = mainController.getRawButtonPressed(4);
        m_leftBumper = mainController.getRawButton(5);
        m_leftBumperReleased = mainController.getRawButtonReleased(5);
        m_rightBumper = mainController.getRawButton(6);
        m_leftStick = mainController.getRawButton(9);
        m_rightStick = mainController.getRawButton(10);
        m_leftTrigger = mainController.getRawAxis(2);
        m_rightTrigger = mainController.getRawAxis(3);
        m_leftAnalogX = mainController.getRawAxis(0);
        m_rightAnalogX = mainController.getRawAxis(4);
        m_leftAnalogY = mainController.getRawAxis(1);
        m_rightAnalogY = mainController.getRawAxis(5);
        m_DPad = mainController.getPOV();
        m_pauseButton = mainController.getRawButtonPressed(8);
        SmartDashboard.putNumber("D pad", m_DPad);
        SmartDashboard.putNumber("rpm", rpm);
    }

    public static void update()
    {
        SmartDashboard.putNumber("leftanalog", m_leftAnalogY);
        SmartDashboard.putNumber("rightanalog", m_rightAnalogY);
    }

    public static void ProcessControllers()
    {
        if(m_rightTrigger > 0.5)
        {
            Robot.ballMech.variableDistanceShooter();
            // Robot.ballMech.shooterMotor1.set(ControlMode.PercentOutput, 0.9);
        }
        else
        {
            Robot.ballMech.stopShooter();
            Robot.ballMech.runRoller(0);
            Robot.sigmaSight.counter = 0;
            Robot.ballMech.counter = 0;
            Robot.ballMech.shooterState = 0;
        }

        // if(m_rightTrigger > 0.5)
        // {
        //     Robot.ballMech.setShooterMotors(rpm);
        // }
        // else
        // {
        //     Robot.ballMech.stopShooter();
        // }

        if(m_DPad == 90)
        {
            rpm += 200;
        }
        if(m_DPad == 270)
        {
            rpm -= 200;
        }

        if(m_leftTrigger > 0.5)
        {
            Robot.drivetrain.highGear(true);
        }
        else
        {
            Robot.drivetrain.highGear(false);
        }

        if(m_leftBumper)
        {
            Robot.ballMech.runRollerWithIntake();
            Robot.ballMech.intake(-1);
        }
        else if(m_rightBumper)
        {
            Robot.ballMech.runRollerWithIntake();
            Robot.ballMech.intake(1);
        }
        else if(!(m_rightTrigger > 0.5))
        {
            Robot.ballMech.stopIntake();
            Robot.ballMech.rollerState = 0;
        }

        if(m_buttonY)
        {
            Robot.wheelOfFortune.WOFCylinder();
        }

        if(m_buttonA)
        {
            WOF_Running = true;
        }

        if(WOF_Running)
        {
            Robot.wheelOfFortune.runWOF();
        }
        else
        {
            IO.mainController.setRumble(RumbleType.kLeftRumble, 0);
            Robot.wheelOfFortune.WOFmotor.set(0);
            Robot.wheelOfFortune.WOFencoder.setPosition(0);
        }
        // Robot.wheelOfFortune.ruvib();
        // System.out.println("WOF enc: " + Robot.wheelOfFortune.WOFencoder.getPosition());

        if(m_pauseButton)
        {
            Robot.climbMech.extendHanger();
        }

        if(m_DPad == 0)
        {
            Robot.climbMech.climb();
        }
        else if(m_DPad == 180)
        {
            Robot.climbMech.setClimbMotors(-.3);
        }
        else
        {
            Robot.climbMech.setClimbMotors(0);
        }

        // if(m_buttonX)
        // {
        //     Robot.drivetrain.autoDrive();
        // }
        // else
        // {
        //     Robot.drivetrain.shootCounter = 0;
        //     Robot.drivetrain.driveCounter = 0;
        // }

        // if(m_buttonB)
        // {
        //     Robot.drivetrain.turnAngle(-180);
        // }
        // else
        // {
        //     // Robot.drivetrain.sigmaDrive(0, 0);
        //     // Robot.drivetrain.driveStraightState = 0;
        // }

        // if(m_leftStick)
        // {
        //     Robot.drivetrain.driveToAngle(5, 90);
        // }

        // if(m_buttonB)
        // {
        //     Robot.ballMech.runRoller(-1);
        // }
        // else
        // {
        //     Robot.ballMech.runRoller(0);
        // }

        // if(m_buttonX)
        // {
        //     Robot.ballMech.rollerMotor.set(.6);
        // }
        // else if(!m_leftBumper)
        // {
        //     Robot.ballMech.rollerMotor.set(0);
        // }

    }
}