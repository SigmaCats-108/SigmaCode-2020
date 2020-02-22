package frc.robot;

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
        m_rightBumper = mainController.getRawButtonPressed(6);
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
    }

    public static void ProcessControllers()
    {
        if(m_rightTrigger > 0.5)
        {
            Robot.ballMech.variableDistanceShooter();;
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
        //     Robot.ballMech.setShooterMotors(Robot.sigmaSight.desiredSpeed());
        // }
        // else
        // {
        //     Robot.ballMech.stopShooter();
        // }

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
            Robot.ballMech.intake(-0.90);
        }
        else
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
        Robot.wheelOfFortune.ruvib();
        System.out.println("WOF enc: " + Robot.wheelOfFortune.WOFencoder.getPosition());

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