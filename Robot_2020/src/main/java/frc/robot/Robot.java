package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.subsystems.SigmaSight;
import frc.subsystems.ColorWheel;
import frc.subsystems.Drivetrain;
import frc.SensorInputs.NavX;
import frc.subsystems.BallMech;
import frc.subsystems.ClimbMech;

public class Robot extends TimedRobot 
{
    public static SigmaSight sigmaSight;  
    public static Drivetrain drivetrain;
    public static NavX navX;
    public static ColorWheel wheelOfFortune;
    public static BallMech ballMech;
    public static ClimbMech climbMech;
    public static SupplyCurrentLimitConfiguration currentLimit;

    SendableChooser<String> m_chooser;
	final String kAutoNameDefault = "!Do Nothing!";
	final String shoot3 = "shoot 3 and back up";
	final String sixBallAuto = "six ball auto";
	// String rightAuto = "Right Auto";
	// String centerAuto = "Center Auto";
	String m_autoSelected;

    @Override
    public void robotInit() 
    {
        currentLimit = new SupplyCurrentLimitConfiguration(true, 20, 25, 1);
        sigmaSight = new SigmaSight();
        drivetrain = new Drivetrain();
        navX = new NavX();
        wheelOfFortune = new ColorWheel();
        ballMech = new BallMech();
        climbMech = new ClimbMech();

        Robot.climbMech.hangerSolenoid.set(Value.kReverse);

        // m_chooser.setDefaultOption(kAutoNameDefault, kAutoNameDefault);
		// m_chooser.addOption(shoot3, shoot3);
		// m_chooser.addOption(sixBallAuto, sixBallAuto);
		// // m_chooser.addOption(centerAuto, centerAuto);
		// // m_chooser.addOption(rightAuto, rightAuto);
		// SmartDashboard.putData("Auto Modes", m_chooser);
    }

    @Override
    public void robotPeriodic()
    {
        sigmaSight.updateValues();
        // sigmaSight.testValues();
        drivetrain.update();
        navX.updateAHRS();
        // navX.update();
        ballMech.update();
        wheelOfFortune.updateColors();
        // climbMech.update();
        IO.update();
    }
    
    @Override
    public void autonomousInit() 
    {
        // m_autoSelected = m_chooser.getSelected();

        Robot.navX.resetAngle();
        Robot.drivetrain.autoState = 0;
        Robot.drivetrain.driveStraightState = 0;
    }
    // int autoState = 0;
    int autoCounter = 0;
    @Override
    public void autonomousPeriodic()
    {
        // Robot.sigmaSight.turnOnLights();
        // // Robot.drivetrain.autoDrive();
        // Robot.drivetrain.sixBallAuto(100, 10);
        // Robot.ballMech.runRoller(-1);
        // Robot.ballMech.shooterMotor1.set(ControlMode.PercentOutput, 0.25);
        // switch(m_autoSelected)
        // {
        //     case shoot3:
        //     // Robot.drivetrain.threeBallAuto();
        //     break;
            
        //     case sixBallAuto:
        //     // Robot.drivetrain.sixBallAuto(100, 50);
        //     System.out.println("STUFF STUFF STUFF");
        //     break;
        // }
        // if(autoCounter > 000)
        // {
        //     Robot.drivetrain.threeBallAuto();
        // }
        // autoCounter++;
        // Robot.drivetrain.sixBallAutoTwo();
        // Robot.drivetrain.turnAngle(-181);
        // Robot.drivetrain.driveStraight(270);
        Robot.drivetrain.threeBallAuto();
    }

    @Override
    public void teleopInit()
    {
        Robot.drivetrain.rightTalon1.configGetSupplyCurrentLimit(currentLimit, 500);
		Robot.drivetrain.rightTalon2.configGetSupplyCurrentLimit(currentLimit, 500);
		Robot.drivetrain.leftTalon1.configGetSupplyCurrentLimit(currentLimit, 500);
        Robot.drivetrain.leftTalon3.configGetSupplyCurrentLimit(currentLimit, 500);

        Robot.navX.resetAngle();
    }

    @Override
    public void teleopPeriodic()
    {
        IO.UpdateControllers();
        drivetrain.sigmaDrive(IO.m_leftAnalogY, IO.m_rightAnalogY);
        IO.ProcessControllers();
    }

    @Override
    public void testInit() { }

    @Override
    public void testPeriodic()
    {
    }
}