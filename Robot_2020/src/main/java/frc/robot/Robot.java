package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.subsystems.SigmaSight;
import frc.subsystems.ColorWheel;
import frc.subsystems.Drivetrain;
import frc.SensorInputs.NavX;
import frc.subsystems.BallMech;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot 
{
    public static SigmaSight sigmaSight;  
    public static Drivetrain drivetrain;
    public static NavX navX;
    public static ColorWheel wheelOfFortune;
    public static BallMech ballMech;
    private Command m_autonomousCommand;

	SendableChooser<String> m_chooser;
	String kAutoNameDefault = "!Do Nothing!";
	String autoLine = "AutoLine";
	String leftAuto = "Left Auto";
	String rightAuto = "Right Auto";
	String centerAuto = "Center Auto";
	String m_autoSelected;

    @Override
    public void robotInit() 
    {
        sigmaSight = new SigmaSight();
        drivetrain = new Drivetrain();
        navX = new NavX();
        wheelOfFortune = new ColorWheel();
        ballMech = new BallMech();

        m_chooser.setDefaultOption(kAutoNameDefault, kAutoNameDefault);
		m_chooser.addOption(autoLine, autoLine);
		m_chooser.addOption(leftAuto, leftAuto);
		m_chooser.addOption(centerAuto, centerAuto);
		m_chooser.addOption(rightAuto, rightAuto);
		SmartDashboard.putData("Auto Modes", m_chooser);
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();

        sigmaSight.updateValues();
        sigmaSight.testValues();
        drivetrain.update();
        navX.updateAHRS();
        ballMech.update();
        IO.update();
    }
    
    @Override
    public void autonomousInit() 
    {
        m_autoSelected = m_chooser.getSelected();
    }

    @Override
    public void autonomousPeriodic()
    {
        
    }

    @Override
    public void teleopInit()
    {
    }

    @Override
    public void teleopPeriodic()
    {
        IO.UpdateControllers();
        drivetrain.sigmaDrive(IO.m_leftAnalogY, IO.m_rightAnalogY);
        // ballMech.talonTest(IO.m_rightAnalogY);
        IO.ProcessControllers();
      
    }

    @Override
    public void testInit() { }

    @Override
    public void testPeriodic()
    {
    }
}