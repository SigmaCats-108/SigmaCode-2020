package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.subsystems.SigmaSight;
import frc.subsystems.ColorWheel;
import frc.subsystems.Drivetrain;
import frc.SensorInputs.NavX;
import frc.subsystems.BallMech;

public class Robot extends TimedRobot 
{
    public static SigmaSight sigmaSight;  
    public static Drivetrain drivetrain;
    public static NavX navX;
    // public static ColorWheel wheelOfFortune;
    public static BallMech ballMech;
    private Command m_autonomousCommand;

    @Override
    public void robotInit() 
    {
        sigmaSight = new SigmaSight();
        drivetrain = new Drivetrain();
        navX = new NavX();
        // wheelOfFortune = new ColorWheel();
        ballMech = new BallMech();
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
    }
    
    @Override
    public void autonomousInit() 
    {
        m_autonomousCommand = drivetrain.getAutonomousCommand();

        if (m_autonomousCommand != null)
        {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic()
    {
        
    }

    @Override
    public void teleopInit()
    {
        if (m_autonomousCommand != null)
        {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic()
    {
        IO.UpdateControllers();
        // drivetrain.sigmaDrive(IO.m_leftAnalogY, IO.m_rightAnalogY);
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