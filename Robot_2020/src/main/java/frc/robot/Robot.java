package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

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

    @Override
    public void robotInit() 
    {
        sigmaSight = new SigmaSight();
        drivetrain = new Drivetrain();
        navX = new NavX();
        wheelOfFortune = new ColorWheel();
        ballMech = new BallMech();
        climbMech = new ClimbMech();
    }

    @Override
    public void robotPeriodic()
    {
        sigmaSight.updateValues();
        sigmaSight.testValues();
        drivetrain.update();
        navX.updateAHRS();
        ballMech.update();
        wheelOfFortune.updateColors();
    }
    
    @Override
    public void autonomousInit() 
    {
  
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
        IO.ProcessControllers();
    }

    @Override
    public void testInit() { }

    @Override
    public void testPeriodic()
    {
    }
}