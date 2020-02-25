package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

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
    public static StatorCurrentLimitConfiguration currentLimit;

    @Override
    public void robotInit() 
    {
        currentLimit = new StatorCurrentLimitConfiguration(true, 38, 30, 0.5);
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
        navX.update();
        ballMech.update();
        wheelOfFortune.updateColors();
        climbMech.update();
        IO.update();
        sigmaSight.turnOnLights();
    }
    
    @Override
    public void autonomousInit() 
    {
        Robot.navX.resetAngle();
        Robot.drivetrain.autoState = 0;
        Robot.drivetrain.driveStraightState = 0;
    }
    // int autoState = 0;
    @Override
    public void autonomousPeriodic()
    {
        // Robot.sigmaSight.turnOnLights();
        // Robot.drivetrain.autoDrive();
        Robot.drivetrain.sixBallAuto(100, 10);
    }

    @Override
    public void teleopInit()
    {
        Robot.drivetrain.rightTalon1.configGetStatorCurrentLimit(currentLimit,1000);
		Robot.drivetrain.rightTalon2.configGetStatorCurrentLimit(currentLimit,1000);
		Robot.drivetrain.leftTalon1.configGetStatorCurrentLimit(currentLimit,1000);
        Robot.drivetrain.leftTalon3.configGetStatorCurrentLimit(currentLimit,1000);
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