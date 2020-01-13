package frc.subsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

import frc.robot.Robot;

public class PathFollower
{
    // Odometry class for tracking robot pose
    private DifferentialDriveOdometry m_odometry;

    public DifferentialDriveWheelSpeeds getWheelSpeeds() 
    {
        return new DifferentialDriveWheelSpeeds(Robot.drivetrain.getLeftEncoderVel(), Robot.drivetrain.getRightEncoderVel());
    }

    public void update()
    {
        m_odometry.update(Rotation2d.fromDegrees(Robot.navX.angle), Robot.drivetrain.getLeftEncoder(), Robot.drivetrain.getRightEncoder());
    }

    public Pose2d getPose() 
    {
        return m_odometry.getPoseMeters();
    }

    
}