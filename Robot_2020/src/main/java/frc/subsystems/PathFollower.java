package frc.subsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import java.util.List;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Robot;

public class PathFollower
{
    public static final double ksVolts = 0.195;
    public static final double kvVoltSecondsPerMeter = 0.0486;
    public static final double kaVoltSecondsSquaredPerMeter = 0.00531;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 0.507;
    public static final double kTrackwidthMeters = 1.1232503390357302;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 8;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

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

    public PathSucker pathSucker = new PathSucker();

    public class PathSucker
    {
        public PathSucker()
        {
            var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(ksVolts,
                                        kvVoltSecondsPerMeter,
                                        kaVoltSecondsSquaredPerMeter),
                kDriveKinematics,
                10);

        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(kMaxSpeedMetersPerSecond,
                                kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(1, 1),
                new Translation2d(2, -1)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config
        );
        final Drivetrain m_robotDrive = new Drivetrain();

        RamseteCommand ramseteCommand = new RamseteCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            new RamseteController(kRamseteB, kRamseteZeta),
            new SimpleMotorFeedforward(ksVolts,
                                    kvVoltSecondsPerMeter,
                                    kaVoltSecondsSquaredPerMeter),
            kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(kPDriveVel, 0, 0),
            new PIDController(kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive);
        }
    }
}