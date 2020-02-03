package frc.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class SigmaSight
{
    boolean validTarget;
    double xVal, yVal, area, skew;
    double steering_adjust, distance_adjust, left_command, right_command;
    double turnKp = RobotMap.HATCH_VISION_TURN_PGAIN;
    double distanceKp = RobotMap.HATCH_VISION_DISTANCE_PGAIN;
    double desiredArea = RobotMap.HATCH_VISION_DESIRED_TARGET_AREA;
    double minAimCommand = RobotMap.HATCH_VISION_MIN_AIM_COMMAND;
    
    private enum Direction {LEFT, RIGHT, OTHER};
    Direction targetDirection = Direction.LEFT;

    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv = limelightTable.getEntry("tv");
    NetworkTableEntry tx = limelightTable.getEntry("tx");
    NetworkTableEntry ty = limelightTable.getEntry("ty");
    NetworkTableEntry ta = limelightTable.getEntry("ta");
    NetworkTableEntry ts = limelightTable.getEntry("ts");

    /**
     * Updates the vision target's position periodically
     */
    public void updateValues()
    {
        validTarget = isValidTarget();
        xVal = tx.getDouble(0.0);
        yVal = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        skew = ts.getDouble(0.0);
        updateLastKnownDirection();
    }

    /**
     * Checks for a valid vision target
     * @return True if a target is found, and False if not
     */
    public boolean isValidTarget()
    {
        return !(xVal == 0.0 && yVal == 0.0);
    }

    /**
     * Records the current direction of the detected target for use in the event that
     * the target is lost and must be found again
     */
    public void updateLastKnownDirection()
    {
        if(xVal > 1)
        {
            targetDirection = Direction.RIGHT;
        }
        else if(xVal < -1)
        {
            targetDirection = Direction.LEFT;
        }
    }

    /**
     * Turns towards the last known target's direction if no valid target is found
     */
    public void seekTarget()
    {
        if (targetDirection == Direction.RIGHT)
        {
            Robot.drivetrain.sigmaDrive(0.5, -0.5);
        } 
        else if (targetDirection == Direction.LEFT)
        {
            Robot.drivetrain.sigmaDrive(-0.5, 0.5);
        }
    }
    
    /**
     * Will turn towards a detected target, slowing down as the turn error decreases
     */
    public void turnToTarget()
    {
        steering_adjust = turnKp * xVal;
        Robot.drivetrain.sigmaDrive(steering_adjust, -steering_adjust);
    }

    public boolean lineUpToShoot()
    {
        steering_adjust = turnKp * xVal;
        Robot.drivetrain.sigmaDrive(steering_adjust, -steering_adjust);
        return Math.abs(xVal) < 1;
    }
    
    /**
     * Turns towards the target and gets within range simultaniously
     * @return Once the robot is within the target zone, returns true
     */
    public boolean aimAndRange()
    {
        if (xVal > 1.0)
        {
            steering_adjust = turnKp * -xVal - minAimCommand;
        }
        else if (xVal < 1.0)
        {
            steering_adjust = turnKp * -xVal + minAimCommand;
        }

        distance_adjust = distanceKp * (desiredArea - area);

        left_command = steering_adjust + distance_adjust * -1;
        right_command = -steering_adjust + distance_adjust * -1;

        System.out.println("leftSpeed: " + left_command);
        Robot.drivetrain.sigmaDrive(left_command, right_command);

        return (area > desiredArea - 0.5 && area < desiredArea + 0.5 && xVal > -1.2 && xVal < 1.2);
    }

    /**
     * Prints out the detected object's position values to the dashboard
     */
    public void testValues()
    {
        SmartDashboard.putBoolean("tv", validTarget);
        SmartDashboard.putNumber("tx", xVal);
        SmartDashboard.putNumber("ty", yVal);
        SmartDashboard.putNumber("ta", area);
        SmartDashboard.putNumber("ts", skew);
    }

	public double desiredSpeed()
	{
        return 4698.373 +  0.00435206 * Math.pow(Math.E, 0.4609538 * Math.abs(yVal));
	}
}