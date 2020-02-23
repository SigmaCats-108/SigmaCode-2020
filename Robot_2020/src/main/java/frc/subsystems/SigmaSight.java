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

    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight-shooter");
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

    public int counter = 0;
    public boolean lineUpToShoot()
    {
        steering_adjust = turnKp * xVal;
        Robot.drivetrain.sigmaDrive(steering_adjust, -steering_adjust);
        if(Math.abs(xVal) < 2)
        {
            counter++;
        }
        return counter > 50;
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
      //  return 63181.81 -  45476.19 * Math.pow(Math.E, -0.00244242 * Math.abs(yVal));
        SmartDashboard.putNumber("desired speed", 17693.11 + 791.3032 * yVal + 125.3293 * Math.pow(yVal, 2)  + 5.391753 * Math.pow(yVal, 3));
        return 17693.11 + 791.3032 * yVal + 125.3293 * Math.pow(yVal, 2)  + 5.391753 * Math.pow(yVal, 3);
    }
    // y = 16316.36 + 1.560498e-10*e^(-2.183764*x) equation 1
    // y = 63181.81 - 45476.19*e^(-0.00244242*x) equation 2
    //y = 17866.53 + 339.3089*x - 16.45606*x^2 - 3.645881*x^3
    //y = 17768.27 + 321.3398*x - 4.327441*x^2 - 2.216103*x^3
    //y = 17693.11 + 791.3032*x + 125.3293*x^2 + 5.391753*x^3
    //y = 17693.11 + 791.3032*x + 125.3293*x^2 + 5.391753*x^3
    public boolean inRange()
    {
        return yVal > 10 && yVal < 30;
    }

    public void turnOffLights()
    {
        NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("ledMode").setNumber(1);
    }    
    
    public void turnOnLights()
    {
        NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("ledMode").setNumber(3);
    }
}