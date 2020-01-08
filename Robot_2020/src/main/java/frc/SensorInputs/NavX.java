package frc.SensorInputs;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class NavX
{
    public AHRS AHRS;
    public double yaw, angle, pitch, roll;

    public NavX()
    {
        AHRS = new AHRS(SPI.Port.kMXP);
    }
    
    public void updateAHRS()
    {
        yaw = AHRS.getYaw();
        angle = AHRS.getAngle();
        pitch = AHRS.getPitch();
        roll = AHRS.getRoll();
    }

    public void resetAngle()
    {
        AHRS.zeroYaw();
    }

    public void testAngle()
    {
       // System.out.println("Yaw: " + yaw + " Yaw (Unclamped): " + angle + " Pitch: " + pitch + " Roll: " + roll);
    }
}