package frc.subsystems;

import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.IO;

public class ColorWheel
{
	private static final I2C.Port i2cPort = I2C.Port.kOnboard;
	private static final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
	private final ColorMatch m_colorMatcher = new ColorMatch();
	private final Color kBlueTarget = ColorMatch.makeColor(0.127, 0.424, 0.444);
    private final Color kGreenTarget = ColorMatch.makeColor(0.166, 0.577, 0.257);
    private final Color kRedTarget = ColorMatch.makeColor(0.520, 0.346, 0.133);
    private final Color kYellowTarget = ColorMatch.makeColor(0.318, 0.357, 0.125);
	
	public ColorWheel()
	{
		m_colorMatcher.addColorMatch(kBlueTarget);
		m_colorMatcher.addColorMatch(kGreenTarget);
		m_colorMatcher.addColorMatch(kRedTarget);
		m_colorMatcher.addColorMatch(kYellowTarget); 
	}

	public void updateColors()
	{
		Color detectedColor = colorSensor.getColor();
		SmartDashboard.putNumber("Red", detectedColor.red);
		SmartDashboard.putNumber("Green", detectedColor.green);
		SmartDashboard.putNumber("Blue", detectedColor.blue);
	}

	public void ruvib()
	{
		Color detectedColor = colorSensor.getColor();
		ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
		if(match.color == kBlueTarget)
		{
			System.out.println("Blue");
		}
		else if (match.color == kGreenTarget)
		{
			System.out.println("Green");
		}
		else if (match.color == kRedTarget)
		{
			System.out.println("Red");
		}
		else if (match.color == kYellowTarget)
		{
			System.out.println("Yellow");
		}
		else
		{
			System.out.println("Nothing");
		}
	}

	public void positionControl()
	{
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		Color detectedColor = colorSensor.getColor();
		ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
		switch(gameData.charAt(0))
		{
			case 'B' :
			if(match.color != kBlueTarget)
			{
				//move motors
				System.out.println("Blue");
			}
			else
			{
				//stop motors
				IO.mainController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
			}
			break;
			case 'G' :
			if(match.color != kGreenTarget)
			{
				//move motors
				System.out.println("Green");
			}
			else
			{
				//stop motors
				IO.mainController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
			}
			break;
			case 'R' :
			if(match.color != kRedTarget)
			{
				//move motors
				System.out.println("Red");
			}
			else
			{
				//stop motors
				IO.mainController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
			}
			break;
			case 'Y' :
			if(match.color != kYellowTarget)
			{
				//move motors
				System.out.println("Yellow");
			}
			else
			{
				//stop motors
				IO.mainController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
			}
			break;	
		}
	}
}