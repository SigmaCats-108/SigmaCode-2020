package frc.subsystems;

import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.IO;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ColorWheel
{
	private CANSparkMax WOFmotor = new CANSparkMax(57, MotorType.kBrushless);
	private CANEncoder WOFencoder = WOFmotor.getEncoder();
	// private DoubleSolenoid WOFCylinder = new DoubleSolenoid(0,1);

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

	// private void WOFCylinder()
	// {
	// 	if(WOFCylinder.get() == Value.kForward)
	// 	{
	// 		WOFCylinder.set(Value.kReverse);
	// 	}
	// 	else
	// 	{
	// 		WOFCylinder.set(Value.kForward);
	// 	}
	// }

	public void rotationControl(int position)
	{
		if(WOFencoder.getPosition() < position)
		{
			WOFmotor.set(1);
		}
		else
		{
			WOFmotor.set(0);
			IO.mainController.setRumble(RumbleType.kLeftRumble, 1.0);
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
				WOFmotor.set(1.0);
				System.out.println("Blue");

			}
			else
			{
				WOFmotor.set(0);
				IO.mainController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
			}
			break;
			case 'G' :
			if(match.color != kGreenTarget)
			{
				WOFmotor.set(1.0);
				System.out.println("Green");
			}
			else
			{
				WOFmotor.set(0);
				IO.mainController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
			}
			break;
			case 'R' :
			if(match.color != kRedTarget)
			{
				WOFmotor.set(1.0);
				System.out.println("Red");
			}
			else
			{
				WOFmotor.set(0);
				IO.mainController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
			}
			break;
			case 'Y' :
			if(match.color != kYellowTarget)
			{
				WOFmotor.set(1.0);
				System.out.println("Yellow");
			}
			else
			{
				WOFmotor.set(0);
				IO.mainController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
			}
			break;	
		}
	}
}