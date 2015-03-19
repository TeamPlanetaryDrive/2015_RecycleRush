package org.usfirst.frc.team2856.robot;

import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.SpeedController;

public class Rollers {

	private SpeedController leftRoller, rightRoller, center;//center is a temporary name
	
	public Rollers(){
		leftRoller = new Jaguar(RobotConstants.ROLLERS_SC_LEFT_CHANNEL);
		rightRoller = new Jaguar(RobotConstants.ROLLERS_SC_RIGHT_CHANNEL);
		center = new Jaguar(RobotConstants.ROLLERS_SC_CENTER_CHANNEL);
				
	}
	
	public void setLeft(double eff){leftRoller.set(eff);}
	public void setRight(double eff){rightRoller.set(eff);}
	public void setCenter(double eff){center.set(eff);}
	
	
}

