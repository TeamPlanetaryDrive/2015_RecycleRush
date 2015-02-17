package org.usfirst.frc.team2856.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class Pivot {
	private NetworkTable table;
	private SpeedController motor;
	DigitalInput closedSwitch;

	public Pivot() {
		table = NetworkTable.getTable(RobotConstants.NT_SOURCE);
		motor = new Jaguar(RobotConstants.PIVOT_SC_CHANNEL);
		closedSwitch = new DigitalInput(RobotConstants.PIVOT_LIMIT_CHANNEL);
	}
	
	public boolean IsMoveActive() {
		return false;
	}
	
	public void Move(double input){
		//if closed, only allow to move down
		if(closedSwitch.get()){
			if(input > 0)//input wants to go down
				motor.set(input);
			else//input doesn't want to go down
				motor.set(0);
		} else{//is not closed
			motor.set(input);
		}
	}
	
	public void Update(){
		table.putBoolean("Closed", closedSwitch.get());
	}
}
