package org.usfirst.frc.team2856.robot;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class Lift {
	private NetworkTable table;
    
	public Lift() {
		table = NetworkTable.getTable(RobotConstants.NT_SOURCE);
	}

}
