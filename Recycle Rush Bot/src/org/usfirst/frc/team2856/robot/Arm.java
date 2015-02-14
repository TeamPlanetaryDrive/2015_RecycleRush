package org.usfirst.frc.team2856.robot;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class Arm {
	private NetworkTable table;

	public Arm(NetworkTable tableIn) {
		table = tableIn;
	}
}
