package org.usfirst.frc.team2856.robot;

public class AutoInputManager extends AbstractInputManager {
	private Arm arm;
	private Drive drive;
	private Lift lift;

	public AutoInputManager(Arm armIn, Drive driveIn, Lift liftIn) {
		arm = armIn;
		drive = driveIn;
		lift = liftIn;
	}

}