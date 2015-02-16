package org.usfirst.frc.team2856.robot;

public class AutoInputManager extends AbstractInputManager {
	public static final int
		STATE_IDLE = 0,
		STATE_ACTIVE = 1;
	
	private Arm arm;
	private Drive drive;
	private Lift lift;

	private int state;
	private int action;
	
	public AutoInputManager(Arm armIn, Drive driveIn, Lift liftIn) {
		arm = armIn;
		drive = driveIn;
		lift = liftIn;
		
		state = STATE_IDLE;
		action = 0;
	}

	@Override
	public void preUpdate() {
		if(drive.IsMoveActive())
			state = STATE_ACTIVE;
		else
			state = STATE_IDLE;
		
		if(state == STATE_IDLE) {
			switch(action++) {
			case 0: drive.PidMoveStart(2); break;
			case 1: drive.PidMoveStart(-2); break;
			}
		}
	}
	
}
