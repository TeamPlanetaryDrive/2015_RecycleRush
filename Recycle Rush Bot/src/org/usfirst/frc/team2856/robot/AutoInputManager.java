package org.usfirst.frc.team2856.robot;

public class AutoInputManager extends AbstractInputManager {
//	private Arm arm;
	private CounterWeight cWeight;
	private Drive drive;
	private Lift lift;
	private Pivot pivot;

	private int action;
	
	public AutoInputManager(/*Arm arm, */CounterWeight cWeight, Drive drive, Lift lift, Pivot pivot) {
//		this.arm = arm;
		this.cWeight = cWeight;
		this.drive = drive;
		this.lift = lift;
		this.pivot = pivot;
		
		action = 0;
	}

	@Override
	public void preUpdate() {
		if( /*!arm.IsLeftMoveActive() &&
			!arm.IsRightMoveActive() &&*/
			!cWeight.IsMoveActive() &&
			!drive.IsMoveActive() &&
			!lift.IsMoveActive() &&
			!pivot.IsMoveActive()) {
			switch(action++) {
				//case 0: drive.PidMoveStart(3); break;
				//case 1: drive.PidMoveStart(-3); break;
			}
		}
	}
	
}
