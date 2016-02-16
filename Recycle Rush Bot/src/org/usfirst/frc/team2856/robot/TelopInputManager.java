package org.usfirst.frc.team2856.robot;

import edu.wpi.first.wpilibj.Joystick;

public class TelopInputManager extends AbstractInputManager {
	//private Arm arm;
	//private CounterWeight cWeight;
//	private Drive drive;
	private Lift lift;
	//private Pivot pivot;

	public TelopInputManager(/*Arm arm, */CounterWeight cWeight, Drive drive, Lift lift, Pivot pivot) {
		super(new Joystick(RobotConstants.IM_JOYSTICK_LEFT),    // left = 0
				new Joystick(RobotConstants.IM_JOYSTICK_RIGHT), // right = 1
				new Joystick(RobotConstants.IM_XBOX_CONTROLLER)); // xbox = 2
		
		//this.arm = arm;
		//this.cWeight = cWeight;
//		this.drive = drive;
		this.lift = lift;
		//this.pivot = pivot;
		
		this.addTriAxisAction(
				1, ATTACK3_AXIS_X, // right x
				1, ATTACK3_AXIS_Y, // right y
				0, ATTACK3_AXIS_X, // left x
				drive::RDrive, true);
		
		this.addButtonAction(0, 8, drive::GyroSetActive, false);   // left 8
		this.addButtonAction(0, 9, drive::GyroClearActive, false); // left 9
		this.addButtonAction(1, 3, drive::GyroReset, false);       // right 3
		
		this.addButtonAction(0, 6, drive::PidSpeedStart, false);   // left 6
		this.addButtonAction(0, 7, drive::PidStop, false);         // left 7
		this.addButtonAction(1, 11, drive::EncoderReset, false);   // right 11
		
		// FIXME
		//this.addButtonAction(1, 5, this::fwd2Rev, false);
		//this.addButtonAction(1, 4, this::back2Rev, false);
		
//		this.addBiAxisAction(
//				2, XBOX_AXIS_LX, // xbox left x
//				2, XBOX_AXIS_RX, // xbox right x
//				this::armsSetEffort, false);
//		this.addButtonAction(2, XBOX_BUTTON_A, // xbox A
//				this::leftPIDMoveStart, false);
//		this.addButtonAction(2, XBOX_BUTTON_B, // xbox B
//				this::rightPIDMoveStart, false);
//		this.addButtonAction(2, XBOX_BUTTON_X, // xbox X
//				arm::LeftPidStop, false);
//		this.addButtonAction(2, XBOX_BUTTON_Y, // xbox Y
//				arm::RightPidStop, false);
		this.addBiAxisAction(
				2, XBOX_AXIS_LTRIGGER,
				2, XBOX_AXIS_RTRIGGER,
				this::liftEffort, false);
		this.addButtonAction(2, XBOX_BUTTON_LBUMP, liftMove(12), false);
		this.addButtonAction(2, XBOX_BUTTON_RBUMP, liftMove(24), false);
		this.addButtonAction(2, XBOX_BUTTON_BACK, lift::PidStop, false);
	}
	
//	private void armsSetEffort(double left, double right) {
//		arm.setEffort(-left, right);
//	}
//	
//	private void leftPIDMoveStart() {
//		arm.LeftPidMoveStart(90);
//	}
//	
//	private void rightPIDMoveStart() {
//		arm.RightPidMoveStart(90);
//	}
	
//	private void fwd2Rev() {
//		drive.PidMoveStart(2);
//	}
	
//	private void back2Rev() {
//		drive.PidMoveStart(-2);
//	}
	
	private Runnable liftMove(int num) {
		return () -> { lift.PidMoveStart(num); };
	}
	
	private void liftEffort(double left, double right) {
		lift.setEffort(right - left);
	}
}
