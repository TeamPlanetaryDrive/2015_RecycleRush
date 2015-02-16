package org.usfirst.frc.team2856.robot;

import edu.wpi.first.wpilibj.Joystick;

public class TelopInputManager extends AbstractInputManager {
	private Arm arm;
	private Drive drive;
	private Lift lift;

	public TelopInputManager(Arm armIn, Drive driveIn, Lift liftIn) {
		super(new Joystick(RobotConstants.IM_JOYSTICK_LEFT),    // left = 0
				new Joystick(RobotConstants.IM_JOYSTICK_RIGHT), // right = 1
				new Joystick(RobotConstants.IM_XBOX_CONTROLLER)); // xbox = 2
		
		arm = armIn;
		drive = driveIn;
		lift = liftIn;
		
		this.addTriAxisAction(
				1, ATTACK3_AXIS_X, // right x
				1, ATTACK3_AXIS_Y, // right y
				0, ATTACK3_AXIS_X, // left x
				drive::RDrive, true);
		
		this.addButtonAction(1, 8, drive::GyroSetActive, false);   // left 8
		this.addButtonAction(1, 9, drive::GyroClearActive, false); // left 9
		this.addButtonAction(0, 3, drive::GyroReset, false);       // right 3
		
		this.addButtonAction(0, 6, drive::PidSpeedStart, false);   // left 6
		this.addButtonAction(0, 7, drive::PidStop, false);         // left 7
		this.addButtonAction(1, 11, drive::EncoderReset, false);   // right 11
		
		this.addBiAxisAction(
				2, XBOX_AXIS_LX, // xbox left x
				2, XBOX_AXIS_RX, // xbox right x
				this::armsSetEffort, false);
		this.addButtonAction(2, XBOX_BUTTON_A, // xbox A
				this::leftPIDMoveStart, false);
		this.addButtonAction(2, XBOX_BUTTON_B, // xbox B
				this::rightPIDMoveStart, false);
		this.addButtonAction(2, XBOX_BUTTON_X, // xbox X
				arm::LeftPidStop, false);
		this.addButtonAction(2, XBOX_BUTTON_Y, // xbox Y
				arm::RightPidStop, false);
		this.addAxisAction(2, 3, lift::setEffort, false);
	}
	
	private void armsSetEffort(double left, double right) {
		arm.setEffort(-left, right);
	}
	
	private void leftPIDMoveStart() {
		arm.LeftPidMoveStart(90);
	}
	
	private void rightPIDMoveStart() {
		arm.RightPidMoveStart(90);
	}
	
	
}
