package org.usfirst.frc.team2856.robot;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;

public class InputManager extends AbstractInputManager_ {

	private RobotDrive drive;
	private Gyro gyro;
	
	private boolean useGyro;
	
	public InputManager(RobotDrive drive, Gyro gyro,
			Joystick left, Joystick right) {
		super(left, right); // left = 0, right = 1
		
		this.drive = drive;
		this.gyro = gyro;
		useGyro = true;
		
		this.addTriAxisAction(
				1, ATTACK3_AXIS_X, // right x
				1, ATTACK3_AXIS_Y, // right y
				0, ATTACK3_AXIS_X, // left x
				this::driveBot, true);
		this.addButtonAction(1, 1, this::setUseGyro, false); // right trigger
		this.addButtonAction(1, 2, gyro::reset, false); // right button 2
	}
	
	public void driveBot(double xvel, double yvel, double rotate) {
		drive.mecanumDrive_Cartesian(xvel, yvel,
				rotate, useGyro ? gyro.getAngle() : 0);
	}
	
	public void setUseGyro(boolean button) {
		useGyro = button;
	}
	
}
