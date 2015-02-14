package org.usfirst.frc.team2856.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;

public class PidSpeedController implements SpeedController {
	private final PIDController pidInstance;
	private final SpeedController motorInstance;
	private double maxSpeed;

	public PidSpeedController(PIDController pid, SpeedController motor) {
		pidInstance = pid;
		motorInstance = motor;
		maxSpeed = 1.0;
	}

	public double get() {
		return motorInstance.get();
	}

	public void set(double effort, byte syncGroup) {
		set(effort);
	}

	public void set(double effort) {
		if (pidInstance.isEnable())
		{
			pidInstance.setSetpoint(effort * maxSpeed);
		}
		else
		{
			motorInstance.set(effort);
		}
	}

	public void disable() {
		if (pidInstance.isEnable())
		{
			pidInstance.disable();
		}
		motorInstance.disable();
	}

	public void pidWrite(double output) {
		motorInstance.pidWrite(output);
	}

	public double getMaxSpeed() {
		return maxSpeed;
	}

	public void setMaxSpeed(double speed) {
		maxSpeed = speed;
	}
}
