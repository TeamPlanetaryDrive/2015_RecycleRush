package org.usfirst.frc.team2856.robot;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class Arm {
	private NetworkTable table;
	private PowerDistributionPanel power;
    
	// Left arm
	private AnalogPotentiometer leftPot;
	private SpeedController leftMotor;
	private PIDController leftPID;
	private PidSpeedController leftPidSpeed;

	// Right arm
	private AnalogPotentiometer rightPot;
	private SpeedController rightMotor;
	private PIDController rightPID;
	private PidSpeedController rightPidSpeed;

	// Other variables
	private boolean moveActive;
	private MoveRefGen refGen;
	private double smallNumber;

	public Arm() {
		table = NetworkTable.getTable(RobotConstants.NT_SOURCE);
		power = new PowerDistributionPanel();

		// PID controller gains will be updated prior to enabling the controllers

		// Front Left Wheel
		leftPot = new AnalogPotentiometer(RobotConstants.ARM_POT_LEFT_CHANNEL, RobotConstants.ARM_POT_LEFT_RANGE, RobotConstants.ARM_POT_LEFT_OFFSET);
		leftMotor = new Jaguar(RobotConstants.ARM_SC_LEFT_CHANNEL);
		leftPID = new PIDController(0, 0, 0, leftPot, leftMotor, RobotConstants.ARM_PID_PERIOD);
		leftPidSpeed = new PidSpeedController(leftPID, leftMotor);

		// Rear Left Wheel
		rightPot = new AnalogPotentiometer(RobotConstants.ARM_POT_RIGHT_CHANNEL, RobotConstants.ARM_POT_RIGHT_RANGE, RobotConstants.ARM_POT_RIGHT_OFFSET);
		rightMotor = new Jaguar(RobotConstants.ARM_SC_RIGHT_CHANNEL);
		rightPID = new PIDController(0, 0, 0, rightPot, rightMotor, RobotConstants.ARM_PID_PERIOD);
		rightPidSpeed = new PidSpeedController(rightPID, rightMotor);

		// Set PID output range
		leftPID.setOutputRange (-RobotConstants.ARM_PID_EFFORT_MAX, RobotConstants.ARM_PID_EFFORT_MAX);
		rightPID.setOutputRange  (-RobotConstants.ARM_PID_EFFORT_MAX, RobotConstants.ARM_PID_EFFORT_MAX);
		
		// PID move initialization
		moveActive = false;
		refGen = new MoveRefGen();

		// Set initial network table values
		table.putNumber("Arm.AccelRate", RobotConstants.ARM_ACCEL_RATE);
		table.putNumber("Arm.MaxSpeed", RobotConstants.ARM_SPEED_MAX);
		table.putNumber("Arm.Kp", RobotConstants.ARM_PID_KP);
		table.putNumber("Arm.Ki", RobotConstants.ARM_PID_KI);
		table.putNumber("Arm.Kd", RobotConstants.ARM_PID_KD);
	}

	public void setSpeed(double left, double right) {
		if (!moveActive)
		{
			leftPidSpeed.set(left);
			rightPidSpeed.set(right);
		}
	}

	public boolean IsMoveActive() {
		return moveActive;
	}

	public void PidMoveStart(double distance) {
		double Kp, Ki, Kd;
		double accelRate;
		double maxSpeed;

		// Update local parameters
		Kp = table.getNumber("Arm.Kp");
		Ki = table.getNumber("Arm.Ki");
		Kd = table.getNumber("Arm.Kd");
		accelRate = table.getNumber("Arm.AccelRate");
		maxSpeed = table.getNumber("Arm.MaxSpeed");

		// Reset PID controllers
		leftPID.reset();
		rightPID.reset();

		// Set PID parameters
		frontLeftPID.setPID(Kp, Ki, Kd);
		rearLeftPID.setPID(Kp, Ki, Kd);
		frontRightPID.setPID(Kp, Ki, Kd);
		rearRightPID.setPID(Kp, Ki, Kd);

		// Set PID set points to zero
		frontLeftPID.setSetpoint(0);
		rearLeftPID.setSetpoint(0);
		frontRightPID.setSetpoint(0);
		rearRightPID.setSetpoint(0);

		// Disable user watchdog
		rDrive.setSafetyEnabled(false);

		// Enable PID controllers
		frontLeftPID.enable();
		rearLeftPID.enable();
		frontRightPID.enable();
		rearRightPID.enable();

		// Configure and start move reference generator
		refGen.Configure(accelRate, maxSpeed, RobotConstants.DT_PID_POS_SETTLE);
		refGen.Start(distance);
	}

	public void PidSpeedStart() {
		double Kp, Ki, Kd;
		double maxSpeed;
		
		// Update local parameters
		Kp = table.getNumber("DT.Spd.Kp");
		Ki = table.getNumber("DT.Spd.Ki");
		Kd = table.getNumber("DT.Spd.Kd");
		maxSpeed = table.getNumber("DT.MaxSpeed");

		// Reset PID controllers
		frontLeftPID.reset();
		rearLeftPID.reset();
		frontRightPID.reset();
		rearRightPID.reset();

		// Set encoders to output speed to PID controllers
		frontLeftEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate);
		rearLeftEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate);
		frontRightEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate);
		rearRightEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate);

		// Set maximum speed
		frontLeftPidSpeed.setMaxSpeed(maxSpeed);
		rearLeftPidSpeed.setMaxSpeed(maxSpeed);
		frontRightPidSpeed.setMaxSpeed(maxSpeed);
		rearRightPidSpeed.setMaxSpeed(maxSpeed);

		// Set PID parameters
		frontLeftPID.setPID(Kp, Ki, Kd);
		rearLeftPID.setPID(Kp, Ki, Kd);
		frontRightPID.setPID(Kp, Ki, Kd);
		rearRightPID.setPID(Kp, Ki, Kd);

		// Set PID set points to zero
		frontLeftPID.setSetpoint(0);
		rearLeftPID.setSetpoint(0);
		frontRightPID.setSetpoint(0);
		rearRightPID.setSetpoint(0);

		// Enable PID controllers
		frontLeftPID.enable();
		rearLeftPID.enable();
		frontRightPID.enable();
		rearRightPID.enable();
		
		// Position move active
		moveActive = true;
	}

	public void PidStop() {
		// Disable PID controllers
		frontLeftPID.disable();
		rearLeftPID.disable();
		frontRightPID.disable();
		rearRightPID.disable();

		// enable user watchdog
		rDrive.setSafetyEnabled(true); 

		// Position move finished
		moveActive = false;
	}

	public void Update(boolean debug) {
		smallNumber = (smallNumber == 0) ? 0.001: 0;
		
		if (moveActive)
		{
			refGen.Update();
			if (refGen.IsActive())
			{
				double refPos = refGen.GetRefPosition();
				frontLeftPID.setSetpoint(refPos);
				rearLeftPID.setSetpoint(refPos);
				frontRightPID.setSetpoint(refPos);
				rearRightPID.setSetpoint(refPos);
				if (debug)
				{
					table.putNumber("DT.PosR", refPos + smallNumber);
				}
			}
			else
			{
				PidStop();
			}
		}

		if (debug)
		{
			// Front Left Wheel
			//table.putNumber("FL.PosR", frontLeftEncoder.getDistance() + smallNumber);
			table.putNumber("FL.Pos",  frontLeftEncoder.getDistance() + smallNumber);
			table.putNumber("FL.VelR", frontLeftPID.getSetpoint() + smallNumber);
			table.putNumber("FL.Vel",  frontLeftEncoder.getRate() + smallNumber);
			table.putNumber("FL.Eff",  frontLeftMotor.get() + smallNumber);
			table.putNumber("FL.Cur",  power.getCurrent(RobotConstants.DT_CUR_FRONTLEFT_CHANNEL) + smallNumber);
	
			// Rear Left Wheel
			//table.putNumber("RL.PosR", rearLeftEncoder.getDistance() + smallNumber);
			table.putNumber("RL.Pos",  rearLeftEncoder.getDistance() + smallNumber);
			table.putNumber("RL.VelR", rearLeftPID.getSetpoint() + smallNumber);
			table.putNumber("RL.Vel",  rearLeftEncoder.getRate() + smallNumber);
			table.putNumber("RL.Eff",  rearLeftMotor.get() + smallNumber);
			table.putNumber("RL.Cur",  power.getCurrent(RobotConstants.DT_CUR_REARLEFT_CHANNEL) + smallNumber);
	
			// Front Right Wheel
			//table.putNumber("FR.PosR", frontRightEncoder.getDistance() + smallNumber);
			table.putNumber("FR.Pos",  frontRightEncoder.getDistance() + smallNumber);
			table.putNumber("FR.VelR", frontRightPID.getSetpoint() + smallNumber);
			table.putNumber("FR.Vel",  frontRightEncoder.getRate() + smallNumber);
			table.putNumber("FR.Eff",  frontRightMotor.get() + smallNumber);
			table.putNumber("FR.Cur",  power.getCurrent(RobotConstants.DT_CUR_FRONTRIGHT_CHANNEL) + smallNumber);
	
			// Rear Right Wheel
			//table.putNumber("RR.PosR", rearRightEncoder.getDistance() + smallNumber);
			table.putNumber("RR.Pos",  rearRightEncoder.getDistance() + smallNumber);
			table.putNumber("RR.VelR", rearRightPID.getSetpoint() + smallNumber);
			table.putNumber("RR.Vel",  rearRightEncoder.getRate() + smallNumber);
			table.putNumber("RR.Eff",  rearRightMotor.get() + smallNumber);
			table.putNumber("RR.Cur",  power.getCurrent(RobotConstants.DT_CUR_REARRIGHT_CHANNEL) + smallNumber);
		}
	}
}
