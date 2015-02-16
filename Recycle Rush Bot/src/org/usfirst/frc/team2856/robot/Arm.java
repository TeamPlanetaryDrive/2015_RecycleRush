package org.usfirst.frc.team2856.robot;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.PIDController;
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
	private MoveRefGen leftRefGen;
	private boolean leftMoveActive;
	private double leftStartPos;
	private DigitalInput leftLimit;
	
	// Right arm
	private AnalogPotentiometer rightPot;
	private SpeedController rightMotor;
	private PIDController rightPID;
	private MoveRefGen rightRefGen;
	private boolean rightMoveActive;
	private double rightStartPos;
	private DigitalInput rightLimit;
	
	// Other variables
	private double smallNumber;

	public Arm() {
		table = NetworkTable.getTable(RobotConstants.NT_SOURCE);
		power = new PowerDistributionPanel();

		// PID controller gains will be updated prior to enabling the controllers

		// Left arm
		leftPot = new AnalogPotentiometer(RobotConstants.ARM_POT_LEFT_CHANNEL, RobotConstants.ARM_POT_LEFT_RANGE, RobotConstants.ARM_POT_LEFT_OFFSET);
		leftMotor = new Jaguar(RobotConstants.ARM_SC_LEFT_CHANNEL);
		leftPID = new PIDController(0, 0, 0, leftPot, leftMotor, RobotConstants.ARM_PID_PERIOD);
		leftRefGen = new MoveRefGen();
		leftMoveActive = false;
		leftLimit = new DigitalInput(RobotConstants.ARM_LIMIT_LEFT_CHANNEL);

		// Right arm
		rightPot = new AnalogPotentiometer(RobotConstants.ARM_POT_RIGHT_CHANNEL, RobotConstants.ARM_POT_RIGHT_RANGE, RobotConstants.ARM_POT_RIGHT_OFFSET);
		rightMotor = new Jaguar(RobotConstants.ARM_SC_RIGHT_CHANNEL);
		rightPID = new PIDController(0, 0, 0, rightPot, rightMotor, RobotConstants.ARM_PID_PERIOD);
		rightRefGen = new MoveRefGen();
		rightMoveActive = false;
		rightLimit = new DigitalInput(RobotConstants.ARM_LIMIT_RIGHT_CHANNEL);

		// Set PID output range
		leftPID.setOutputRange (-RobotConstants.ARM_PID_EFFORT_MAX, RobotConstants.ARM_PID_EFFORT_MAX);
		rightPID.setOutputRange  (-RobotConstants.ARM_PID_EFFORT_MAX, RobotConstants.ARM_PID_EFFORT_MAX);
		
		// Set initial network table values
		table.putNumber("Arm.AccelRate", RobotConstants.ARM_ACCEL_RATE);
		table.putNumber("Arm.MaxSpeed", RobotConstants.ARM_SPEED_MAX);
		table.putNumber("Arm.Kp", RobotConstants.ARM_PID_KP);
		table.putNumber("Arm.Ki", RobotConstants.ARM_PID_KI);
		table.putNumber("Arm.Kd", RobotConstants.ARM_PID_KD);
	}

	public void setEffort(double left, double right) {
		if (!leftMoveActive)
		{
			if (!(leftLimit.get() && left < 0))
			{
				leftMotor.set(left);
			}
		}
		if (!rightMoveActive)
		{
			if (!(rightLimit.get() && right < 0))
			{
				rightMotor.set(right);
			}
		}
	}

	public boolean IsLeftMoveActive() {
		return leftMoveActive;
	}

	public boolean IsRightMoveActive() {
		return rightMoveActive;
	}

	public void LeftPidMoveStart(double position) {
		double Kp, Ki, Kd;
		double accelRate;
		double distance;
		double maxSpeed;

		// Update local parameters
		Kp = table.getNumber("Arm.Kp");
		Ki = table.getNumber("Arm.Ki");
		Kd = table.getNumber("Arm.Kd");
		accelRate = table.getNumber("Arm.AccelRate");
		maxSpeed = table.getNumber("Arm.MaxSpeed");

		// Reset PID controller
		leftPID.reset();

		// Set PID parameters
		leftPID.setPID(Kp, Ki, Kd);

		// Set PID set point to zero
		leftPID.setSetpoint(0);

		// Enable PID controller
		leftPID.enable();

		// Configure and start move reference generator
		leftRefGen.Configure(accelRate, maxSpeed, RobotConstants.DT_PID_POS_SETTLE);
		leftStartPos = leftPot.get();
		distance = position - leftStartPos;
		leftRefGen.Start(distance);
		leftMoveActive = true;
	}

	public void RightPidMoveStart(double position) {
		double Kp, Ki, Kd;
		double accelRate;
		double distance;
		double maxSpeed;

		// Update local parameters
		Kp = table.getNumber("Arm.Kp");
		Ki = table.getNumber("Arm.Ki");
		Kd = table.getNumber("Arm.Kd");
		accelRate = table.getNumber("Arm.AccelRate");
		maxSpeed = table.getNumber("Arm.MaxSpeed");

		// Reset PID controllers
		rightPID.reset();

		// Set PID parameters
		rightPID.setPID(Kp, Ki, Kd);

		// Set PID set point to zero
		rightPID.setSetpoint(0);

		// Enable PID controller
		rightPID.enable();

		// Configure and start move reference generator
		rightRefGen.Configure(accelRate, maxSpeed, RobotConstants.DT_PID_POS_SETTLE);
		rightStartPos = rightPot.get();
		distance = position - rightStartPos;
		rightRefGen.Start(distance);
		rightMoveActive = true;
	}

	public void LeftPidStop() {
		// Disable PID controller
		leftPID.disable();

		// Position move finished
		leftMoveActive = false;
	}

	public void RightPidStop() {
		// Disable PID controller
		rightPID.disable();

		// Position move finished
		rightMoveActive = false;
	}

	public void Update(boolean debug) {
		smallNumber = (smallNumber == 0) ? 0.001: 0;
		
		if (leftMoveActive)
		{
			leftRefGen.Update();
			if (leftRefGen.IsActive() && !(leftLimit.get() && (leftRefGen.GetRefPosition() < 0)))
			{
				double refPos = leftRefGen.GetRefPosition() + leftStartPos;
				leftPID.setSetpoint(refPos);
				if (debug)
				{
					table.putNumber("ArmL.PosR", refPos + smallNumber);
				}
			}
			else
			{
				LeftPidStop();
			}
		}
		else if (leftLimit.get() && leftMotor.get() < 0)
		{
			leftMotor.set(0);
		}

		if (rightMoveActive)
		{
			rightRefGen.Update();
			if (rightRefGen.IsActive() && !(rightLimit.get() && (rightRefGen.GetRefPosition() < 0)))
			{
				double refPos = rightRefGen.GetRefPosition() + rightStartPos;
				rightPID.setSetpoint(refPos);
				if (debug)
				{
					table.putNumber("ArmR.PosR", refPos + smallNumber);
				}
			}
			else
			{
				RightPidStop();
			}
		}
		else if (rightLimit.get() && rightMotor.get() < 0)
		{
			rightMotor.set(0);
		}

		if (debug)
		{
			// Left arm
			//table.putNumber("ArmL.PosR", leftPot.get() + smallNumber);
			table.putNumber("ArmL.Pos",  leftPot.get() + smallNumber);
			table.putNumber("ArmL.Eff",  leftMotor.get() + smallNumber);
			table.putNumber("ArmL.Cur",  power.getCurrent(RobotConstants.ARM_CUR_LEFT_CHANNEL) + smallNumber);
	
			// Right arm
			//table.putNumber("ArmR.PosR", rightPot.get() + smallNumber);
			table.putNumber("FL.Pos",  rightPot.get() + smallNumber);
			table.putNumber("FL.Eff",  rightMotor.get() + smallNumber);
			table.putNumber("FL.Cur",  power.getCurrent(RobotConstants.ARM_CUR_RIGHT_CHANNEL) + smallNumber);
		}
	}
}
