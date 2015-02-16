package org.usfirst.frc.team2856.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class Lift {
	private NetworkTable table;
	private PowerDistributionPanel power;
	private Encoder encoder;
	private SpeedController motor;
	private PIDController pid;
	private DigitalInput lowerLimit;
	private DigitalInput upperLimit;
	
	private boolean moveActive;
	private MoveRefGen refGen;
	private double startPos;
	private double smallNumber;
    private boolean PIDon;
    
 	//move vars
 	private double distFromOrigin; //assuming origin is the bottom
 	private double totalDist; //inches

	public Lift() {
		table = NetworkTable.getTable(RobotConstants.NT_SOURCE);
		power = new PowerDistributionPanel();

		// PID controller gains will be updated prior to enabling the controllers
		encoder = new Encoder(RobotConstants.LIFT_ENC_CHANNEL_A, RobotConstants.LIFT_ENC_CHANNEL_B, false, EncodingType.k4X);
		motor = new Jaguar(RobotConstants.LIFT_SC_CHANNEL);
		pid = new PIDController(0, 0, 0, encoder, motor, RobotConstants.ARM_PID_PERIOD);
		lowerLimit = new DigitalInput(RobotConstants.LIFT_LIMIT_LOWER_CHANNEL);
		upperLimit = new DigitalInput(RobotConstants.LIFT_LIMIT_UPPER_CHANNEL);
		
		// Set encoder resolution
		encoder.setDistancePerPulse(RobotConstants.LIFT_ENC_RESOLUTION);

		// Set encoder samples to average
		encoder.setSamplesToAverage(RobotConstants.LIFT_ENC_SAMPLES_TO_AVERAGE);

		// Start encoder
		encoder.reset();

		// Set PID output range
		pid.setOutputRange (-RobotConstants.ARM_PID_EFFORT_MAX, RobotConstants.ARM_PID_EFFORT_MAX);

		// PID move initialization
		moveActive = false;
		refGen = new MoveRefGen();

		// Set initial network table values
		table.putNumber("Arm.AccelRate", RobotConstants.ARM_ACCEL_RATE);
		table.putNumber("Arm.MaxSpeed", RobotConstants.ARM_SPEED_MAX);
		table.putNumber("Arm.Pos.Kp", RobotConstants.ARM_PID_KP);
		table.putNumber("Arm.Pos.Ki", RobotConstants.ARM_PID_KI);
		table.putNumber("Arm.Pos.Kd", RobotConstants.ARM_PID_KD);
		
		// Set other parameters
		PIDon = false;
		distFromOrigin = 0;
		totalDist = RobotConstants.LIFT_HEIGHT;
	}

	public void setEffort(double effort) {
		if (!moveActive)
		{
//			if ((effort < 0 && !lowerLimit.get()) ||
//				(effort > 0 && !upperLimit.get())   )
//			{
				motor.set(effort);
//			}
//			else
//			{
//				motor.set(0);
//			}
		}
	}

	public void EncoderReset() {
        encoder.reset();
	}

	public boolean IsMoveActive() {
		return moveActive;
	}

	/*move to specific location*/
	public void moveToTop(){
		if(!upperLimit.get()){
			if(PIDon)
				PidMoveStart(totalDist - distFromOrigin);//find out distance
			else
				setEffort(RobotConstants.LIFT_HOMING_UP);
		}
	}

	public void moveToBottom(){
		if(!lowerLimit.get()){
			if(PIDon)
				PidMoveStart(-distFromOrigin);//find out distance
			else
				setEffort(RobotConstants.LIFT_HOMING_DOWN);
		}
	}

	public void PidMoveStart(double position) {
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
		pid.reset();

        // Set encoders to output position to PID controller
		encoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);

		// Set PID parameters
		pid.setPID(Kp, Ki, Kd);

		// Set PID set points to zero
		pid.setSetpoint(0);

		// Enable PID controllers
		pid.enable();

		// Configure and start move reference generator
		refGen.Configure(accelRate, maxSpeed, RobotConstants.DT_PID_POS_SETTLE);
		startPos = encoder.get();
		distance = position - startPos;
		refGen.Start(distance);
		moveActive = true;
	}

	public void PidStop() {
		// Disable PID controller
		pid.disable();

		// Position move finished
		moveActive = false;
	}

	public void Update(boolean debug) {
		smallNumber = (smallNumber == 0) ? 0.001: 0;
		
		if (moveActive)
		{
			refGen.Update();
			if (refGen.IsActive() && (
				(refGen.GetRefPosition() < 0 && !lowerLimit.get()) ||
				(refGen.GetRefPosition() > 0 && !upperLimit.get())	)
			   )
			{
				double refPos = refGen.GetRefPosition() + startPos;
				pid.setSetpoint(refPos);
				if (debug)
				{
					table.putNumber("Arm.PosR", refPos + smallNumber);
				}
			}
			else
			{
				PidStop();
			}
		}
//		else if ((motor.get() < 0 && lowerLimit.get()) ||
//				 (motor.get() > 0 && upperLimit.get())   )
//		{
//			motor.set(0);
//		}

		if (debug)
		{
			//table.putNumber("Arm.PosR", encoder.getDistance() + smallNumber);
			table.putNumber("Arm.Pos",  encoder.getDistance() + smallNumber);
			//table.putNumber("Arm.VelR", pid.getSetpoint() + smallNumber);
			table.putNumber("Arm.Vel",  encoder.getRate() + smallNumber);
			table.putNumber("Arm.Eff",  motor.get() + smallNumber);
			table.putNumber("Arm.Cur",  power.getCurrent(RobotConstants.LIFT_MOTOR_POWERPANEL_CHANNEL) + smallNumber);
			}
	}
}
