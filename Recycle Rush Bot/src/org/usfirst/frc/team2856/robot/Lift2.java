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

public class Lift2 {
	private NetworkTable table;
	private PowerDistributionPanel power;
	private Encoder encoder;
	private SpeedController motor;
	private PIDController pid;
	private DigitalInput lowerSwitch;
	private DigitalInput upperSwitch;
	
	private boolean moveActive;
	private MoveRefGen refGen;
	private double startPos;
	private double smallNumber;

	public Lift2() {
		table = NetworkTable.getTable(RobotConstants.NT_SOURCE);
		power = new PowerDistributionPanel();

		// PID controller gains will be updated prior to enabling the controllers
		encoder = new Encoder(RobotConstants.LIFT_ENC_CHANNEL_A, RobotConstants.LIFT_ENC_CHANNEL_B, false, EncodingType.k4X);
		motor = new Jaguar(RobotConstants.DT_SC_FRONTLEFT_CHANNEL);
		pid = new PIDController(0, 0, 0, encoder, motor, RobotConstants.DT_PID_PERIOD);
		lowerSwitch = new DigitalInput(RobotConstants.LIFT_LIMIT_LOWER_CHANNEL);
		upperSwitch = new DigitalInput(RobotConstants.LIFT_LIMIT_UPPER_CHANNEL);
		
		// Set encoder resolution
		encoder.setDistancePerPulse(RobotConstants.DT_ENC_RESOLUTION);

		// Set encoder samples to average
		encoder.setSamplesToAverage(RobotConstants.DT_ENC_SAMPLES_TO_AVERAGE);

		// Start encoder
		encoder.reset();

		// Set PID output range
		pid.setOutputRange (-RobotConstants.DT_PID_EFFORT_MAX, RobotConstants.DT_PID_EFFORT_MAX);

		// PID move initialization
		moveActive = false;
		refGen = new MoveRefGen();

		// Set initial network table values
		table.putNumber("DT.AccelRate", RobotConstants.DT_ACCEL_RATE);
		table.putNumber("DT.MaxSpeed", RobotConstants.DT_SPEED_MAX);
		table.putNumber("DT.Pos.Kp", RobotConstants.DT_PID_POSITION_KP);
		table.putNumber("DT.Pos.Ki", RobotConstants.DT_PID_POSITION_KI);
		table.putNumber("DT.Pos.Kd", RobotConstants.DT_PID_POSITION_KD);
	}

//	public void RDrive(double x, double y, double rotation) {
//		double gyroAngle;
//
//		if (!moveActive)
//		{
//			gyroAngle = gyroActive ? gyro.getAngle() : 0;
//			rDrive.mecanumDrive_Cartesian(x, y, rotation, gyroAngle);
//		}
//	}

	public void EncoderReset() {
        encoder.reset();
	}

	public boolean IsMoveActive() {
		return moveActive;
	}

	public void PidMoveStart(double distance) {
		double Kp, Ki, Kd;
		double accelRate;
		double maxSpeed;

		// Update local parameters
		Kp = table.getNumber("DT.Pos.Kp");
		Ki = table.getNumber("DT.Pos.Ki");
		Kd = table.getNumber("DT.Pos.Kd");
		accelRate = table.getNumber("DT.AccelRate");
		maxSpeed = table.getNumber("DT.MaxSpeed");

		// Reset PID controller
		pid.reset();

		// Reset encoder
        encoder.reset();

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
			if (refGen.IsActive())
			{
				double refPos = refGen.GetRefPosition();
				pid.setSetpoint(refPos);
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
			//table.putNumber("FL.PosR", encoder.getDistance() + smallNumber);
			table.putNumber("FL.Pos",  encoder.getDistance() + smallNumber);
			table.putNumber("FL.VelR", pid.getSetpoint() + smallNumber);
			table.putNumber("FL.Vel",  encoder.getRate() + smallNumber);
			table.putNumber("FL.Eff",  motor.get() + smallNumber);
			table.putNumber("FL.Cur",  power.getCurrent(RobotConstants.DT_CUR_FRONTLEFT_CHANNEL) + smallNumber);
			}
	}
}
