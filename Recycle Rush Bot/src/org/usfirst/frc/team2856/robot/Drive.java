package org.usfirst.frc.team2856.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class Drive {
	private NetworkTable table;
	private PowerDistributionPanel power;
    
	// Front Left Wheel
	private Encoder frontLeftEncoder;
	private SpeedController frontLeftMotor;
	private PIDController frontLeftPID;
	private PidSpeedController frontLeftPidSpeed;

	// Rear Left Wheel
	private Encoder rearLeftEncoder;
	private SpeedController rearLeftMotor;
	private PIDController rearLeftPID;
	private PidSpeedController rearLeftPidSpeed;

	// Front Right Wheel
	private Encoder frontRightEncoder;
	private SpeedController frontRightMotor;
	private PIDController frontRightPID;
	private PidSpeedController frontRightPidSpeed;

	// Rear Right Wheel
	private Encoder rearRightEncoder;
	private SpeedController rearRightMotor;
	private PIDController rearRightPID;
	private PidSpeedController rearRightPidSpeed;

	// Drive Train
	private RobotDrive rDrive;
	
	// Gyro
	private Gyro gyro;
	private boolean gyroActive;

	// Other variables
	private boolean moveActive;
	private MoveRefGen refGen;
	private double smallNumber;

	public Drive() {
		table = NetworkTable.getTable(RobotConstants.NT_SOURCE);
		power = new PowerDistributionPanel();

		// PID controller gains will be updated prior to enabling the controllers

		// Front Left Wheel
		frontLeftEncoder = new Encoder(RobotConstants.DT_ENC_FRONTLEFT_CHANNEL_A, RobotConstants.DT_ENC_FRONTLEFT_CHANNEL_B, false, EncodingType.k4X);
		frontLeftMotor = new Talon(RobotConstants.DT_SC_FRONTLEFT_CHANNEL);
		frontLeftPID = new PIDController(0, 0, 0, frontLeftEncoder, frontLeftMotor, RobotConstants.DT_PID_PERIOD);
		frontLeftPidSpeed = new PidSpeedController(frontLeftPID, frontLeftMotor);

		// Rear Left Wheel
		rearLeftEncoder = new Encoder(RobotConstants.DT_ENC_REARLEFT_CHANNEL_A, RobotConstants.DT_ENC_REARLEFT_CHANNEL_B, false, EncodingType.k4X);
		rearLeftMotor = new Talon(RobotConstants.DT_SC_REARLEFT_CHANNEL);
		rearLeftPID = new PIDController(0, 0, 0, rearLeftEncoder, rearLeftMotor, RobotConstants.DT_PID_PERIOD);
		rearLeftPidSpeed = new PidSpeedController(rearLeftPID, rearLeftMotor);

		// Front Right Wheel
		frontRightEncoder = new Encoder(RobotConstants.DT_ENC_FRONTRIGHT_CHANNEL_A, RobotConstants.DT_ENC_FRONTRIGHT_CHANNEL_B, true, EncodingType.k4X);
		frontRightMotor = new Talon(RobotConstants.DT_SC_FRONTRIGHT_CHANNEL);
		frontRightPID = new PIDController(0, 0, 0, frontRightEncoder, frontRightMotor, RobotConstants.DT_PID_PERIOD);
		frontRightPidSpeed = new PidSpeedController(frontRightPID, frontRightMotor);

		// Rear Right Wheel
		rearRightEncoder = new Encoder(RobotConstants.DT_ENC_REARRIGHT_CHANNEL_A, RobotConstants.DT_ENC_REARRIGHT_CHANNEL_B, true, EncodingType.k4X);
		rearRightMotor = new Talon(RobotConstants.DT_SC_REARRIGHT_CHANNEL);
		rearRightPID = new PIDController(0, 0, 0, rearRightEncoder, rearRightMotor, RobotConstants.DT_PID_PERIOD);
		rearRightPidSpeed = new PidSpeedController(rearRightPID, rearRightMotor);

		// Drive Train
		rDrive = new RobotDrive(frontLeftPidSpeed, rearLeftPidSpeed, frontRightPidSpeed, rearRightPidSpeed);
		
		// Gyro
		gyro = new Gyro(RobotConstants.DT_GYRO_CHANNEL);
		gyroActive = false;
		
		// Set encoder resolution
		frontLeftEncoder.setDistancePerPulse(RobotConstants.DT_ENC_RESOLUTION);
		rearLeftEncoder.setDistancePerPulse(RobotConstants.DT_ENC_RESOLUTION);
		frontRightEncoder.setDistancePerPulse(RobotConstants.DT_ENC_RESOLUTION);
		rearRightEncoder.setDistancePerPulse(RobotConstants.DT_ENC_RESOLUTION);

		// Set encoder samples to average
		frontLeftEncoder.setSamplesToAverage(RobotConstants.DT_ENC_SAMPLES_TO_AVERAGE);
		rearLeftEncoder.setSamplesToAverage(RobotConstants.DT_ENC_SAMPLES_TO_AVERAGE);
		frontRightEncoder.setSamplesToAverage(RobotConstants.DT_ENC_SAMPLES_TO_AVERAGE);
		rearRightEncoder.setSamplesToAverage(RobotConstants.DT_ENC_SAMPLES_TO_AVERAGE);

		// Start encoders
		frontLeftEncoder.reset();
		rearLeftEncoder.reset();
		frontRightEncoder.reset();
		rearRightEncoder.reset();

		// Set PID output range
		frontLeftPID.setOutputRange (-RobotConstants.DT_PID_EFFORT_MAX, RobotConstants.DT_PID_EFFORT_MAX);
		rearLeftPID.setOutputRange  (-RobotConstants.DT_PID_EFFORT_MAX, RobotConstants.DT_PID_EFFORT_MAX);
		frontRightPID.setOutputRange(-RobotConstants.DT_PID_EFFORT_MAX, RobotConstants.DT_PID_EFFORT_MAX);
		rearRightPID.setOutputRange (-RobotConstants.DT_PID_EFFORT_MAX, RobotConstants.DT_PID_EFFORT_MAX);

		// Start gyro
		gyro.reset();
		
		// PID move initialization
		moveActive = false;
		refGen = new MoveRefGen();

		// Set initial network table values
		table.putNumber("DT.AccelRate", RobotConstants.DT_ACCEL_RATE);
		table.putNumber("DT.MaxSpeed", RobotConstants.DT_SPEED_MAX);
		table.putNumber("DT.Pos.Kp", RobotConstants.DT_PID_POSITION_KP);
		table.putNumber("DT.Pos.Ki", RobotConstants.DT_PID_POSITION_KI);
		table.putNumber("DT.Pos.Kd", RobotConstants.DT_PID_POSITION_KD);
		table.putNumber("DT.Spd.Kp", RobotConstants.DT_PID_SPEED_KP);
		table.putNumber("DT.Spd.Ki", RobotConstants.DT_PID_SPEED_KI);
		table.putNumber("DT.Spd.Kd", RobotConstants.DT_PID_SPEED_KD);
	}

	public void RDrive(double x, double y, double rotation) {
		double gyroAngle;
		
		gyroAngle = gyroActive ? gyro.getAngle() : 0;
		rDrive.mecanumDrive_Cartesian(x, y, rotation, gyroAngle);
	}

	public void EncoderReset() {
        frontLeftEncoder.reset();
        rearLeftEncoder.reset();
        frontRightEncoder.reset();
        rearRightEncoder.reset();
	}

	public void GyroClearActive() {
		gyroActive = false;
	}

	public double GyroGetAngle() {
		return gyro.getAngle();
	}

	public double GyroGetRate() {
		return gyro.getRate();
	}

	public boolean GyroIsActive() {
		return gyroActive;
	}
	
	public void GyroReset() {
		gyro.reset();
	}

	public void GyroSetActive() {
		gyroActive = true;
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

		// Reset PID controllers
		frontLeftPID.reset();
		rearLeftPID.reset();
		frontRightPID.reset();
		rearRightPID.reset();

		// Reset encoders
        frontLeftEncoder.reset();
        rearLeftEncoder.reset();
        frontRightEncoder.reset();
        rearRightEncoder.reset();

        // Set encoders to output position to PID controllers
		frontLeftEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);
		rearLeftEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);
		frontRightEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);
		rearRightEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);

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
	}

	public void PidStop() {
		// Disable PID controllers
		frontLeftPID.disable();
		rearLeftPID.disable();
		frontRightPID.disable();
		rearRightPID.disable();
	}

	public void Update() {
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
				table.putNumber("DT.PosR", refPos + smallNumber);
			}
			else
			{
				PidStop();
			}
		}

		smallNumber = (smallNumber == 0) ? 0.001: 0;

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
