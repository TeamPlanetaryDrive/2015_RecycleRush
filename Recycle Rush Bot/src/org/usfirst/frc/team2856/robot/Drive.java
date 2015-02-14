package org.usfirst.frc.team2856.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class Drive {
	private NetworkTable table;
    
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

	// PID Constants
	private double Kp = 0.1;  //2.00;
	private double Ki = 0.01; //0.01;
	private double Kd = 0.2;  //1.00;


	public Drive() {
		table = NetworkTable.getTable(RobotConstants.NT_SOURCE);

		// Front Left Wheel
		frontLeftEncoder = new Encoder(RobotConstants.DT_ENC_FRONTLEFT_CHANNEL_A, RobotConstants.DT_ENC_FRONTLEFT_CHANNEL_B, false, EncodingType.k4X);
		frontLeftMotor = new Talon(RobotConstants.DT_SC_FRONTLEFT_CHANNEL);
		frontLeftPID = new PIDController(Kp, Ki, Kd, frontLeftEncoder, frontLeftMotor, RobotConstants.DT_PID_PERIOD);
		frontLeftPidSpeed = new PidSpeedController(frontLeftPID, frontLeftMotor);

		// Rear Left Wheel
		rearLeftEncoder = new Encoder(RobotConstants.DT_ENC_REARLEFT_CHANNEL_A, RobotConstants.DT_ENC_REARLEFT_CHANNEL_B, false, EncodingType.k4X);
		rearLeftMotor = new Talon(RobotConstants.DT_SC_REARLEFT_CHANNEL);
		rearLeftPID = new PIDController(Kp, Ki, Kd, rearLeftEncoder, rearLeftMotor, RobotConstants.DT_PID_PERIOD);
		rearLeftPidSpeed = new PidSpeedController(rearLeftPID, rearLeftMotor);

		// Front Right Wheel
		frontRightEncoder = new Encoder(RobotConstants.DT_ENC_FRONTRIGHT_CHANNEL_A, RobotConstants.DT_ENC_FRONTRIGHT_CHANNEL_B, true, EncodingType.k4X);
		frontRightMotor = new Talon(RobotConstants.DT_SC_FRONTRIGHT_CHANNEL);
		frontRightPID = new PIDController(Kp, Ki, Kd, frontRightEncoder, frontRightMotor, RobotConstants.DT_PID_PERIOD);
		frontRightPidSpeed = new PidSpeedController(frontRightPID, frontRightMotor);

		// Rear Right Wheel
		rearRightEncoder = new Encoder(RobotConstants.DT_ENC_REARRIGHT_CHANNEL_A, RobotConstants.DT_ENC_REARRIGHT_CHANNEL_B, true, EncodingType.k4X);
		rearRightMotor = new Talon(RobotConstants.DT_SC_REARRIGHT_CHANNEL);
		rearRightPID = new PIDController(Kp, Ki, Kd, rearRightEncoder, rearRightMotor, RobotConstants.DT_PID_PERIOD);
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
		
		// Set initial network table values
		table.putNumber("DT.Kp", Kp);
		table.putNumber("DT.Ki", Ki);
		table.putNumber("DT.Kd", Kd);
	}

	public void RDrive(double x, double y, double rotation) {
		double gyroAngle;
		
		gyroAngle = gyroActive ? gyro.getAngle() : 0;
		rDrive.mecanumDrive_Cartesian(x, y, rotation, gyroAngle);
	}

	public boolean isGyroActive()
	{ return gyroActive; }
	public void setGyroActive(boolean active)
	{ gyroActive = active; }
	
	public void EncoderGetPosition() {
		
	}

	public void EncoderGetRate() {
		
	}

	public void EncoderReset() {
		
	}

	public void GyroGetAngle() {
		
	}

	public void GyroGetRate() {
		
	}

	public void GyroReset() {
		
	}

	public void PidSpeedStart() {
		
	}

	public void PidStop() {
		
	}

	public void Update() {
		
	}
}
