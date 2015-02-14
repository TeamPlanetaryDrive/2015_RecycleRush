package org.usfirst.frc.team2856.robot;

public class RobotConstants {

	// Network table
	public static final String
		NT_SOURCE = "SmartDashboard";

	// InputManager Joysticks and the XBox controller Ports
	public static final int
		IM_JOYSTICK_LEFT   = 0,
		IM_JOYSTICK_RIGHT  = 1,
		IM_XBOX_CONTROLLER = 0;

	// Arm SpeedController Channels
	public static final int
		ARM_SC_LEFT_CHANNEL  = 4,
		ARM_SC_RIGHT_CHANNEL = 5;		

	// Drive Train Gyro Channel
	public static final int
		DT_GYRO_CHANNEL = 0;

	// Drive Train SpeedController Channels
	public static final int
		DT_SC_FRONTLEFT_CHANNEL  = 3,
		DT_SC_REARLEFT_CHANNEL   = 2,
		DT_SC_FRONTRIGHT_CHANNEL = 0,
		DT_SC_REARRIGHT_CHANNEL  = 1;

	// Drive Train Encoder Channels
	public static final int
		DT_ENC_FRONTLEFT_CHANNEL_A  = 0,
		DT_ENC_FRONTLEFT_CHANNEL_B  = 1,
		DT_ENC_REARLEFT_CHANNEL_A   = 2,
		DT_ENC_REARLEFT_CHANNEL_B   = 3,
		DT_ENC_FRONTRIGHT_CHANNEL_A = 4,
		DT_ENC_FRONTRIGHT_CHANNEL_B = 5,
		DT_ENC_REARRIGHT_CHANNEL_A  = 6,
		DT_ENC_REARRIGHT_CHANNEL_B  = 7;

	// Drive Train Parameters
	public static final double
		DT_ENC_RESOLUTION = 1/188.3,
		DT_PID_PERIOD = 0.010,
		DT_PID_EFFORT_MAX = 1.0;

}
