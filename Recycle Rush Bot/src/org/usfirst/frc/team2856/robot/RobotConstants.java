package org.usfirst.frc.team2856.robot;

public class RobotConstants {

	// Network table
	public static final String
		NT_SOURCE = "SmartDashboard";

	// Periodic update period
	public static final double
		PERIODIC_UPDATE_PERIOD = 0.020; // (s)

	// InputManager Joysticks and the XBox controller Ports
	public static final int
		IM_JOYSTICK_LEFT   = 0,
		IM_JOYSTICK_RIGHT  = 1,
		IM_XBOX_CONTROLLER = 2;

	// Arm Potentiometer Channels
	public static final int
		ARM_POT_LEFT_CHANNEL = 1,  //FIXME: value needs checking
		ARM_POT_RIGHT_CHANNEL = 2;  //FIXME: value needs checking

	// Arm Potentiometer Parameters
	public static final double
		ARM_POT_LEFT_RANGE = 270.0,  //FIXME: value needs updating
		ARM_POT_LEFT_OFFSET = 0.0,  //FIXME: value needs updating
		ARM_POT_RIGHT_RANGE = -270.0,
		ARM_POT_RIGHT_OFFSET = 258.0;

	// Arm SpeedController Channels
	public static final int
		ARM_SC_LEFT_CHANNEL  = 4,
		ARM_SC_RIGHT_CHANNEL = 5;		
	
	//Arm Limit Switch Channels
	public static final int
		ARM_LIMIT_LEFT_CHANNEL = 4,
		ARM_LIMIT_RIGHT_CHANNEL = 5;

	// Arm Current Channels
	public static final int
		ARM_CUR_LEFT_CHANNEL  = 8,
		ARM_CUR_RIGHT_CHANNEL = 14;

	// Arm Parameters
	public static final double
		ARM_ACCEL_RATE = 5.0,     // (rev/s^2)
		ARM_PID_EFFORT_MAX = 0.5, // (0-1)
		ARM_PID_PERIOD = 0.010,   // (s)
		ARM_PID_POS_SETTLE = 0.5, // (s)
		ARM_SPEED_MAX = 2.5;      // (rev/s)

	// Arm PID Gains
	public static final double
		ARM_PID_KP = 2.00,
		ARM_PID_KI = 0.01,
		ARM_PID_KD = 1.00;

	// Drive Train Gyro Channel
	public static final int
		DT_GYRO_CHANNEL = 0;

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

	// Drive Train SpeedController Channels
	public static final int
		DT_SC_FRONTLEFT_CHANNEL  = 3,
		DT_SC_REARLEFT_CHANNEL   = 2,
		DT_SC_FRONTRIGHT_CHANNEL = 0,
		DT_SC_REARRIGHT_CHANNEL  = 1;

	// Drive Train Current Channels
	public static final int
		DT_CUR_FRONTLEFT_CHANNEL  = 12,
		DT_CUR_REARLEFT_CHANNEL   = 13,
		DT_CUR_FRONTRIGHT_CHANNEL = 0,
		DT_CUR_REARRIGHT_CHANNEL  = 1;

	// Drive Train Parameters
	public static final int
		DT_ENC_SAMPLES_TO_AVERAGE = 4;
	public static final double
		DT_ACCEL_RATE = 5.0,     // (rev/s^2)
		DT_ENC_RESOLUTION = 1/188.3, // (wheel revs: 7 cpr * 26.9)
		DT_PID_EFFORT_MAX = 1.0, // (0-1)
		DT_PID_PERIOD = 0.010,   // (s)
		DT_PID_POS_SETTLE = 0.5, // (s)
		DT_SPEED_MAX = 2.5;      // (rev/s)

	// Drive Train PID Gains
	public static final double
		DT_PID_POSITION_KP = 2.00,
		DT_PID_POSITION_KI = 0.01,
		DT_PID_POSITION_KD = 1.00,
		DT_PID_SPEED_KP = 0.10,
		DT_PID_SPEED_KI = 0.01,
		DT_PID_SPEED_KD = 0.20;

	//Lift SpeedController Channels
	public static final int
		LIFT_SC_CHANNEL = 6;
	
	//Lift Encoder Channels
	public static final int
		LIFT_ENC_CHANNEL_A = 8,
		LIFT_ENC_CHANNEL_B = 9;
	
	//Lift Limit Switch
	public static final int
		LIFT_LIMIT_LOWER_CHANNEL = 10,
		LIFT_LIMIT_UPPER_CHANNEL = 11;
	
	//Lift Parameters
	public static final double
		LIFT_ENC_RESOLUTION = 1/188.3,
		LIFT_PID_POS_SETTLE = 0.5; // (s)
	
	//Lift motor power distribution channel
	public static final int
		LIFT_MOTOR_POWERPANEL_CHANNEL = 1;  //FIXME: value needs updating
	
	//Lift PID values
	public static final double
		LIFT_KI = 0.2,
		LIFT_KP = 0.1,
		LIFT_KD = 0;
	
	//height
	public static final double
		LIFT_HEIGHT = 70;//inches
	
	// Pivot SpeedController Channel
	public static final int
		PIVOT_SC_CHANNEL = 6;

	// Pivot Limit Switch Channel
	public static final int
		PIVOT_LIMIT_CHANNEL = 6;
}
