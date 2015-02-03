package org.usfirst.frc.team2856.robot;

public class RobotConstants {
	
	// Ports for the Joysticks and the XBox controller.
	public static final int
			IM_JOYSTICK_LEFT   = 0,
			IM_JOYSTICK_RIGHT  = 1,
			IM_CONTROLLER_XBOX = 0;

	// Channels of the Drive Train SpeedControllers.
	// ex: FRONTL = Front-Left
	public static final int
			DT_SC_FRONTL_CHANNEL = 0,
			DT_SC_FRONTR_CHANNEL = 1,
			DT_SC_BACKL_CHANNEL  = 2,
			DT_SC_BACKR_CHANNEL  = 3;
	
	// Whether or not the specified Drive Train SpeedController is inverted.
	// ex: FRONTL = Front-Left
	public static final boolean
			DT_SC_FRONTL_INVERTED = false,
			DT_SC_FRONTR_INVERTED = false,
			DT_SC_BACKL_INVERTED  = false,
			DT_SC_BACKR_INVERTED  = false;
	
}
