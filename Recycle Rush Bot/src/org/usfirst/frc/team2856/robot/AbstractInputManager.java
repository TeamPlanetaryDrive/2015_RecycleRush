package org.usfirst.frc.team2856.robot;

import java.util.TreeMap;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Joystick;

public abstract class AbstractInputManager {

	// TODO: Complete value update and caching
	public static final int
			ATTACK3_AXIS_X = 0,
			ATTACK3_AXIS_Y = 1,
			ATTACK3_AXIS_Z = 2;
	
	public static final int
			XBOX_AXIS_LX       = 0,
			XBOX_AXIS_LY       = 1,
			XBOX_AXIS_LTRIGGER = 2,
			XBOX_AXIS_RTRIGGER = 3,
			XBOX_AXIS_RX       = 4,
			XBOX_AXIS_RY       = 5,
			
			XBOX_BUTTON_A      = 1,
			XBOX_BUTTON_B      = 2,
			XBOX_BUTOTN_X      = 3,
			XBOX_BUTTON_Y      = 4,
			XBOX_BUTTON_LBUMP  = 5,
			XBOX_BUTTON_RBUMP  = 6,
			XBOX_BUTTON_BACK   = 7,
			XBOX_BUTTON_START  = 8,
			XBOX_BUTTON_L3     = 9,
			XBOX_BUTTON_R3     = 10,
			
			XBOX_POV           = 0;
			
			
	private    Joystick[] joysticks;
	private       boolean currButton, oldButton;
	private        double currAxis, oldAxis;
	private           int currPOV, oldPOV;
	
	private TreeMap<Pair, Consumer<Boolean>> buttonMap;
	private TreeMap<Pair, Consumer<Double>>  axisMap;
	private TreeMap<Pair, Consumer<Integer>> povMap;
	
	protected boolean[][] buttons;
	protected  double[][] axes;
	protected     int[][] povs;
	
	public AbstractInputManager(Joystick... jsticks) {
		joysticks = jsticks;
		buttons = new boolean[joysticks.length][];
		axes = new double[joysticks.length][];
		povs = new int[joysticks.length][];
		
		for(int i = 0; i < joysticks.length; i++) {
			buttons[i] = new boolean[joysticks[i].getButtonCount()];
			axes[i] = new double[joysticks[i].getAxisCount()];
			povs[i] = new int[joysticks[i].getPOVCount()];
		}
	}
	
	public void updateInputs() {
		for(int i = 0; i < joysticks.length; i++) {
			for(int j = 0; j < buttons[i].length; j++) {
				buttons[i][j] = joysticks[i].getRawButton(j + 1);
			}
			
			for(int j = 0; j < axes[i].length; j++) {
				axes[i][j] = joysticks[i].getRawAxis(j);
			}
			
			for(int j = 0; j < povs[i].length; j++) {
				povs[i][j] = joysticks[i].getPOV(j);
			}
		}
	}
	
	public void update() {
		for(Pair pair: buttonMap.keySet()) {
			oldButton = joysticks[pair.ijoy].getRawButton(pair.ivalue);
			
		}
	}
	
	private static class Pair implements Comparable<Pair> {
		
		public final int ijoy;
		public final int ivalue;
		
		public Pair(int ijoy, int ivalue) {
			this.ijoy = ijoy;
			this.ivalue = ivalue;
		}
		
		@Override
		public int compareTo(Pair o) {
			return ijoy   != o.ijoy   ? (ijoy   > o.ijoy   ? 1 : -1)
				 : ivalue != o.ivalue ? (ivalue > o.ivalue ? 1 : -1)
				 : 0;
		}
		
	}
}
