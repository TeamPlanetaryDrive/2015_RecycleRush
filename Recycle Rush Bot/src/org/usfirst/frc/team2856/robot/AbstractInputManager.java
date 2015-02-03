package org.usfirst.frc.team2856.robot;

import edu.wpi.first.wpilibj.Joystick;

public abstract class AbstractInputManager {

	private    Joystick[] joysticks;
	protected boolean[][] buttons;
	protected  double[][] axes;
	
	public AbstractInputManager(Joystick... jsticks) {
		joysticks = jsticks;
		buttons = new boolean[joysticks.length][];
		axes = new double[joysticks.length][];
		
		for(int i = 0; i < joysticks.length; i++) {
			buttons[i] = new boolean[joysticks[i].getButtonCount()];
			axes[i] = new double[joysticks[i].getAxisCount()];	
		}
	}
	
	public void update() {
		for(int i = 0; i < joysticks.length; i++) {
			for(int j = 0; j < buttons[i].length; j++) {
				
			}
		}
	}
}
