
package org.usfirst.frc.team2856.robot;

import edu.wpi.first.wpilibj.IterativeRobot;

public class Robot extends IterativeRobot {
    private AbstractInputManager input;
    private Arm arm;
    private Drive drive;
    private Lift lift;
	
	public void robotInit() {
		arm = new Arm();
		drive = new Drive();
		lift = new Lift();
    }

    public void autonomousInit() {
    	input = new AutoInputManager();
    }

    public void autonomousPeriodic() {
    	input.update();
    }

    public void disabledInit() {

    }

    public void disabledPeriodic() {

    }

    public void teleopInit() {
        input = new TelopInputManager();
    }
    
    public void teleopPeriodic() {
        input.update();
    }
    
    public void testInit() {
        
    }

    public void testPeriodic() {
    
    }
    
}
