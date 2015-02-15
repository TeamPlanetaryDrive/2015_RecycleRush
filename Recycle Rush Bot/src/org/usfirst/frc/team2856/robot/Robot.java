
package org.usfirst.frc.team2856.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class Robot extends IterativeRobot {
    private AbstractInputManager input;
    private Arm arm;
    private Drive drive;
    private Lift lift;
    private NetworkTable table;
    private PowerDistributionPanel power;

	public void robotInit() {
		table = NetworkTable.getTable(RobotConstants.NT_SOURCE);
		arm   = new Arm();
		drive = new Drive();
		lift  = new Lift();
		power = new PowerDistributionPanel();
    }

    public void autonomousInit() {
    	input = new AutoInputManager(arm, drive, lift);
    }

    public void autonomousPeriodic() {
    	input.update();
    }

    public void disabledInit() {

    }

    public void disabledPeriodic() {

    }

    public void teleopInit() {
        input = new TelopInputManager(arm, drive, lift);
    }

    public void teleopPeriodic() {
        input.update();
        arm.Update(false);
        drive.Update(false);
    }

    public void testInit() {
    	input = new TelopInputManager(arm, drive, lift);
    }

    public void testPeriodic() {
        input.update();
        arm.Update(true);
        drive.Update(true);
        table.putNumber("TotalCurrent", power.getTotalCurrent());
    }

}
