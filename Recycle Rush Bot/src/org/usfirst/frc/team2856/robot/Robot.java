
package org.usfirst.frc.team2856.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class Robot extends IterativeRobot {
    private AbstractInputManager input;
    private Arm arm;
    private CounterWeight cWeight;
    private Drive drive;
    private Lift lift;
    private Pivot pivot;
    private NetworkTable table;
    private PowerDistributionPanel power;

	public void robotInit() {
		table   = NetworkTable.getTable(RobotConstants.NT_SOURCE);
		arm     = new Arm();
		cWeight = new CounterWeight();
		drive   = new Drive();
		lift    = new Lift();
		pivot   = new Pivot();
		power   = new PowerDistributionPanel();
    }

    public void autonomousInit() {
    	input = new AutoInputManager(arm, cWeight, drive, lift, pivot);
    }

    public void autonomousPeriodic() {
    	input.update();
    	drive.Update(false);
    }

    public void disabledInit() {

    }

    public void disabledPeriodic() {

    }

    public void teleopInit() {
        input = new TelopInputManager(arm, cWeight, drive, lift, pivot);
    }

    public void teleopPeriodic() {
        input.update();
        arm.Update(false);
        drive.Update(false);
        lift.Update(false);
    }

    public void testInit() {
    	input = new TelopInputManager(arm, cWeight, drive, lift, pivot);
    }

    public void testPeriodic() {
        input.update();
        arm.Update(false);
        drive.Update(false);
        lift.Update(true);
        table.putNumber("TotalCurrent", power.getTotalCurrent());
    }

}
