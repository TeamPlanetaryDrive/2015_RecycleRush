package org.usfirst.frc.team2856.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class Lift {
	private NetworkTable table;
    private Encoder enc;
    private SpeedController motor;
    PowerDistributionPanel pdp;
    DigitalInput lowerSwitch, upperSwitch;
    
    //PID vars
    private PIDController pid;
    private double
    	Kp = 0.2,
    	Ki = 0.1,
    	Kd = 0;
    private boolean PIDon = false;
    
	public Lift(){
		//Instantiate Variables
		table = NetworkTable.getTable(RobotConstants.NT_SOURCE);
		
		enc = new Encoder(RobotConstants.LIFT_ENC_CHANNEL_A, RobotConstants.LIFT_ENC_CHANNEL_B);
		motor = new Jaguar(RobotConstants.LIFT_SC_CHANNEL);
		lowerSwitch = new DigitalInput(RobotConstants.LIFT_LIMIT_LOWER_CHANNEL);
		upperSwitch = new DigitalInput(RobotConstants.LIFT_LIMIT_UPPER_CHANNEL);
		
		//@param Kp, Ki, Kd, PIDSource, PIDOutput
		pid = new PIDController(0, 0, 0, enc, motor);
		
		//Start Variables
		enc.reset();
		enc.setDistancePerPulse(RobotConstants.LIFT_ENC_RESOLUTION);
		//enc.setPIDSourceParameter(PIDSourceParameter.kDistance);
		
	}
	

	public void PIDMoveStart(double dist){
		pid.enable();
		pid.setInputRange(-1, 1);
		pid.setOutputRange(-1, 1);
		pid.setSetpoint(dist);
	}
	
	public void PIDStop(){
		
	}
	
	public double EncoderGetPosition(){
		return enc.getDistance();
	}

	public void EncoderReset(){
		enc.reset();
	}
	
	public void Update(){
		table.putNumber("Lift.Pos", enc.getDistance());
		//table.putNumber("Lift.PosR", );
		table.putNumber("Lift.Vel", enc.getRate());
		//table.putNumber("Lift.VelR", );
		//table.putNumber("Lift.Eff", );
		//table.putNumber("Lift.Cur", pdp.getCurrent(channel));//Channel Unknown
		
		
		
	}
	
	public void Move(double input){
		
		if (lowerSwitch.get() || upperSwitch.get()) {//if a switch is pressed
			//if lower limit is pressed only let this go up
			if (lowerSwitch.get()) {
				if (input < 0)
					motor.set(0);
				else
					motor.set(input);
			}
			//if upper limit is pressed only let it go down
			if (upperSwitch.get()) {
				if (input > 0)
					motor.set(0);
				else
					motor.set(input);
			}
		}else{//no switch is pressed
			motor.set(input);
		}
	}
	
	
	//work on effort and network tables
}
