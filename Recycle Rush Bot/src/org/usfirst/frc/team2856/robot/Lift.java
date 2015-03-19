package org.usfirst.frc.team2856.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSource.PIDSourceParameter;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class Lift {
	//Basic vars
	private NetworkTable table;
    private Encoder enc;
    private SpeedController motor;
    PowerDistributionPanel pdp;
    DigitalInput lowerSwitch, upperSwitch;
    
    //PID vars
    private PIDController pid;
    private PidSpeedController control;
    private double
    	Kp = RobotConstants.LIFT_PID_KP,
    	Ki = RobotConstants.LIFT_PID_KI,
    	Kd = RobotConstants.LIFT_PID_KD;
    
    private boolean PIDon = false;
    
    //misc vars
    private boolean moveActive;
 	private MoveRefGen refGen;
 	private double smallNumber;
    
 	//move vars
 	private double distFromOrigin;//assuming origin is the bottom
 	private double totalDist  = RobotConstants.LIFT_HEIGHT;//inches
 	private double target;
 	
	public Lift(){
		//Instantiate Variables
		table = NetworkTable.getTable(RobotConstants.NT_SOURCE);
		
		enc = new Encoder(RobotConstants.LIFT_ENC_CHANNEL_A, RobotConstants.LIFT_ENC_CHANNEL_B);
		motor = new Jaguar(RobotConstants.LIFT_SC_CHANNEL);
		lowerSwitch = new DigitalInput(RobotConstants.LIFT_LIMIT_LOWER_CHANNEL);
		upperSwitch = new DigitalInput(RobotConstants.LIFT_LIMIT_UPPER_CHANNEL);
		
		//@param Kp, Ki, Kd, PIDSource, PIDOutput
		pid = new PIDController(Ki, Kp, Kd, enc, motor);
		
		//Start Variables
		enc.reset();
		enc.setDistancePerPulse(RobotConstants.LIFT_ENC_RESOLUTION);
		enc.setPIDSourceParameter(PIDSourceParameter.kDistance);
		
		//instantiate PidSpeedController as an intermediate variable
		control = new PidSpeedController(pid, motor);
		
		// PID move initialization
		moveActive = false;
		refGen = new MoveRefGen();
		
		//move vars
		distFromOrigin = 0;
	}
	

	public void PIDMoveStart(double dist){
		double Kp, Ki, Kd;
		double accelRate;
		double maxSpeed;

		// Update local parameters
		Kp = table.getNumber("Lift.Kp");
		Ki = table.getNumber("Lift.Ki");
		Kd = table.getNumber("Lift.Kd");
		accelRate = table.getNumber("Lift.AccelRate");
		maxSpeed = table.getNumber("Lift.MaxSpeed");

		// Reset PID controller
		pid.reset();

		// Reset encoders
        enc.reset();
        
		// Set PID parameters
		pid.setPID(Kp, Ki, Kd);

		// Set PID set points to zero
		pid.setSetpoint(0);
		
		// Enable PID controllers
		pid.enable();

		// Configure and start move reference generator
		refGen.Configure(accelRate, maxSpeed, RobotConstants.LIFT_PID_POS_SETTLE);
		refGen.Start(dist);
		
		// Position move active
		moveActive = true;
	}
	
	public void PIDStop(){
		//disable pid
		pid.disable();
		
		// Position move finished
		moveActive = false;
	}
	
	public double EncoderGetPosition(){
		return enc.getDistance();
	}

	public void EncoderReset(){
		enc.reset();
	}
	
	public void Update(boolean debug){
		
		smallNumber = (smallNumber == 0) ? 0.001: 0;
		
		//if there is a move active
		if (moveActive){
			//update the reference generator
			refGen.Update();
			
			if (refGen.IsActive()){
				//get and set reference position
				double refPos = refGen.GetRefPosition();
				pid.setSetpoint(refPos);
				
				if (debug){
					table.putNumber("Lift.PosR", refPos + smallNumber);
				}
			}
			else{
				PIDStop();
			}
		}

		
		if (debug){
			//update table
			
			//table.putNumber("Lift.PosR", enc.getDistance() + smallNumber);
			table.putNumber("Lift.Pos",  enc.getDistance() + smallNumber);
			table.putNumber("Lift.VelR", pid.getSetpoint() + smallNumber);
			table.putNumber("Lift.Vel",  enc.getRate() + smallNumber);
			table.putNumber("Lift.Eff",  motor.get() + smallNumber);
			table.putNumber("Lift.Cur",  pdp.getCurrent(RobotConstants.LIFT_MOTOR_POWERPANEL_CHANNEL) + smallNumber);
		}
		distFromOrigin += enc.getDistance();
	}
	
	public void Move(double effort){
		
		if (lowerSwitch.get() || upperSwitch.get()) {//if a switch is pressed
			//if lower limit is pressed only let this go up
			if (lowerSwitch.get()) {
				if (effort < 0)
					motor.set(0);
				else
					motor.set(effort);
			}
			//if upper limit is pressed only let it go down
			if (upperSwitch.get()) {
				if (effort > 0)
					motor.set(0);
				else
					motor.set(effort);
			}
		}else{//no switch is pressed
			motor.set(effort);
		}
	}
	
	/*move to specific location*/
	public void moveToTop(){
		if(!upperSwitch.get()){
			if(PIDon)
				PIDMoveStart(totalDist - distFromOrigin);//find out distance
			else
				moveDist(totalDist - distFromOrigin);
		}
	}
	
	public void moveToBottom(){
		if(!lowerSwitch.get()){
			if(PIDon)
				PIDMoveStart(-distFromOrigin);//find out distance
			else
				moveDist(-distFromOrigin);
		}
	}
	
	/*FIX THIS FOR DISTANCE*/
	public void moveDist(double dist){
		target = dist += distFromOrigin;
		if (lowerSwitch.get() || upperSwitch.get()) {//if a switch is pressed
			//if lower limit is pressed only let this go up
			if (lowerSwitch.get()) {
				if(dist < 0){
					distFromOrigin += enc.getDistance();
					dist = 0;
					enc.reset();
				}
			}
			//if upper limit is pressed only let it go down
			if (upperSwitch.get()) {
				if(dist > 0){
				distFromOrigin += enc.getDistance();
				dist = 0;
				enc.reset();
			}
			}else{//no switch is pressed
				if(target < distFromOrigin){
					motor.set(-.07);
				}else if(target > distFromOrigin){
					motor.set(.07);
				}
			}
		}
	}
}