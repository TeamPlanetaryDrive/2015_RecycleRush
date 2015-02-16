package org.usfirst.frc.team2856.robot;

public class MoveRefGen {
	// State machine enumeration
	private static enum MoveState {
		Idle,
		Accel,
		SteadyState,
		Decel,
		Settle
	}

	// Parameters
	private double accelRate;
	private double maxSpeed;
	private double settleTime;

	// States
	private MoveState moveState;
	private double decelStart;
	private double direction;
	private double endDist;
	private double refDist;
	private double refSpeed;
	private double settleTimer;

	MoveRefGen() {
		moveState = MoveState.Idle;
	}
	
	public void Configure(double accelRate, double maxSpeed, double settleTime) {
		this.accelRate = accelRate;
		this.maxSpeed = maxSpeed;
		this.settleTime = settleTime;
	}

	public double GetRefPosition() {
		return direction * refDist;
	}

	public double GetRefVelocity() {
		return direction * refSpeed;
	}

	public boolean IsActive() {
		return moveState != MoveState.Idle;
	}

	public void Start(double distance) {
		double decelDist;
		
		if (distance != 0)
		{
			if (distance > 0)
			{
				direction = 1;
				endDist = distance;
			}
			else
			{
				direction = -1;
				endDist = -1 * distance;
			}
			decelDist = (maxSpeed * maxSpeed) / (2 * accelRate);
			if ((2 * decelDist) > endDist)
			{
				decelDist = endDist / 2;
			}
			decelStart = endDist - decelDist;
			refDist = 0;
			refSpeed = 0;
			moveState = MoveState.Accel;
		}
	}

	public void Stop() {
		refSpeed = 0;
		moveState = MoveState.Idle;
	}

	public void Update() {
		switch (moveState) {
			case Idle:
				// Do nothing
				break;
			case Accel:
				refSpeed += accelRate * RobotConstants.PERIODIC_UPDATE_PERIOD;
				if (refSpeed >= maxSpeed)
				{
					refSpeed = maxSpeed;
					moveState = MoveState.SteadyState;
				}
				refDist += refSpeed * RobotConstants.PERIODIC_UPDATE_PERIOD;
				if (refDist >= decelStart)
				{
					moveState = MoveState.Decel;
				}
				break;
			case SteadyState:
				refDist += refSpeed * RobotConstants.PERIODIC_UPDATE_PERIOD;
				if (refDist >= decelStart)
				{
					moveState = MoveState.Decel;
				}
				break;
			case Decel:
	            refSpeed -= accelRate * RobotConstants.PERIODIC_UPDATE_PERIOD;
	            if (refSpeed <= 0)
	            {
	                refSpeed = 0;
	                refDist = endDist;
	                settleTimer = 0;
	                moveState = MoveState.Settle;
	            }
	            else
	            {
	                refDist += refSpeed * RobotConstants.PERIODIC_UPDATE_PERIOD;
	                if (refDist >= endDist)
	                {
	                    refSpeed = 0;
	                    refDist = endDist;
	                    if (settleTime != 0)
	                    {
		                    settleTimer = 0;
		                    moveState = MoveState.Settle;
	                    }
	                    else
	                    {
	                    	moveState = MoveState.Idle;
	                    }
	                }
	            }
				break;
			case Settle:
	            settleTimer += RobotConstants.PERIODIC_UPDATE_PERIOD;
	            if (settleTimer >= settleTime)
	            {
	                moveState = MoveState.Idle;
	            }
				break;
		}
	}
}
