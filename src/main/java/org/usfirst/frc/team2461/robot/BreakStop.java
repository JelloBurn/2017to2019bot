package org.usfirst.frc.team2461.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class BreakStop
{
	private DoubleSolenoid breakStop;
	private MetalSkinsController player;
	
	enum State {BEGIN, RETRACT, DEPLOY};
	private State breakStopState;
	private boolean lastButtonState;
	
	public BreakStop(DoubleSolenoid breakStopIn, MetalSkinsController playerIn)
	{
		breakStop = breakStopIn;
		player = playerIn;
		breakStopState = State.BEGIN;
	}
	
	/**
	 * The Break Stop uses the Y Button to deploy and retract
	 * the breaks.
	 * <p>
	 * ****Controller Buttons:****
	 * 		<br>Y Button: <br>&nbsp;&nbsp;&nbsp;&nbsp;Toggle Deploy and Retract
	 */
	public void run()
	{
		switch(breakStopState)
		{
			case BEGIN:
				begin();
				break;
			case DEPLOY:
				deploy();
				break;
			case RETRACT:
				retract();
				break;
			default:
				break;
		}
	}
	
	private void begin()
	{
		breakStop.set(DoubleSolenoid.Value.kReverse);
		breakStopState = State.RETRACT;
	}
	
	private void retract()
	{
		if(checkButtonY())
		{
			breakStop.set(DoubleSolenoid.Value.kForward);
			breakStopState = State.DEPLOY;
		}
	}
	
	private void deploy()
	{
		if(checkButtonY())
		{
			breakStop.set(DoubleSolenoid.Value.kReverse);
			breakStopState = State.RETRACT;
		}
	}
	
	private boolean checkButtonY()
	{
		if(player.getYButton() && !lastButtonState)
		{
			lastButtonState = true;
			return true;
		}
		else if(player.getYButton() && lastButtonState)
		{
			return false;
		}
		else
		{
			lastButtonState = false;
			return false;
		}
	}
}
