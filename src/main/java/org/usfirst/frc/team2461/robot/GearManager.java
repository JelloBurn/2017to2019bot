package org.usfirst.frc.team2461.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearManager
{
	private DigitalInput limitSwitchLeft;
	private DigitalInput limitSwitchRight;
	private DigitalInput limitSwitchCenter;
	private Talon motor;
	private MetalSkinsController player;
	private double maximunSpeed = 0.5;
	enum State {BEGIN, RETURN_HOME, OPERATE};
	private State currentState = State.BEGIN;
	enum GearPosition {LEFT, CENTER, RIGHT};
	private GearPosition gearPosition;
	private boolean debugState = false;
	
	/**
	 * This constructor takes in a center limit switch so
	 * that the returnToHome ability is enabled
	 * @param motorIn Motor to move the gear holder
	 * @param lsLeft Left side limit switch
	 * @param lsRight Right side limit switch
	 * @param lsCenter Center limit switch
	 * @param playerIn MetalSkins controller
	 */
	public GearManager(Talon motorIn, DigitalInput lsLeft, DigitalInput lsRight, DigitalInput lsCenter, MetalSkinsController playerIn)
	{
		motor = motorIn;
		limitSwitchLeft = lsLeft;
		limitSwitchRight = lsRight;
		limitSwitchCenter = lsCenter;
		player = playerIn;
		currentState = State.BEGIN;
	}
	
	/**
	 * This constructor does not take in a center limit switch
	 * and therefore disables the returnToHome ability
	 * @param motorIn Motor to move the gear holder
	 * @param lsLeft Left side limit switch
	 * @param lsRight Right side limit switch
	 * @param playerIn MetalSkins controller
	 */
	public GearManager(Talon motorIn, DigitalInput lsLeft, DigitalInput lsRight, MetalSkinsController playerIn)
	{
		motor = motorIn;
		limitSwitchLeft = lsLeft;
		limitSwitchRight = lsRight;
		player = playerIn;
		currentState = State.BEGIN;
	}
	
	/**
	 * The gear Managers uses the Left and Right Triggers on the
	 * controller to move the Gear Collector back and forth. The
	 * Back Button (If using a center limit switch) move the sled
	 * to the center position.
	 * 
	 * <p>****Controller Buttons:****
	 *		<br>Left Trigger: <br>&nbsp;&nbsp;&nbsp;&nbsp;Move Track Left 
	 *		<br>Right Trigger: <br>&nbsp;&nbsp;&nbsp;&nbsp;Move Track Right
	 *		<br>Back Button: <br>&nbsp;&nbsp;&nbsp;&nbsp;Move gear sled to center position
	 */
	public void run()
	{
		switch(currentState)
		{
			case BEGIN:
				begin();
				break;
			case RETURN_HOME:
				returnToHome();
				break;
			case OPERATE:
				operate();
				break;
			default:
				motor.set(0);
		}
		
		if(debugState)
		{
			debug();
		}
	}
	
	private void operate()
	{
		if(player.getTriggerAxis(Hand.kLeft)==1 && player.getTriggerAxis(Hand.kRight)==1)
		{
			motor.set(0);
		}
		else if(player.getTriggerAxis(Hand.kLeft)==1)
		{
			if(!limitSwitchLeft.get())
			{
				motor.set(-maximunSpeed);
				gearPosition = null;
			}
			else
			{
				motor.set(0);
				gearPosition = GearPosition.LEFT;
			}
		}
		else if(player.getTriggerAxis(Hand.kRight)==1)
		{
			if(!limitSwitchRight.get())
			{
				motor.set(maximunSpeed);
				gearPosition = null;
			}
			else
			{
				motor.set(0);
				gearPosition = GearPosition.RIGHT;
			}
		}
		else if(player.getBackButton())
		{
			currentState = State.RETURN_HOME;
		}
		else
		{
			motor.set(0);
		}
	}
	
	private void returnToHome()
	{
		if(limitSwitchCenter != null)
		{	
			if(gearPosition == null || gearPosition == GearPosition.RIGHT)
			{
				motor.set(-maximunSpeed);
			}
			else if(gearPosition == GearPosition.LEFT)
			{
				motor.set(maximunSpeed);
			}
			else if(gearPosition == GearPosition.CENTER)
			{
				motor.set(0);
				currentState = State.OPERATE;
			}
		}
		else
		{
			motor.set(0);
			currentState = State.OPERATE;
		}
	}
	
	private void begin()
	{
		if(limitSwitchCenter != null)
		{
			if(limitSwitchLeft.get())
			{
				gearPosition = GearPosition.LEFT;
			}
			else if(limitSwitchRight.get())
			{
				gearPosition = GearPosition.RIGHT;
			}
			else if(limitSwitchCenter.get())
			{
				gearPosition = GearPosition.CENTER;
			}
			
			currentState = State.OPERATE;
		}
		else
		{
			if(limitSwitchLeft.get())
			{
				gearPosition = GearPosition.LEFT;
			}
			else if(limitSwitchRight.get())
			{
				gearPosition = GearPosition.RIGHT;
			}
			
			currentState = State.OPERATE;
		}
	}
	
	public void setMaximumSpeed(double value)
	{
		maximunSpeed = value;
	}
	
	private void debug()
	{
		SmartDashboard.putString("GearManager Debug - Current State", currentState.toString());
	}
	
	public void enableDebugMode()
	{
		debugState = true;
	}
	
	public void disableDebugMode()
	{
		debugState = false;
	}
}
