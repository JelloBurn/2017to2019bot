package org.usfirst.frc.team2461.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotLift
{
	private Spark motor1;
	private Spark motor2;
	private DigitalInput limitSwitchTop;
	private MetalSkinsController player;
	private double maximumSpeed = 1;
	enum State {BEGIN, LIFT_FORWARD, LIFT_REVERSE, LIFT_FORWARD_STOP, LIFT_REVERSE_STOP, LIFT_NEUTRAL};
	private State ropeState;
	private boolean debugState = false;
//	private boolean lastButtonState = false;

	/**
	 * This constructor takes in a limit switch so that
	 * the system can stop when we reach the goal
	 * @param motor1In Motor1 of the lift system
	 * @param motor2In Motor2 of the lift system
	 * @param limitSwitchTopIn Limit switch that tells us we have 
	 * reached the goal
	 * @param playerIn MetalSkins Controller to control lift
	 */
	public RobotLift(Spark motor1In, Spark motor2In, DigitalInput limitSwitchTopIn, MetalSkinsController playerIn)
	{
		motor1 = motor1In;
		motor2 = motor2In;
		limitSwitchTop = limitSwitchTopIn;
		player = playerIn;
		ropeState = State.BEGIN;
	}

	/**
	 * This constructor creates a lift system with no
	 * stopping mechanism and must be controlled by a
	 * user
	 * @param motor1In Motor1 of the lift system
	 * @param motor2In Motor2 of the lift system
	 * @param playerIn MetalSkins Controller to control lift
	 */
	public RobotLift(Spark motor1In, Spark motor2In, MetalSkinsController playerIn)
	{
		motor1 = motor1In;
		motor2 = motor2In;
		player = playerIn;
		ropeState = State.BEGIN;
	}

	/**
	 * The Robot Lift uses the Bumper Buttons to raise and
	 * lower the robot.
	 * 
	 * <p>****Controller Buttons:****
	 * 		<br>Left Bumper: <br>&nbsp;&nbsp;&nbsp;&nbsp;Lower Robot
	 * 		<br>Right Bumper: <br>&nbsp;&nbsp;&nbsp;&nbsp;Raise Robot
	 */
	public void run()
	{
		switch(ropeState)
		{
			case BEGIN:
				begin();
				break;
			case LIFT_NEUTRAL:
				liftNeutral();
				break;
			case LIFT_FORWARD:
				liftForward();
				break;
			case LIFT_FORWARD_STOP:
				liftForwardStop();
				break;
			case LIFT_REVERSE:
				liftReverse();
				break;
			case LIFT_REVERSE_STOP:
				liftReverseStop();
				break;
		}
		
		if(debugState)
		{
			debug();
		}
	}

	private void setMotorSpeed(double value)
	{
		motor1.set(value);
		motor2.set(value);
	}

	public void setMaximumSpeed(double value)
	{
		maximumSpeed = value;
	}
	
	private void liftForward()
	{
		if(limitSwitchTop != null)
		{
			if(!player.getBumper(Hand.kRight))
			{
				setMotorSpeed(0);
				ropeState = State.LIFT_NEUTRAL;
			}
			else if(limitSwitchTop.get())
			{
				setMotorSpeed(0);
				ropeState = State.LIFT_FORWARD_STOP;
			}
		}
		else
		{
			if(!player.getBumper(Hand.kRight))
			{
				setMotorSpeed(0);
				ropeState = State.LIFT_NEUTRAL;
			}
		}
	}
	
	private void liftForwardStop()
	{
		if(player.getBumper(Hand.kLeft))
		{
			setMotorSpeed(-maximumSpeed);
			ropeState = State.LIFT_REVERSE;
		}
	}
	
	private void liftReverse()
	{
		if(limitSwitchTop != null)
		{
			if(!player.getBumper(Hand.kLeft))
			{
				setMotorSpeed(0);
				ropeState = State.LIFT_NEUTRAL;
			}
			else if(limitSwitchTop.get())
			{
				setMotorSpeed(0);
				ropeState = State.LIFT_REVERSE_STOP;
			}
		}
		else
		{
			if(!player.getBumper(Hand.kLeft))
			{
				setMotorSpeed(0);
				ropeState = State.LIFT_NEUTRAL;
			}
		}
		
	}
	
	private void liftReverseStop()
	{
		if(player.getBumper(Hand.kRight))
		{
			setMotorSpeed(-maximumSpeed);
			ropeState = State.LIFT_FORWARD;
		}
	}
	
	private void liftNeutral()
	{
		if(player.getBumper(Hand.kLeft) && player.getBumper(Hand.kRight))
		{

		}
		else if(player.getBumper(Hand.kLeft))
		{
			setMotorSpeed(-maximumSpeed);
			ropeState = State.LIFT_REVERSE;
		}
		else if(player.getBumper(Hand.kRight))
		{
			setMotorSpeed(maximumSpeed);
			ropeState = State.LIFT_FORWARD;
		}
	}
	
	private void begin()
	{
		setMotorSpeed(0);
		ropeState = State.LIFT_NEUTRAL;
	}
	
	/*
	private boolean checkButtonBumperRight()
	{
		if(player.getBumper(Hand.kRight) && !lastButtonState)
		{
			lastButtonState = true;
			return true;
		}
		else if(player.getBumper(Hand.kRight) && lastButtonState)
		{
			return false;
		}
		else
		{
			lastButtonState = false;
			return false;
		}
	}
	*/

	private void debug()
	{
		SmartDashboard.putString("Robot Lift - ropeState", ropeState.toString());
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
