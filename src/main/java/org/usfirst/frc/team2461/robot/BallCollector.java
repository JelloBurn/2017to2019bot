package org.usfirst.frc.team2461.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;

public class BallCollector
{
	double timeValue;
	private DoubleSolenoid trayLift;
	private DoubleSolenoid doorOperator;
	XboxController player;

	private enum State
	{
		BEGIN, DRIVE, COLLECT, DEPOSIT, COLLECT_RETRACT_DOOR
	}

	State stateNow;

	/**
	 * 
	 * @param playerIn MetalSkins Controller
	 * @param trayLiftIn Double Solenoid that moves the entire collector up and down
	 * @param doorOperatorIn Double Solenoid that opens/closes door
	 */
	public BallCollector(XboxController playerIn, DoubleSolenoid trayLiftIn, DoubleSolenoid doorOperatorIn)
	{
		player = playerIn;
		trayLift = trayLiftIn;
		doorOperator = doorOperatorIn;
		stateNow = State.BEGIN;
	}

	/**
	 * The Ball Collector uses the X button on the controller
	 * to open the door and raise the collector so that we receive
	 * balls. The A button is used to open the door so that we deposit
	 * the balls
	 *                    
	 * <p>****Controller Buttons:****
	 *		<br>X Button: <br>&nbsp;&nbsp;&nbsp;&nbsp;Collect Balls
	 *		<br>A Button: <br>&nbsp;&nbsp;&nbsp;&nbsp;Deploy Balls
	 */
	public void run()
	{
		switch (stateNow)
		{
			case BEGIN:
				begin();
				break;

			case DRIVE:
				drive();
				break;

			case COLLECT:
				collect();
				break;

			case COLLECT_RETRACT_DOOR:
				collectRetractDoor();
				break;

			case DEPOSIT:
				deposit();
				break;

			default:
				break;
		}
	}

	private void begin()
	{
		doorOperator.set(DoubleSolenoid.Value.kReverse);
		trayLift.set(DoubleSolenoid.Value.kReverse);
		stateNow = State.DRIVE;
	}

	private void drive()
	{
		if (player.getXButton())
		{
			doorOperator.set(DoubleSolenoid.Value.kForward);
			trayLift.set(DoubleSolenoid.Value.kForward);
			stateNow = State.COLLECT;
		} else if (player.getAButton())
		{
			doorOperator.set(DoubleSolenoid.Value.kForward);
			stateNow = State.DEPOSIT;
		}
	}

	private void collect()
	{
		if (!player.getXButton())
		{
			doorOperator.set(DoubleSolenoid.Value.kReverse);
			timeValue = Robot.getTime() + 1;
			stateNow = State.COLLECT_RETRACT_DOOR;
		}
	}

	private void collectRetractDoor()
	{
		if (Robot.getTime() >= timeValue)
		{
			trayLift.set(DoubleSolenoid.Value.kReverse);
			stateNow = State.DRIVE;
		}
	}

	private void deposit()
	{
		if (!player.getAButton())
		{
			doorOperator.set(DoubleSolenoid.Value.kReverse);
			stateNow = State.DRIVE;
		}
	}
}
