package org.usfirst.frc.team2461.robot.autonomous;

import org.usfirst.frc.team2461.robot.Robot;
import org.usfirst.frc.team2461.robot.Tracker;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CenterAuto
{
	enum State {BEGIN, DRIVE_FORWARD, FINDING_TARGETS, CENTERING_TARGETS, STOP};
	private State autoState;
	private State previousState;
	//private RobotDrive chassis;
	private MecanumDrive chassis;
	private double futureTime;
	private Tracker track;
	private double maximumDriveSpeed = -0.35;
	private double maximumDriveSpeed2 = -0.2;
	private double maximumTurnSpeed = 0.1;
	private double initialDriveForwardTime = 2.5;
	private double centerUpDriveForwardTime = 3.5;
	private double targetRange = 20;
	private boolean debugState = false;
	
//	public CenterAuto(RobotDrive driveTrain, Tracker trackIn)
//	{
//		chassis = driveTrain;
//		track = trackIn;
//		autoState = State.BEGIN;
//	}
	
	public CenterAuto(MecanumDrive driveTrain, Tracker trackIn)
	{
		chassis = driveTrain;
		track = trackIn;
		autoState = State.BEGIN;
	}
	
	public void run()
	{
		switch(autoState)
		{
			case BEGIN:
				begin();
				break;
			case CENTERING_TARGETS:
				centeringTargets();
				break;
			case DRIVE_FORWARD:
				driveForward();
				break;
			case FINDING_TARGETS:
				findingTargets();
				break;
			case STOP:
				stop();
				break;
			default:
				break;
		}
		
		if(debugState)
			debug();
	}
	
	private void begin()
	{
		futureTime = Robot.getTime() + initialDriveForwardTime;
		//chassis.mecanumDrive_Cartesian(0, maximumDriveSpeed, 0, 0);
		chassis.driveCartesian(maximumDriveSpeed, 0, 0);
		autoState = State.DRIVE_FORWARD;
		previousState = State.BEGIN;
		track.startTracking();
	}
	
	private void driveForward()
	{
		if(previousState == State.BEGIN)
		{
			if(Robot.getTime() > futureTime)
			{
				//chassis.mecanumDrive_Cartesian(0, 0, -maximumTurnSpeed, 0);
				chassis.driveCartesian(0, 0, -maximumTurnSpeed);
				autoState = State.FINDING_TARGETS;
				previousState = State.DRIVE_FORWARD;
			}
			else
			{
				//chassis.mecanumDrive_Cartesian(0, maximumDriveSpeed, 0, 0);
				chassis.driveCartesian(maximumDriveSpeed, 0, 0);
			}
		}
		else if(previousState == State.CENTERING_TARGETS)
		{
			if(Robot.getTime() > futureTime)
			{
				//chassis.mecanumDrive_Cartesian(0, 0, 0, 0);
				chassis.driveCartesian(0, 0, 0);
				autoState = State.STOP;
				previousState = State.DRIVE_FORWARD;
			}
			else if(track.getFromCenter() > targetRange || track.getFromCenter() < -targetRange)
			{
				autoState = State.CENTERING_TARGETS;
				previousState = State.DRIVE_FORWARD;
				//chassis.mecanumDrive_Cartesian(0, 0, 0, 0);
				chassis.driveCartesian(0, 0, 0);
			}
			else
			{
				//chassis.mecanumDrive_Cartesian(0, maximumDriveSpeed2, 0, 0);
				chassis.driveCartesian(maximumDriveSpeed2, 0, 0);
			}
		}
	}
	
	private void findingTargets()
	{
		if(track.getTargetStatus())
		{
			autoState = State.CENTERING_TARGETS;
			previousState = State.FINDING_TARGETS;
			//chassis.mecanumDrive_Cartesian(0, 0, 0, 0);
			chassis.driveCartesian(0, 0, 0);
		}
		else
		{
			//chassis.mecanumDrive_Cartesian(0, 0, -maximumTurnSpeed, 0);
			chassis.driveCartesian(0, 0, -maximumTurnSpeed);
		}
	}
	
	private void centeringTargets()
	{
		if(track.getFromCenter() > -targetRange && track.getFromCenter() < targetRange)
		{
			autoState = State.DRIVE_FORWARD;
			previousState = State.CENTERING_TARGETS;
			futureTime = Robot.getTime() + centerUpDriveForwardTime;
			//chassis.mecanumDrive_Cartesian(0, maximumDriveSpeed, 0, 0);
			chassis.driveCartesian(maximumDriveSpeed, 0, 0);
		}
		else
		{
			centerUpTarget();
		}
	}
	
	private void stop()
	{
		
	}
	
	public void setMaximumDrivingSpeed(double value)
	{
		maximumDriveSpeed = value;
	}
	
	public void setMaximumTurningSpeed(double value)
	{
		maximumTurnSpeed = value;
	}
	
	private void centerUpTarget()
	{
		//chassis.mecanumDrive_Cartesian(0, 0, normalizeTurnAdjust(), 0);
		chassis.driveCartesian(0, 0, normalizeTurnAdjust());
	}
	
	private void debug()
	{
		SmartDashboard.putString("CenterAuto Debug State", this.autoState.toString());
		SmartDashboard.putNumber("Number of found contours", track.getfilterContoursOutput());
		SmartDashboard.putBoolean("Found Target", track.getTargetStatus());
		SmartDashboard.putNumber("Counter", track.getCounter());
	}
	
	public void enableDebugMode()
	{
		debugState = true;
	}
	
	public void disableDebugMode()
	{
		debugState = false;
	}
	
	private double normalizeTurnAdjust()
	{
		double turnValue = track.getFromCenter() / track.IMG_WIDTH;
		if(turnValue < 0.1 && turnValue >= 0)
			return 0.1;
		else if(turnValue > -0.1 && turnValue <= 0)
			return -0.1;
		else
			return turnValue;
	}
}
