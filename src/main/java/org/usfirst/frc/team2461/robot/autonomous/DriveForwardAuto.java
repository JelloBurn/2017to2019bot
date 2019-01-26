package org.usfirst.frc.team2461.robot.autonomous;

import org.usfirst.frc.team2461.robot.Robot;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveForwardAuto
{
	enum State {BEGIN, DRIVE_FORWARD, STOP};
	private State autoState;
	//private RobotDrive chassis;
	private MecanumDrive chassis;
	private double futureTime;
	private double maximumDriveSpeed = -0.30;
	private double initialDriveForwardTime = 2.60;
	private boolean debugState = false;
	
//	public DriveForwardAuto(RobotDrive driveTrain)
//	{
//		chassis = driveTrain;
//		autoState = State.BEGIN;
//	}
	
	public DriveForwardAuto(MecanumDrive driveTrain)
	{
		chassis = driveTrain;
		autoState = State.BEGIN;
	}
	
	public void run()
	{
		switch(autoState){
			case BEGIN:
				begin();
				break;
			case DRIVE_FORWARD:
				driveForward();
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
	}
	
	private void driveForward()
	{
		if(Robot.getTime() > futureTime)
		{
			//chassis.mecanumDrive_Cartesian(0, 0, 0, 0);
			chassis.driveCartesian(0, 0, 0);
			autoState = State.STOP;
		}
		else
		{
			//chassis.mecanumDrive_Cartesian(0, maximumDriveSpeed, 0, 0);
			chassis.driveCartesian(maximumDriveSpeed, 0, 0);
		}
	}
	
	private void stop()
	{
		
	}
	
	public void setMaximumSpeed(double value)
	{
		maximumDriveSpeed = value;
	}
	
	private void debug()
	{
		SmartDashboard.putString("DriveForwardAuto Debug State", this.autoState.toString());
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
