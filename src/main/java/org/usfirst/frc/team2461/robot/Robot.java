package org.usfirst.frc.team2461.robot;

import org.usfirst.frc.team2461.robot.autonomous.*;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot
{
	// ****Creating Constants to use****
	// =================================
	// *******Motor Constants****
	final int LEFT_FRONT_WHEEL = 0;
	final int LEFT_REAR_WHEEL = 1;
	final int RIGHT_FRONT_WHEEL = 2;
	final int RIGHT_REAR_WHEEL = 3;
	final int GEAR_MOTOR = 4;
	final int[] ROBOT_LIFT_MOTORS = { 5, 6 };

	// *******Solenoid Constants****
	final int[] TRAY_LIFT_SOLENOIDS = { 0, 1 };
	final int[] BALL_DOOR_SOLENOIDS = { 2, 3 };
	final int[] BREAK_STOP_SOLENOIDS = { 4, 5 };

	// *******Digital Input Constants****
	final int GEAR_MANAGER_LIMIT_SWITCH_LEFT = 1;
	final int GEAR_MANAGER_LIMIT_SWITCH_RIGHT = 0;

	// ****Auto Strings****
	final String defaultAuto = "Default";
	final String CenterAuto = "Center Auto";
	final String LeftAuto = "Left Auto";
	final String RightAuto = "Right Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();

	// ****Autonomous Variables****
	// ============================
	DriveForwardAuto driveForwardAuto;
	LeftTurnAuto leftTurnAuto;
	CenterAuto centerAuto;
	RightTurnAuto rightTurnAuto;

	// ****Creating Drive Train Subsystem****
	// ======================================
	double powerFactor = 0.85;
	Talon leftFront = new Talon(LEFT_FRONT_WHEEL);
	Talon leftRear = new Talon(LEFT_REAR_WHEEL);
	Talon rightFront = new Talon(RIGHT_FRONT_WHEEL);
	Talon rightRear = new Talon(RIGHT_REAR_WHEEL);
	//RobotDrive chassis = new RobotDrive(leftFront, leftRear, rightFront, rightRear);
	MecanumDrive chassis = new MecanumDrive(leftFront, leftRear, rightFront, rightRear);

	// ****Creating MetalSkins Controller****
	// ======================================
	MetalSkinsController player1 = new MetalSkinsController(0);

	// ****Creating Camera Subsystem****
	// =================================
	Tracker track;

	// ****Creating PID Drive Train Subsystem****
	// ==========================================
	BuiltInAccelerometer acc = new BuiltInAccelerometer();
	ADXRS450_Gyro gyro1 = new ADXRS450_Gyro();
	private static Timer time = new Timer();
	PIDDriveTrain pidDriveTrain = new PIDDriveTrain(chassis, acc, 0.5, 0.0, 0.0);
	// PIDDriveTrain pidDriveTrain = new PIDDriveTrain(chassis, acc, 0.5, 0.0, 0.0, gyro1);

	// ****Creating Ball Collector Subsystems****
	// ==========================================
	DoubleSolenoid trayLift = new DoubleSolenoid(TRAY_LIFT_SOLENOIDS[0], TRAY_LIFT_SOLENOIDS[1]);
	DoubleSolenoid doorOperator = new DoubleSolenoid(BALL_DOOR_SOLENOIDS[0], BALL_DOOR_SOLENOIDS[1]);
	BallCollector ballCollector = new BallCollector(player1, trayLift, doorOperator);

	// ****Create Gear Manager Subsystems****
	// ======================================
	Talon gearMotor = new Talon(GEAR_MOTOR);
	DigitalInput gearManagerLimitSwitchLeft = new DigitalInput(GEAR_MANAGER_LIMIT_SWITCH_LEFT);
	DigitalInput gearManagerLimitSwitchRight = new DigitalInput(GEAR_MANAGER_LIMIT_SWITCH_RIGHT);
	GearManager gearManager = new GearManager(gearMotor, gearManagerLimitSwitchLeft, gearManagerLimitSwitchRight, player1);

	// ****Create Robot Lift Subsystem****
	// ===================================
	Spark motor1 = new Spark(ROBOT_LIFT_MOTORS[0]);
	Spark motor2 = new Spark(ROBOT_LIFT_MOTORS[1]);
	RobotLift robotLift = new RobotLift(motor1, motor2, player1);
	// DigitalInput robotLiftLimitSwitch = new DigitalInput(2);
	// RobotLift robotLift = new RobotLift(motor1, motor1, robotLiftLimitSwitch,
	// player1);
	
	//****Create Break Stop Subsystem****
	//===================================
	DoubleSolenoid breakStopDoubleSolenoid = new DoubleSolenoid(BREAK_STOP_SOLENOIDS[0], BREAK_STOP_SOLENOIDS[1]);
	BreakStop breakStop = new BreakStop(breakStopDoubleSolenoid, player1);
	Compressor c = new Compressor();
	
	boolean isDisabled = false;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit()
	{
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("Left Auto", LeftAuto);
		chooser.addObject("Center Auto", CenterAuto);
		chooser.addObject("Right Auto", RightAuto);
		SmartDashboard.putData("Auto choices", chooser);

		//acc.setRange(Range.k4G);
		time.start();
		//pidDriveTrain.setEnabled();

		// ****Setting up Drive Train Subsystems****
		rightFront.setInverted(true);
		rightRear.setInverted(true);

		// ****Setting up Gear Manager Subsystem****
		gearManager.setMaximumSpeed(0.5);
		//gearManager.enableDebugMode();

		// ****Setting up Robot Lift Subsystem****
		robotLift.setMaximumSpeed(1);

		// ****Setting up Camera****
		track = new Tracker(player1);
		
		// ****Starting Compressor Stuff****
		c.setClosedLoopControl(true);
		c.start();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit()
	{
		autoSelected = chooser.getSelected();
		//track.startTracking();
	
		switch (autoSelected)
		{
			case LeftAuto:
				leftTurnAuto = new LeftTurnAuto(chassis, track);
				leftTurnAuto.enableDebugMode();
				break;
			case CenterAuto:
				centerAuto = new CenterAuto(chassis, track);
				centerAuto.enableDebugMode();
				break;
			case RightAuto:
				rightTurnAuto = new RightTurnAuto(chassis, track);
				rightTurnAuto.enableDebugMode();
				break;
			case defaultAuto:
				driveForwardAuto = new DriveForwardAuto(chassis);
				driveForwardAuto.enableDebugMode();
				break;
			default:
				// Put default auto code here
				break;
		}

		//pidDriveTrain.resetSetPoint(1.5, 0, 0);
		//pidDriveTrain.setMaxOutput(0.50, 0.15);
		//pidDriveTrain.setAccelerometerOffsets();
		//pidDriveTrain.setAccScalefactor(5);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic()
	{
		if(isDisabled)
		{
			isDisabled = false;
		}
		switch (autoSelected)
		{
			case LeftAuto:
				leftTurnAuto.run();
				break;
			case CenterAuto:
				centerAuto.run();
				break;
			case RightAuto:
				rightTurnAuto.run();
				break;
			case defaultAuto:
				driveForwardAuto.run();
				break;
			default:
				// Put default auto code here
				break;
		}

		// pidDriveTrain.run();

	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic()
	{
		//This is for verifying the operations of the tracking code
		if(isDisabled)
		{
			isDisabled = false;
		}
		
		// ****Normal Operation Stuff****
		
		//Drive normal unless using rear camera. Flip driving controls if so!
		if(track.getCurrentCamera() == 0) {
			//chassis.mecanumDrive_Cartesian(player1.getX(Hand.kLeft) * powerFactor, player1.getY(Hand.kLeft) * powerFactor, player1.getX(Hand.kRight) * powerFactor, 0);
			chassis.driveCartesian(player1.getY(Hand.kLeft) * powerFactor, player1.getX(Hand.kLeft) * powerFactor, player1.getX(Hand.kRight) * powerFactor);
		} else {
			//chassis.mecanumDrive_Cartesian(-player1.getX(Hand.kLeft) * powerFactor, -player1.getY(Hand.kLeft) * powerFactor, player1.getX(Hand.kRight) * powerFactor, 0);
			chassis.driveCartesian(-player1.getY(Hand.kLeft) * powerFactor, -player1.getX(Hand.kLeft) * powerFactor, player1.getX(Hand.kRight) * powerFactor);
		}
		
		ballCollector.run();
		gearManager.run();
		robotLift.run();
		breakStop.run();
		track.run();
		SmartDashboard.putString("Camera View", track.getCurrentCameraString());
		SmartDashboard.putBoolean("Camera View Bool", track.getCurrentCamera() == 0);
		
		
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic()
	{
	}

	@Override
	public void disabledPeriodic()
	{
		//This is for verifying the operations of the tracking code
		//pidDriveTrain.resetAccData();
		if(!isDisabled)
		{
			track.stopTracking();
			isDisabled = true;
		}
	}

	public static double getTime()
	{
		return time.get();
	}
	
	public void debugThings()
	{
		// Updating values and outputs to the SmartDashboard
				// These have no affect on what the robot does
				SmartDashboard.putNumber("Accelerometer X", pidDriveTrain.getAccelerometerData()[0]);
				SmartDashboard.putNumber("Accelerometer Y", pidDriveTrain.getAccelerometerData()[1]);
				pidDriveTrain.updateDistance();
				SmartDashboard.putNumber("Velocity X", pidDriveTrain.getVelocities()[0]);
				SmartDashboard.putNumber("Velocity y", pidDriveTrain.getVelocities()[1]);
				SmartDashboard.putNumber("Distance X", pidDriveTrain.getDistanceXTraveled());
				SmartDashboard.putNumber("Distance Y", pidDriveTrain.getDistanceYTraveled());
				
				// SmartDashboard.putNumber("Distance X",
				// pidDriveTrain.getDistanceXTraveled());
				// SmartDashboard.putNumber("Distance Y",
				// pidDriveTrain.getDistanceYTraveled());
				// SmartDashboard.putNumber("Gyro Angle",
				// pidDriveTrain.gyro1.getAngle());
	}
}
