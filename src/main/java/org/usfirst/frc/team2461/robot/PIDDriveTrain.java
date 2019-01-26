package org.usfirst.frc.team2461.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDDriveTrain
{
	private double[] output = {0,0, 0};
	private double[] setPoint = {0,0,0};
	private double kP, kI, kD;
	private double lastTime = 0;
	private double nowTime;
	private double timeChanged;
	private double errorX;
	private double errorY;
	private double errSumX = 0;
	private double errSumY = 0;
	private double dErrX;
	private double dErrY;
	private double lastErrorX = 0;
	private double lastErrorY = 0;
	//private RobotDrive chassis;
	private MecanumDrive chassis;
	private Accelerometer acc;
	public ADXRS450_Gyro gyro1;
	private double velocityXTotal;
	private double distanceXTotal;
	private double velocityYTotal;
	private double distanceYTotal;
	private double maximumXValue = 1;
	private double minimumXValue = -1;
	private double maximumYValue = 1;
	private double minimumYValue = -1;
	private double tolerance = 1;
	private boolean enabled = false;
	private double offsetX = 0, offsetY=0;
	private double accScaleFactor = 1;
	private double timeStop;
	private boolean timeHasBeenStopped = false;
	private double gyroTolerance = 5;
	private double errorGyro, errorGyroSum = 0, dErrorGyro, lastErrorGyro = 0;
	private boolean debugState = false;
	
	/**
	 * 
	 * @param chassisIn RobotDrive
	 * @param accIn Accelerometer Object
	 * @param kP_In Proportional Value for the PID Controller
	 * @param kI_In Integral Value for the PID Controller
	 * @param kD_In Derivative  Value for the PID Controller
	 */
//	public PIDDriveTrain(RobotDrive chassisIn, Accelerometer accIn, double kP_In, double kI_In, double kD_In, ADXRS450_Gyro gyroIn)
//	{
//		chassis = chassisIn;
//		acc = accIn;
//		kP = kP_In;
//		kI = kI_In;
//		kD = kD_In;
//		this.setAccelerometerOffsets();
//		gyro1 = gyroIn;
//		gyro1.calibrate();
//	}
//	
//	public PIDDriveTrain(RobotDrive chassisIn, Accelerometer accIn, double kP_In, double kI_In, double kD_In)
//	{
//		chassis = chassisIn;
//		acc = accIn;
//		kP = kP_In;
//		kI = kI_In;
//		kD = kD_In;
//		this.setAccelerometerOffsets();
//	}
	
	public PIDDriveTrain(MecanumDrive chassisIn, Accelerometer accIn, double kP_In, double kI_In, double kD_In, ADXRS450_Gyro gyroIn)
	{
		chassis = chassisIn;
		acc = accIn;
		kP = kP_In;
		kI = kI_In;
		kD = kD_In;
		this.setAccelerometerOffsets();
		gyro1 = gyroIn;
		gyro1.calibrate();
	}
	
	public PIDDriveTrain(MecanumDrive chassisIn, Accelerometer accIn, double kP_In, double kI_In, double kD_In)
	{
		chassis = chassisIn;
		acc = accIn;
		kP = kP_In;
		kI = kI_In;
		kD = kD_In;
		this.setAccelerometerOffsets();
	}
	
	public void compute()
	{
		nowTime = Robot.getTime();
		timeChanged = nowTime - lastTime;
		
		updateDistance();
		
		if(!onTargetX())
		{
			errorX = setPoint[0] - distanceXTotal;
			errSumX += (errorX * timeChanged);
			dErrX = (errorX - lastErrorX) / timeChanged;
			
			//Compute X Output
			output[0] = kP * errorX + kI * (1 - errSumX) + kD * dErrX;
		}
		else
		{
			output[0] = 0;
		}
		
		if(!onTargetY())
		{
			errorY = setPoint[1] - distanceYTotal;
			errSumY += (errorY * timeChanged);
			dErrY = (errorY - lastErrorY) / timeChanged;
			
			//Compute Y Output
			output[1] = kP * errorY + kI * (1 - errSumY) + kD * dErrY;
		}
		else
		{
			output[1] = 0;
		}
		
		try{
			if(!onTargetGryo())
			{
				errorGyro = setPoint[2] - getGyroData();
				errorGyroSum += (errorGyro * timeChanged);
				dErrorGyro = (errorGyro - lastErrorGyro) / timeChanged;
				
				output[2] = kP * errorGyro + kI * errorGyroSum + kD * dErrorGyro;
			}
		}
		catch (Exception e)
		{
			output[2] = 0;
		}
		
		
		lastErrorX = errorX;
		lastErrorY = errorY;
		lastTime = nowTime;
		normalizeOutput();
		driveTrainOutput();
	}
	
	public void setDistance(double setPointXIn, double setPointYIn)
	{
		setPoint[0] = setPointXIn;
		setPoint[1] = setPointYIn;
	}
	
	public void setDistance(double[] setPointIn)
	{
		setPoint = setPointIn;
	}
	
	private void driveTrainOutput()
	{
		if(gyro1 != null)
		{
			//chassis.mecanumDrive_Cartesian(output[0], output[1], output[2]*0.025, 0);
			chassis.driveCartesian(output[0], output[1], output[2]*0.025);
		}
		else
		{
			//chassis.mecanumDrive_Cartesian(output[0], output[1], 0, 0);
			chassis.driveCartesian(output[0], output[1], 0);
		}
	}
	
	/**
	 * This method updates the distance and velocity variables.
	 * 
	 * The X axis is positive to the right of the robot (look
	 * from at the robot from the back) and negative to the
	 * left.
	 * 
	 * The Y axis is positive to the front of the robot and
	 * negative to the rear of the robot.
	 */
	public void updateDistance()
	{
		nowTime = Robot.getTime();
		timeChanged = nowTime - lastTime;
		double[] accData = getAccelerometerData();
		
		distanceXTotal += velocityXTotal * timeChanged + (0.5 * accData[0] * (timeChanged*timeChanged));
		velocityXTotal += (accData[0]*timeChanged);
		
		distanceYTotal += velocityYTotal * timeChanged + (0.5 * -accData[1] * (timeChanged*timeChanged));
		velocityYTotal += (-accData[1]*timeChanged);
		
		setVelocitiesToZero();
		lastTime = nowTime;
	}
	
	private void normalizeOutput()
	{
		if(output[0] > maximumXValue)
		{
			output[0] = maximumXValue;
		}
		else if(output[0] < minimumXValue)
		{
			output[0] = minimumXValue;
		}
		
		if(output[1] > maximumYValue)
		{
			output[1] = maximumYValue;
		}
		else if(output[1] < minimumYValue)
		{
			output[1] = minimumYValue;
		}
	}
	
	private boolean onTargetX()
	{
		if((distanceXTotal > (setPoint[0] - tolerance)) && (distanceXTotal < (setPoint[0] + tolerance)))
		{
			return true;
		}
		else
			return false;				
	}
	
	private boolean onTargetY()
	{
		if((distanceYTotal > (setPoint[1] - tolerance)) && (distanceYTotal < (setPoint[1] + tolerance)))
		{
			return true;
		}
		else
			return false;				
	}
	
	public void setEnabled()
	{
		enabled = true;
	}
	
	public void setDisabled()
	{
		enabled = false;
	}
	
	public void run()
	{
		if(enabled)
		{
			compute();
			if(debugState)
			{
				debug();
			}
		}
	}
	
	public void resetSetPoint(double newSetPointX, double newSetPointY, double newGyroSetPoint)
	{
		setPoint[0] = newSetPointX;
		setPoint[1] = newSetPointY;
		setPoint[2] = newGyroSetPoint;
		distanceXTotal = 0;
		distanceYTotal = 0;
		velocityXTotal = 0;
		velocityYTotal = 0;
		lastTime = Robot.getTime();
		if(gyro1 != null)
			gyro1.reset();
	}
	
	public double getDistanceXTraveled()
	{
		return distanceXTotal;
	}
	
	public double getDistanceYTraveled()
	{
		return distanceYTotal;
	}
	
	/**
	 * 
	 * @return Returns a 2-dimension array holding the values
	 * distanceXTotal and distanceYTotal
	 */
	public double[] getDistanceTraveled()
	{
		double[] temp = {distanceXTotal, distanceYTotal};
		return temp;
	}
	
	public double[] getAccelerometerData()
	{
		double[] input = {0, 0};
		for(int x = 0; x < 20; x++)
		{
			input[0] += (acc.getX() - offsetX);
			input[1] += (acc.getY() - offsetY);
		}
		
		input[0] /= 20;
		input[1] /= 20;
		
		if(input[0] < 0.02 && input[0] > -0.02)
		{
			input[0] = 0;
		}
		
		if(input[1] < 0.02 && input[1] > -0.02)
		{
			input[1] = 0;
		}
		
		input[0] *= accScaleFactor;
		input[1] *= accScaleFactor;
		
		determineIfStopped(input);
		
		return input;
	}
	
	public void setMaxOutput(double valueX, double valueY)
	{
		maximumXValue = valueX;
		minimumXValue = -valueX;
		maximumYValue = valueY;
		minimumYValue = -valueY;
	}
	
	public double getXError()
	{
		return setPoint[0] - distanceXTotal;
	}
	
	public double getYError()
	{
		return setPoint[1] - distanceYTotal;
	}
	
	public double[] getOutput()
	{
		return output;
	}
	
	public double getVelocityX()
	{
		return velocityXTotal;
	}
	
	public double getVelocityY()
	{
		return velocityYTotal;
	}
	
	public double[] getVelocities()
	{
		double[] velocities = {velocityXTotal, velocityYTotal};
		return velocities;
	}
	
	public void setAccelerometerOffsets()
	{
		double[] offsetData = getAccelerometerData();
		offsetX = offsetData[0];
		offsetY = offsetData[1];
	}
	
	public void setAccScalefactor(double value)
	{
		accScaleFactor = value;
	}
	
	public void determineIfStopped(double[] input)
	{
		if(input[0] == 0 && input[1] == 0 && !timeHasBeenStopped)
		{
			timeStop = Robot.getTime() + 0.25;
			timeHasBeenStopped = true;
		}
		else if(input[0] != 0 || input[1] != 0)
		{
			timeHasBeenStopped = false;
		}
	}
	
	private void setVelocitiesToZero()
	{
		if(Robot.getTime() > timeStop && timeHasBeenStopped)
		{
			velocityXTotal = 0;
			velocityYTotal = 0;
		}
	}
	
	public void resetAccData()
	{
		distanceXTotal = 0;
		distanceYTotal = 0;
		velocityXTotal = 0;
		velocityYTotal = 0;
	}
	
	public double getGyroData()
	{
		if(gyro1.getAngle() < gyroTolerance && gyro1.getAngle() > -gyroTolerance)
		{
			return 0;
		}
		else
		{
			return gyro1.getAngle();
		}
	}
	
	private boolean onTargetGryo()
	{
		if(setPoint[2] - getGyroData() < 5 && setPoint[2] - getGyroData() > -5)
		{
			return true;
		}
		else
			return false;
	}
	
	private void debug()
	{
		SmartDashboard.putNumber("Accelerometer X", this.getAccelerometerData()[0]);
		SmartDashboard.putNumber("Accelerometer Y", this.getAccelerometerData()[1]);
		SmartDashboard.putNumber("Distance X", this.getDistanceXTraveled());
		SmartDashboard.putNumber("Distance Y", this.getDistanceYTraveled());
		SmartDashboard.putNumber("Velocity X", this.getVelocities()[0]);
		SmartDashboard.putNumber("Velocity y", this.getVelocities()[1]);
		SmartDashboard.putNumber("X Error", this.getXError());
		SmartDashboard.putNumber("Y Error", this.getYError());
		SmartDashboard.putNumber("X Output", this.getOutput()[0]);
		SmartDashboard.putNumber("Y Output", this.getOutput()[1]);
		SmartDashboard.putNumber("Gyro Angle", this.gyro1.getAngle());
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