package org.usfirst.frc.team3215.robot;


import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	boolean LastForklift = false;
	boolean desiredEncoder = false;
	//The joysticks and their respective USB ports on the driver station
	Joystick Joystick1 = new Joystick(0);
	Joystick Joystick2 = new Joystick(1);
	final String defaultAuto = "Default";
	final String autoLeft = "left goal";
	final String autoCenter = "center goal";
	final String autoRight = "right goal";
	String autoSelected;
	SpeedController driveLeft1 = new Victor(0);
	SpeedController driveLeft2 = new Victor(1);
	SpeedController driveRight1 = new Victor(2);
	SpeedController driveRight2 = new Victor(3);
	SpeedController ClimberLeft = new Victor(4);
	SpeedController ClimberRight = new Victor(5);
	SpeedController ForkliftMotor = new Victor(6);
	Encoder ForkliftEncoder = new Encoder(2,3,false,Encoder.EncodingType.k2X);
	BuiltInAccelerometer accel = new BuiltInAccelerometer();
	ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
	SendableChooser<String> chooser = new SendableChooser<>();
	UsbCamera camera;
	Contours grip = new Contours();
	CvSink cvSink;
	CvSource outputStream;
	Mat mat = new Mat();
	Thread visionThread;
	static BNO055 imu;
	private BNO055.CalData cal;
	Ultrasonic ultra;
	double[] defaultValue;
	ArrayList<MatOfPoint> publishableOutput;
	long setTime = 0;
	boolean hasTurned = false;
	DigitalInput climbLimiter = new DigitalInput(2);
	DigitalInput forkliftUp = new DigitalInput(3);
	DigitalInput forkliftDown = new DigitalInput(4);
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		defaultValue = new double[0];
		ultra = new Ultrasonic(1, 0);
		ultra.setAutomaticMode(true);
		imu = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS, BNO055.vector_type_t.VECTOR_EULER);
		while(!imu.isInitialized());
		cal = imu.getCalibration();

		System.out.println("\tCALIBRATION: Sys=" + cal.sys

				+ " Gyro=" + cal.gyro + " Accel=" + cal.accel

				+ " Mag=" + cal.mag);
		visionThread = new Thread(() -> {
			// Get the UsbCamera from CameraServer
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			// Set the resolution
			camera.setResolution(640, 480);
			camera.setExposureManual(0);
			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 640, 480);

			// Mats are very memory expensive. Lets reuse this Mat.
			Mat mat = new Mat();

			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted()) {
				if (cvSink.grabFrame(mat) == 0) {
					// Send the output the error.
					outputStream.notifyError(cvSink.getError());
					// skip the rest of the current iteration
					return;
				}
				grip.process(mat);
				publishableOutput = grip.convexHullsOutput();
			}
		});
		visionThread.setDaemon(true);
		visionThread.start();
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("left goal", autoLeft);
		chooser.addObject("center goal", autoCenter);
		chooser.addObject("right goal", autoRight);
		SmartDashboard.putData("Auto choices", chooser);
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
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
    	ForkliftEncoder.reset();
    	setTime = System.currentTimeMillis();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		SmartDashboard.putNumber("angle", imu.getHeading());
		SmartDashboard.putNumber("distance", ultra.getRangeInches());
		SmartDashboard.putNumber("Contours", publishableOutput.size());
		double[][] contours = new double[publishableOutput.size()][5];
		for(int i = 0; i < publishableOutput.size(); i++){
			MatOfPoint thisContour = publishableOutput.get(i);
			Rect dimen = Imgproc.boundingRect(thisContour);
			contours[i][0] = Imgproc.contourArea(thisContour);
			contours[i][1] = dimen.x + (dimen.width / 2);
			contours[i][2] = dimen.y + (dimen.height / 2);
			contours[i][3] = dimen.height;
			contours[i][4] = dimen.width;
			/*
			 * SmartDashboard.putNumber("area" + i, Imgproc.contourArea(thisContour));
			 * SmartDashboard.putNumber("centerX" + i, dimen.x + (dimen.width / 2));
			 * SmartDashboard.putNumber("centerY" + i, dimen.y + (dimen.height / 2));
			 * SmartDashboard.putNumber("height" + i, dimen.height);
			 * SmartDashboard.putNumber("width" + i, dimen.width);
			 */
			//I commented this out because it's not useful unless console is needed
		}
		switch (autoSelected) {
		case autoLeft:
			// Put custom auto code here
			if(!hasTurned && System.currentTimeMillis() - setTime > 1000){
				while(imu.getHeading() < 30){
					driveLeft1.set(.5);
			        driveLeft2.set(.5);
			        driveRight1.set(-.5);
			        driveRight2.set(-.5);
				}
				hasTurned = true;
			}
			break;
		case autoRight:
			// Put custom auto code here
			if(!hasTurned && System.currentTimeMillis() - setTime > 1000){
				while(imu.getHeading() > -30){
					driveLeft1.set(-.5);
			        driveLeft2.set(-.5);
			        driveRight1.set(.5);
			        driveRight2.set(.5);
				}
				hasTurned = true;
			}
			break;
		case autoCenter:
			hasTurned = true;
		case defaultAuto:
		default:
			// Do Nothing
			if(System.currentTimeMillis() - setTime < 1000){
				driveLeft1.set(1);
		        driveLeft2.set(1);
		        driveRight1.set(1);
		        driveRight2.set(1);
			} else {
				driveLeft1.set(0);
		        driveLeft2.set(0);
		        driveRight1.set(0);
		        driveRight2.set(0);
			}
			return; //exit method
		}
		if(hasTurned){
			int numberOfContours = contours.length;
			if(numberOfContours > 0){
				if(contours[0][0] < 700){
					if(numberOfContours == 2){
						double centerX = (contours[0][1] + contours[1][1]) / 2;
						double driveOff = (Math.abs(centerX - 80)) / 80;
						if(centerX < 80){
							driveLeft1.set(1);
					        driveLeft2.set(1);
					        driveRight1.set(1 - driveOff);
					        driveRight2.set(1 - driveOff);
						} else {
							driveLeft1.set(1 - driveOff);
					        driveLeft2.set(1 - driveOff);
					        driveRight1.set(1);
					        driveRight2.set(1);
						}
					} else if(numberOfContours == 1) {
						double centerX = contours[0][1];
						double driveOff = (Math.abs(centerX - 80)) / 80;
						if(centerX < 80){
							driveLeft1.set(1);
					        driveLeft2.set(1);
					        driveRight1.set(1 - driveOff);
					        driveRight2.set(1 - driveOff);
						} else {
							driveLeft1.set(1 - driveOff);
					        driveLeft2.set(1 - driveOff);
					        driveRight1.set(1);
					        driveRight2.set(1);
						}
					} else {
						driveLeft1.set(1);
				        driveLeft2.set(1);
				        driveRight1.set(1);
				        driveRight2.set(1);
					}
				} else {
					driveLeft1.set(0);
			        driveLeft2.set(0);
			        driveRight1.set(0);
			        driveRight2.set(0);
				}
			}
			
			
			
		}else{
			driveLeft1.set(1);
	        driveLeft2.set(1);
	        driveRight1.set(1);
	        driveRight2.set(1);
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		double[][] contours = new double[publishableOutput.size()][5];
		for(int i = 0; i < publishableOutput.size(); i++){
			MatOfPoint thisContour = publishableOutput.get(i);
			Rect dimen = Imgproc.boundingRect(thisContour);
			contours[i][0] = Imgproc.contourArea(thisContour);
			contours[i][1] = dimen.x + (dimen.width / 2);
			contours[i][2] = dimen.y + (dimen.height / 2);
			contours[i][3] = dimen.height;
			contours[i][4] = dimen.width;
			/*
			 * SmartDashboard.putNumber("area" + i, Imgproc.contourArea(thisContour));
			 * SmartDashboard.putNumber("centerX" + i, dimen.x + (dimen.width / 2));
			 * SmartDashboard.putNumber("centerY" + i, dimen.y + (dimen.height / 2));
			 * SmartDashboard.putNumber("height" + i, dimen.height);
			 * SmartDashboard.putNumber("width" + i, dimen.width);
			 */
			//I commented this out because it's not useful unless console is needed
		}
		if(contours.length > 0){
			SmartDashboard.putNumber("Area: ", contours[0][0]);
			double centerX = 80;
			if(contours.length == 2){
				centerX = (contours[0][1] + contours[1][1]) / 2;
			} else
			if(contours.length == 1){
				centerX = contours[0][1];
			}
			if(centerX < 60){
				SmartDashboard.putString("Go", "Left");
			}else if (centerX > 100){
				SmartDashboard.putString("Go", "Right");
			}else{
				SmartDashboard.putString("Go", "Forward");
			}
		} else {
			SmartDashboard.putString("Go", "No Reflections Found!");
		}
		double turn = .9 *Joystick1.getRawAxis(6); //right joystick x-axis
        double speed = .9 * Joystick1.getRawAxis(1); //left joystick y-axis
        boolean pullForward = Joystick1.getRawButton(5); //left bumper
        boolean pullBackward = Joystick1.getRawButton(6); //right bumper
        boolean ShooterOn = Joystick2.getRawButton(2); //B button
        boolean armUp = Joystick2.getRawButton(5); //left bumper
        boolean armDown = Joystick2.getRawButton(6); //right bumper
        boolean Climber = Joystick1.getRawButton(2); //B button
        boolean Forklift = Joystick1.getRawButton(1); //A button
        boolean Break = Joystick1.getRawButton(3); //X button
        double leftSpeed = speed + turn;
        double rightSpeed = speed - turn;
        if (leftSpeed > 1){
        	leftSpeed = 1;
        } else if (leftSpeed < -1){
        	leftSpeed = -1;
        }
        if (rightSpeed > 1){
    		rightSpeed = 1;
        } else if (rightSpeed < -1){
        	rightSpeed = -1;
        }
        driveLeft1.set(leftSpeed);
        driveLeft2.set(leftSpeed);
        driveRight1.set(rightSpeed);
        driveRight2.set(rightSpeed);
        if(Forklift && !LastForklift){
        	desiredEncoder = !desiredEncoder;
        }
        System.out.println(desiredEncoder);
        if(!desiredEncoder && forkliftDown.get()){
        	ForkliftMotor.set(1);
        }else if(desiredEncoder && forkliftUp.get()){
        	ForkliftMotor.set(-1);
        }else{
        	ForkliftMotor.set(0);
        }
        if(Climber){
        	ClimberLeft.set(1);
        	ClimberRight.set(1);
        }else{
        	ClimberLeft.set(0);
        	ClimberRight.set(0);
        }
        LastForklift = Forklift;
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		
	}
}

