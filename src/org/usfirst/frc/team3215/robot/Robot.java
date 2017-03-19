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
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
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
	// The joysticks and their respective USB ports on the driver station
	Joystick Joystick1 = new Joystick(1);
	Joystick Joystick2 = new Joystick(0);
	final String defaultAuto = "Default";
	final String autoLeft = "left goal";
	final String autoCenter = "center goal";
	final String autoRight = "right goal";
	final String defaultTriple = "defaultTriple";

	String autoSelected;
	SpeedController driveLeft1 = new Victor(0);
	SpeedController driveLeft2 = new Victor(1);
	SpeedController driveRight1 = new Victor(2);
	SpeedController driveRight2 = new Victor(3);
	SpeedController Climber = new Victor(4);
	SpeedController ForkliftMotor = new Victor(5);
	BuiltInAccelerometer accel = new BuiltInAccelerometer();
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
	long setTime = 0, initTime;
	boolean hasTurned = false;
	DigitalOutput led = new DigitalOutput(2);
	boolean goLeft = true;
	boolean init = true;
	double slowLeft = 1;
	double slowRight = 1;
	final String hasturned = "has turned";
	final String turning = "turning";
	double targetAngle = 0.0;
	long headingCheckTime = 0;
	long accelerationCheckTime = 0;
	double lastHeading = 0;
	double leftDriveSpeed = 0;
	double rightDriveSpeed = 0;
	boolean ArrayListisNew = false;
	final Object cameraMutex = new Object();
	boolean canSetExposure = true;
	private long imuCheckTime = 0;
	private double initialImuAngle = 0;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		led.set(false);
		driveRight1.setInverted(false);
		driveRight2.setInverted(false);
		driveLeft1.setInverted(true);
		driveLeft2.setInverted(true);
		defaultValue = new double[0];
		ultra = new Ultrasonic(0, 1);
		ultra.setAutomaticMode(true);
		imu = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS, BNO055.vector_type_t.VECTOR_EULER);
		while (!imu.isInitialized())
			;
		cal = imu.getCalibration();

		System.out.println("\tCALIBRATION: Sys=" + cal.sys

				+ " Gyro=" + cal.gyro + " Accel=" + cal.accel

				+ " Mag=" + cal.mag);
		visionThread = new Thread(() -> {
			// Get the UsbCamera from CameraServer
			camera = CameraServer.getInstance().startAutomaticCapture();
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
				if(System.currentTimeMillis() - imuCheckTime > 100){
					imuCheckTime  = System.currentTimeMillis();
					initialImuAngle  = ((initialImuAngle * .9) + imu.getHeading() * .1);
				}
				if (cvSink.grabFrame(mat) == 0) {
					// Send the output the error.
					outputStream.notifyError(cvSink.getError());
					// skip the rest of the current iteration
					return;
				}
				grip.process(mat);
				synchronized (cameraMutex) {
					publishableOutput = grip.convexHullsOutput();
					ArrayListisNew = true;
				}
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}

		});
		visionThread.setDaemon(true);
		visionThread.start();
		chooser.addDefault("center", defaultAuto);
		chooser.addObject("left goal", autoLeft);
		chooser.addObject("right goal", autoRight);
		chooser.addObject("line", "cross Line");
		chooser.addObject("nothing", "aaaaaaaaaaaaaaaaa");
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
		System.out.println("Auto selected: " + autoSelected);
		led.set(true);
		init = true;
		targetAngle = initialImuAngle;
		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		SmartDashboard.putNumber("angle", imu.getHeading());
		SmartDashboard.putNumber("distance", ultra.getRangeInches());
		SmartDashboard.putNumber("setTime", setTime);
		SmartDashboard.putNumber("Current Time", System.currentTimeMillis());
		SmartDashboard.putBoolean("init", init);
		double[][] contours;
		if (ArrayListisNew) { 
			ArrayListisNew = false;
			synchronized (cameraMutex) {
				ArrayList<MatOfPoint> contourArray = publishableOutput;

				try {
					if (contourArray != null && contourArray.size() > 0) {
						contours = new double[contourArray.size()][5];
						SmartDashboard.putNumber("Contours", contourArray.size());

						for (int i = 0; i < contourArray.size(); i++) {
							MatOfPoint thisContour = contourArray.get(i);
							Rect dimen = Imgproc.boundingRect(thisContour);
							contours[i][0] = Imgproc.contourArea(thisContour);
							contours[i][1] = dimen.x + (dimen.width / 2);
							contours[i][2] = dimen.y + (dimen.height / 2);
							contours[i][3] = dimen.height;
							contours[i][4] = dimen.width;
							/*
							 * SmartDashboard.putNumber("area" + i,
							 * Imgproc.contourArea(thisContour));
							 * SmartDashboard.putNumber("centerX" + i, dimen.x +
							 * (dimen.width / 2));
							 * SmartDashboard.putNumber("centerY" + i, dimen.y +
							 * (dimen.height / 2));
							 * SmartDashboard.putNumber("height" + i,
							 * dimen.height); SmartDashboard.putNumber("width" +
							 * i, dimen.width);
							 */
							// I commented this out because it's not useful
							// unless
							// console is needed
						}
					} else {
						contours = new double[1][5];
						contours[0][0] = 0;
						contours[0][1] = 0;
						contours[0][2] = 0;
						contours[0][3] = 0;
						contours[0][4] = 0;
					}
				} catch (Exception e) {
					System.out.println("Error! There are no contours and publishableoutput was aceessed");

					e.printStackTrace();
					contours = new double[1][5];
					contours[0][0] = 0;
					contours[0][1] = 0;
					contours[0][2] = 0;
					contours[0][3] = 0;
					contours[0][4] = 0;
				}
			}
		} else {
			contours = new double[1][5];
			contours[0][0] = 0;
			contours[0][1] = 0;
			contours[0][2] = 0;
			contours[0][3] = 0;
			contours[0][4] = 0;
		}

		if (init) {
			setTime = System.currentTimeMillis();
			init = false;
		}
		switch (autoSelected) {
		case autoLeft:
			// Put custom auto code here
			
			if (System.currentTimeMillis() - setTime < 2000) {
				PID(targetAngle, false);
			} else {
				autoSelected = turning;
				targetAngle += 60.0;
			}
			break;
		case autoRight:
			// Put custom auto code here
			
			if (System.currentTimeMillis() - setTime < 2000) {
				PID(targetAngle, false);
			} else {
				autoSelected = turning;
				targetAngle -= 60.0;
			}
			break;
		case turning:
			if (Math.abs(imu.getHeading() - targetAngle) > 5.0) {
				PID(targetAngle, true);
			} else {
				autoSelected = "cross Line";
				setTime = System.currentTimeMillis();
			}
			break;
		case hasturned:
			int numberOfContours = contours.length;
			if (ultra.getRangeInches() > 6) {
				if (numberOfContours == 2) {
					double centerX = (contours[0][1] + contours[1][1]) / 2.0;
					if (centerX < 80) {
						targetAngle = imu.getHeading() + .5;
					} else if (centerX > 80) {
						targetAngle = imu.getHeading() - .5;
					}
				} else if (numberOfContours == 1 && contours[0][0] > 10) {
					if (contours[0][1] > 80) {
						targetAngle = imu.getHeading() - 20;
					} else {
						targetAngle = imu.getHeading() + 20;
					}
				}
				PID(targetAngle, true);
			} else {
				try {
					Thread.sleep(500);
				} catch (Exception e) {

				} finally {
					driveLeft1.set(0);
					driveLeft2.set(0);
					driveRight1.set(0);
					driveRight2.set(0);
				}
			}

			return;
		case defaultAuto:
			if (ultra.getRangeInches() < 7) {
				try {
					Thread.sleep(500);
				} catch (Exception e) {

				} finally {
					driveLeft1.set(0);
					driveLeft2.set(0);
					driveRight1.set(0);
					driveRight2.set(0);
				}
			} else {
				PID(targetAngle, true);
			}
			return;
		case "cross Line":
			// crosses the green line on the field.
			if (System.currentTimeMillis() - setTime < 4000) {
				PID(targetAngle, false);
			} else {
				driveLeft1.set(0);
				driveLeft2.set(0);
				driveRight1.set(0);
				driveRight2.set(0);
			}
			return; // exit method
		default:
			return;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		/*
		 * double[][] contours = new double[publishableOutput.size()][5];
		 * 
		 * for(int i = 0; i < publishableOutput.size(); i++){ MatOfPoint
		 * thisContour = publishableOutput.get(i); Rect dimen =
		 * Imgproc.boundingRect(thisContour); contours[i][0] =
		 * Imgproc.contourArea(thisContour); contours[i][1] = dimen.x +
		 * (dimen.width / 2); contours[i][2] = dimen.y + (dimen.height / 2);
		 * contours[i][3] = dimen.height; contours[i][4] = dimen.width;
		 *
		 * SmartDashboard.putNumber("area" + i,
		 * Imgproc.contourArea(thisContour)); SmartDashboard.putNumber("centerX"
		 * + i, dimen.x + (dimen.width / 2)); SmartDashboard.putNumber("centerY"
		 * + i, dimen.y + (dimen.height / 2)); SmartDashboard.putNumber("height"
		 * + i, dimen.height); SmartDashboard.putNumber("width" + i,
		 * dimen.width);
		 *
		 * //I commented this out because it's not useful unless console is
		 * needed }
		 * 
		 * if(contours.length == 2){ SmartDashboard.putNumber("Area: ",
		 * contours[0][0]); double centerX = (contours[0][1] + contours[1][1]) /
		 * 2; if(centerX < 60){ SmartDashboard.putString("Go", "Left"); }else if
		 * (centerX > 100){ SmartDashboard.putString("Go", "Right"); }else{
		 * SmartDashboard.putString("Go", "Forward"); } } else {
		 * SmartDashboard.putString("Go", "No Reflections Found!"); }
		 */
		SmartDashboard.putNumber("heading", imu.getHeading());
		if (canSetExposure) {
			try {
				camera.setExposureManual(75);
				canSetExposure = false;
			} catch (Exception e) {

			}
		}
		double turn = .9 * Joystick1.getRawAxis(4) * Math.abs(Joystick1.getRawAxis(4)); // right
																						// joystick
																						// x-axis
		double speed = .9 * Joystick1.getRawAxis(1) * Math.abs(Joystick1.getRawAxis(1)); // left
																							// joystick
																							// y-axis
		boolean VTEC = Joystick1.getRawButton(5); // left bumper
		boolean slowMode = Joystick1.getRawButton(6); // right bumper
		boolean ShooterOn = Joystick2.getRawButton(2); // B button
		boolean armUp = Joystick2.getRawButton(5); // left bumper
		boolean armDown = Joystick2.getRawButton(6); // right bumper
		boolean climber = Joystick1.getRawButton(2); // B button
		boolean ForkliftUp = Joystick1.getRawButton(1); // A button
		boolean ForkliftDown = Joystick1.getRawButton(3); // X button
		boolean idle = Joystick1.getRawButton(4); // Y button
		boolean inverse = Joystick1.getRawButton(9); // left joystick button
		if (speed < 0 && inverse) {
			turn = -turn;
		}
		double leftSpeed = speed - turn;
		double rightSpeed = speed + turn;
		if (VTEC) {
			SmartDashboard.putString("VTEC", "Just Kicked in Yo");
		} else if (slowMode) {
			leftSpeed = leftSpeed * .2;
			rightSpeed = rightSpeed * .2;
		} else {
			leftSpeed = leftSpeed * .3;
			rightSpeed = rightSpeed * .3;
		}

		if (System.currentTimeMillis() - accelerationCheckTime > 10) {
			double accelRate = (System.currentTimeMillis() - accelerationCheckTime) / 1000.0;
			if (accelRate > .05) {
				accelRate = .05;
			}
			
			if ((Math.abs(Math.signum(leftDriveSpeed) + Math.signum(leftSpeed)) < 0.05) || Math.abs(leftSpeed) < .05) {
				leftDriveSpeed = 0.0;
			} else {
				if (leftDriveSpeed > leftSpeed) {
					leftDriveSpeed -= accelRate;
				} else {
					leftDriveSpeed += accelRate;
				}
			}
			if ((Math.abs(Math.signum(rightDriveSpeed) + Math.signum(rightSpeed)) < 0.05) || Math.abs(rightSpeed) < .05) {
				rightDriveSpeed = 0.0;
			} else {
				if (rightDriveSpeed > rightSpeed) {
					rightDriveSpeed -= accelRate;
				} else {
					rightDriveSpeed += accelRate;
				}
			}
			accelerationCheckTime = System.currentTimeMillis();
		}
		if (leftDriveSpeed > 1) {
			leftDriveSpeed = 1;
		} else if (leftDriveSpeed < -1) {
			leftDriveSpeed = -1;
		}
		if (rightDriveSpeed > 1) {
			rightDriveSpeed = 1;
		} else if (rightDriveSpeed < -1) {
			rightDriveSpeed = -1;
		}
		driveLeft1.set(leftDriveSpeed);
		driveLeft2.set(leftDriveSpeed);
		driveRight1.set(rightDriveSpeed);
		driveRight2.set(rightDriveSpeed);

		if (ForkliftUp) {
			if (VTEC) {
				ForkliftMotor.set(0.5);
			} else if (slowMode) {
				ForkliftMotor.set(0.2);
			} else {
				ForkliftMotor.set(.4);
			}
		} else if (ForkliftDown) {
			if (VTEC) {
				ForkliftMotor.set(-0.5);
			} else if (slowMode) {
				ForkliftMotor.set(-0.2);
			} else {
				ForkliftMotor.set(-.4);
			}

		} else {
			ForkliftMotor.set(0);
		}

		if (climber) {
			if (VTEC) {
				Climber.set(1);
			} else if (slowMode) {
				Climber.set(0.3);
			} else {
				Climber.set(0.6);
			}
		} else {
			Climber.set(0);
		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		PID(0, false);
	}

	private void PID(double targetAngle, boolean canGoNegative) {
		double Heading = imu.getHeading();
		double speedAdd = 0.0;
		if (Math.abs(Heading - targetAngle) > 15.0) {

			if (Heading > targetAngle) {
				if(canGoNegative){
					driveLeft1.set(.2);
					driveLeft2.set(.2);
				} else {
					driveLeft1.set(0);
					driveLeft2.set(0);
				}
				driveRight1.set(-.2);
				driveRight2.set(-.2);
				slowLeft = 0.5;
				slowRight = 1.0;
			} else if (Heading < targetAngle) {
				driveLeft1.set(-.2);
				driveLeft2.set(-.2);
				if(canGoNegative){
					driveRight1.set(.2);
					driveRight2.set(.2);
				} else {
					driveRight1.set(0);
					driveRight2.set(0);
				}
				slowLeft = 1.0;
				slowRight = 0.5;
			}
		} else {
			if (System.currentTimeMillis() - headingCheckTime > 10) {
				double elapsedTime = (System.currentTimeMillis() - headingCheckTime) / 3000.0;
				if (elapsedTime > .05) {
					elapsedTime = .05;
				}
				if (Heading > targetAngle) {
					if (Heading > lastHeading) {
						slowLeft -= elapsedTime;
					} else {
						slowLeft -= elapsedTime / 3.0;
					}
				} else if (Heading < targetAngle) {
					if (Heading < lastHeading) {
						slowRight -= elapsedTime;
					} else {
						slowRight -= elapsedTime / 3.0;
					}
				}
				lastHeading = Heading;
				headingCheckTime = System.currentTimeMillis();
			}

			if (slowLeft > slowRight) {
				speedAdd = 1.0 - slowLeft;
			} else {
				speedAdd = 1.0 - slowRight;
			}
			slowLeft = slowLeft + speedAdd;
			slowRight = slowRight + speedAdd;
			if (slowLeft > 1.0) {
				slowLeft = 1.0;
			}
			if (slowRight > 1.0) {
				slowRight = 1.0;
			}
			if (slowLeft <= 0.0) {
				slowLeft = 0.0;
			}
			if (slowRight <= 0.0) {
				slowRight = 0.0;
			}
			driveLeft1.set(-.3 * slowLeft);
			driveLeft2.set(-.3 * slowLeft);
			driveRight1.set(-.3 * slowRight);
			driveRight2.set(-.3 * slowRight);
		}
		// System.out.println("slowLeft = " + slowLeft);
		// System.out.println("slowRight = " + slowRight);
		// System.out.println("Heading = " + Heading);
	}
}
