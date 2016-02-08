package org.usfirst.frc.team1635.robot.subsystems;

import org.usfirst.frc.team1635.robot.RobotMap;
import org.usfirst.frc.team1635.robot.commands.DriveWithJoystick;

import NavxMXP.AHRS;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.USBCamera;
import edu.wpi.first.wpilibj.Ultrasonic;

/**
 * @author Xxx_MrProLi_xxX & Bot_Miguel & Simranj69t
 * 
 *
 */
public class DriveTrain extends Subsystem {

	RobotDrive robotDrive;
	Joystick stick;
	// int frontLeft, frontRight, rearLeft, rearRight;
	private SpeedController frontLeft, backLeft, frontRight, backRight;
	boolean onTarget;

	SerialPort serial_port;
	
	AnalogInput PressureSensor;
	

	// IMU imu; // This class can be used w/nav6 and navX MXP.
	// IMUAdvanced imu; // This class can be used w/nav6 and navX MXP.
	AHRS imu; // This class can only be used w/the navX MXP.
	boolean first_iteration;

	
	
	// Ultrasonic sonar;

	AnalogInput sonar;
	double DistanceToStop;
	double Degrees;

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public DriveTrain() {
		super();

		// DigitalInput echoChannel;
		// DigitalInput pingChannel;

		// sonar = new Ultrasonic(0, 1);
		sonar = new AnalogInput(0);
		
		PressureSensor = new AnalogInput(3);
		 

		// frontLeft = RobotMap.frontLeftChannel;
		// frontRight = RobotMap.frontRightChannel;
		// rearLeft = RobotMap.rearLeftChannel;
		// rearRight = RobotMap.rearRightChannel;

		frontLeft = new Victor(RobotMap.frontLeftChannel);
		backLeft = new Victor(RobotMap.rearLeftChannel);
		frontRight = new Victor(RobotMap.frontRightChannel);
		backRight = new Victor(RobotMap.rearRightChannel);

		robotDrive = new RobotDrive(frontLeft, backLeft, frontRight, backRight);
		robotDrive.setExpiration(0.1);

		robotDrive.setInvertedMotor(MotorType.kFrontRight, true); // invert the
																	// left side
																	// motors
		robotDrive.setInvertedMotor(MotorType.kRearRight, true);
		// robotDrive.setInvertedMotor(MotorType.kRearLeft, true);

		try {
			serial_port = new SerialPort(57600, SerialPort.Port.kMXP);

			byte update_rate_hz = 50;
			// imu = new IMU(serial_port,update_rate_hz);
			// imu = new IMUAdvanced(serial_port,update_rate_hz);
			imu = new AHRS(serial_port, update_rate_hz);
		} catch (Exception ex) {
			ex.printStackTrace();
			
		}

		if (imu != null) {
			LiveWindow.addSensor("IMU", "Gyro", imu);
		}
		first_iteration = true;

		// When calibration has completed, zero the yaw
		// Calibration is complete approaximately 20 seconds
		// after the robot is powered on. During calibration,
		// the robot should be still

		boolean is_calibrating = imu.isCalibrating();
		if (first_iteration && !is_calibrating) {
			Timer.delay(0.3);
			imu.zeroYaw();
			first_iteration = false;
		}

	}

	/*
	 * to obtain the gyro value in degrees
	 */
	public double obtainYaw() {
		return imu.getYaw();

	}

	public float convertToFarenheit() {
		float celcius = imu.getTempC();
		float farenheit = (float) (celcius * 1.8 + 32);
		return farenheit;

	}

	public void log() {
		SmartDashboard.putNumber("Gyro", obtainYaw());
		SmartDashboard.putNumber("Temperature", convertToFarenheit());
		SmartDashboard.putNumber("FrontLeftMotorSpd", frontLeft.get());
		SmartDashboard.putNumber("FrontRightMotorSpd", frontRight.get());
		SmartDashboard.putNumber("RearLeftMotorSpd", backLeft.get());
		SmartDashboard.putNumber("RearRightMotorSpd", backRight.get());
		//SmartDashboard.putNumber("DistanceSonar", getDistanceSonar());
		SmartDashboard.putNumber("PREssure", PressureSensor.getValue());
		SmartDashboard.putNumber("displacement", imu.getDisplacementY());
		//SmartDashboard.putBoolean("button5", );
		//System.out.println(getDistanceSonar());
		// SmartDashboard.putNumber("", getDistanceSonar());

	}

	// codes for the mecanum drive
	public void mecanumDrive(Joystick joy) {
		robotDrive.mecanumDrive_Cartesian(-joy.getX() * 0.6, -joy.getY() * 0.6, -joy.getRawAxis(3) * 0.6, 0);
		//robotDrive.mecanumDrive_Cartesian(-joy.getX()*3 , -joy.getY()*3, -joy.getRawAxis(3)*3, 0);
		// robotDrive.mecanumDrive_Cartesian(-joy.getX() * 0.6, -joy.getY() *
		// 0.6,
		// 0, 0);
		SmartDashboard.putNumber("Xdirection_values", joy.getX());
		SmartDashboard.putNumber("rotation_xxx_value", joy.getRawAxis(3));
		SmartDashboard.putBoolean("button5", joy.getRawButton(5));
		SmartDashboard.putBoolean("button6", joy.getRawButton(6));
		SmartDashboard.putBoolean("button1", joy.getRawButton(1));
		SmartDashboard.putBoolean("button2", joy.getRawButton(2));
		SmartDashboard.putBoolean("button3", joy.getRawButton(3));
		SmartDashboard.putBoolean("button4", joy.getRawButton(4));
		SmartDashboard.putBoolean("button7", joy.getRawButton(7));
		SmartDashboard.putBoolean("button8", joy.getRawButton(8));

	}

	

	/*
	 * 
	 * this code is designed to obtain the distance from the
	 * xxx_MLG_NEXus_LI_XXX_#360n0SC()P3
	 */
	public double getDistanceSonar() {// sonar is weird: it has a deadzone of 12
										// inches :( ; distance will always be 0
										// within the deadzone
		// double distance = sonar.getRangeInches();
		// double distance = sonar.getValue();

		//double valueToInches = 0.125;
		double valueToInches_2 = 1 / 13.08;// 14.45,20.5
		double distanceX = sonar.getAverageValue();
		//double distanceX = sonar.getVoltage();
		double distance = (distanceX - 237) * valueToInches_2 + 12;
		
		//double distance = (distanceX - 3136.5 ) * valueToInches_2 +172;// Troubleshooting Sonar Sensor 															// convert
																	// voltage
																	// into
																	// inches
		//int distanceInt=(int) distance;
		return distance;

	}

	public void setDistToStop(double dist_) {
		this.DistanceToStop = dist_;
	}

	public void stopRobotAtDistance() {

		double distanceObtained;

		// getDistanceSonar()= 0 ;
		distanceObtained = getDistanceSonar();
		
		//SmartDashboard.putNumber("DistanceSonar", getDistanceSonar());
		if (distanceObtained <= DistanceToStop) {
			robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
			onTarget = true;
			System.out.println(" is robot finished " + true);
		} else {
			// Timer.delay(0.05);// compensate for sonar reading delays
			robotDrive.mecanumDrive_Cartesian(0, 0.2, 0, 0);
			// xxx_delayMecanum(0, 0.2, 0, 0);
			System.out.println(" is robot finished " + false);
		}
	}

	public void mecanumWithParameters(double x_axis, double y_axis, double rotaion) {
		robotDrive.mecanumDrive_Cartesian(-x_axis * 0.6, -y_axis * 0.6, -rotaion * 0.6, 0);

	}

	public void initDefaultCommand() {
		this.setDefaultCommand(new DriveWithJoystick());

		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public boolean isOnTarget() {
		return onTarget;
	}

	public void reset() {
		imu.zeroYaw();
		//sonar.resetAccumulator();
		onTarget = false;
		imu.resetDisplacement();
		// getDistanceSonar()=0;

	}

	public void stop() {
		robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
		reset();
	}

	// public double rotate (double sub){
	// return imu.;
	// }

	public void setRotation(double deg) {
		// imu.zeroYaw();
		this.Degrees = deg;
	}

	public void Rotate() {
		double degreeObtained = obtainYaw();
		if (obtainYaw() > degreeObtained + 30 && obtainYaw() < degreeObtained - 30) {
			robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
			onTarget = true;
		} else {
			robotDrive.mecanumDrive_Cartesian(0, 0, 0.25, 0);
		}

	}

}
