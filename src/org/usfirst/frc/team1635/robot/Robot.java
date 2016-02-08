
package org.usfirst.frc.team1635.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team1635.robot.commands.Autonomous;
import org.usfirst.frc.team1635.robot.commands.Autonomous2;
import org.usfirst.frc.team1635.robot.commands.IsVision;
import org.usfirst.frc.team1635.robot.subsystems.DoubleCamera;
import org.usfirst.frc.team1635.robot.subsystems.DriveTrain;
//import org.usfirst.frc.team1635.robot.subsystems.Sonar;
import org.usfirst.frc.team1635.robot.subsystems.VisionProcessing;

import com.ni.vision.NIVision.Image;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {  

	// Subsystems Declarations
	public static DriveTrain driveTrain;
	public static VisionProcessing visionprocessing;
	public static DoubleCamera doublecamera;
	// public static DoubleCamera_Separate doublecamera2;
	// public static Sonar sonar;
	public static OI oi;

	CameraServer server;

	Command autonomousCommand;

	Image frame;
	Image binaryFrame;
	int imaqError;

	// Controller contr = new Controller(Config.Controller.chn,
	// Config.Controller.maxButtons, Config.Controller.linearity);
	// Joystick contr = oi.getJoystick();
	// CameraFeeds cameraFeeds = new CameraFeeds(contr);

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {

		driveTrain = new DriveTrain();
		visionprocessing = new VisionProcessing();
		

		doublecamera = new DoubleCamera();
		

		// = new DoubleCamera_Separate();
		// sonar = new Sonar();

		// instantiate the command used for the autonomous period
		autonomousCommand = new Autonomous();
		
				
		 
		
		oi = new OI();

		SmartDashboard.putData("VisonTesting", new IsVision());

		// server = CameraServer.getInstance();
		// server.setQuality(15);
		// server.startAutomaticCapture("cam0");

	}

	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		driveTrain.reset();
	}

	public void autonomousInit() {
		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();

	}

	/**
	 * This function is called when the disabled button is hit. You can use it
	 * to reset subsystems before shutting down.
	 */
public void disbledInit() {
		driveTrain.reset();
		
		// doublecamera.

	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		log();
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
	}

	private void log() {
		driveTrain.log();
		visionprocessing.log();
	}
}
