package org.usfirst.frc.team1635.robot.subsystems;

import org.usfirst.frc.team1635.robot.Config;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DoubleCamera extends Subsystem {
	private  int camCenter;
	private  int camRight;
	private int curCam;
	private Image frame;
	private CameraServer server;
	private Joystick stick;
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	public DoubleCamera(){
//		// Get camera ids by supplying camera name ex 'cam0', found on roborio web interface
//        camCenter = NIVision.IMAQdxOpenCamera(Config.CameraFeeds.camNameCenter, NIVision.IMAQdxCameraControlMode.CameraControlModeController);
//        camRight = NIVision.IMAQdxOpenCamera(Config.CameraFeeds.camNameRight, NIVision.IMAQdxCameraControlMode.CameraControlModeController);
//        curCam = camCenter;
//        // Img that will contain camera img
//        frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
//        // Server that we'll give the img to
//        server = CameraServer.getInstance();
//        server.setQuality(Config.CameraFeeds.imgQuality);
//        server.setSize(0);// limit the resolution to 160*120

		
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    public void init()
	{
		changeCam(camCenter);
	}
	
	public void run()
	{
//		if(stick.getButton(Config.CameraFeeds.btCamCenter))
		if(stick.getRawButton(Config.CameraFeeds.btCamCenter))
			changeCam(camCenter);
		
		if(stick.getRawButton(Config.CameraFeeds.btCamRight))
			changeCam(camRight);
		
		updateCam();
	}
	
	public void runCamZero(){
		changeCam(camCenter);
		updateCam();
	}
	
	public void runCamOne(){
		changeCam(camRight);
		updateCam();
	}
	
	/**
	 * Stop aka close camera stream
	 */
	public void end()
	{
		NIVision.IMAQdxStopAcquisition(curCam);
		
	}
	
	/**
	 * Change the camera to get imgs from to a different one
	 * @param newId for camera
	 */
	public void changeCam(int newId)
    {
		NIVision.IMAQdxStopAcquisition(curCam);
    	NIVision.IMAQdxConfigureGrab(newId);
    	NIVision.IMAQdxStartAcquisition(newId);
    	curCam = newId;
    }
    
	/**
	 * Get the img from current camera and give it to the server
	 */
    public void updateCam()
    {
    	NIVision.IMAQdxGrab(curCam, frame, 1);
        server.setImage(frame);
    }
}

