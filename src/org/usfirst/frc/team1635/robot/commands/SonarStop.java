package org.usfirst.frc.team1635.robot.commands;

import org.usfirst.frc.team1635.robot.Robot;
import org.usfirst.frc.team1635.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SonarStop extends Command {
	private double dist_;

    public SonarStop(double dist) {
    	this.dist_=dist;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	requires(Robot.driveTrain);
    	//requires(Robot.sonar);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//Robot.driveTrain.reset();
    	Robot.driveTrain.setDistToStop(dist_+5);
    	//Robot.sonar.setDistToStop(dist_);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveTrain.stopRobotAtDistance();
  
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
       
      return Robot.driveTrain.isOnTarget();
    	
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.stop();
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
