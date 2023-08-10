// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAlign extends CommandBase {
  /** Creates a new AutoAlign. */
  private DriveSubsystem m_driveSubsystem;

  public AutoAlign(DriveSubsystem driveSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSub;
    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    boolean gamePieceFound;
    if (tv != 1) {gamePieceFound = false;}
    else {gamePieceFound = true;}

    if (gamePieceFound == true){
  
      //Robot.driveSubsystem.teleop(0, 0);
      double steering_adjust = 0.0f;

      if (tx > .5 ) //if to the right of center, turns left; to test deadzone 
      {
          steering_adjust = DriveConstants.ALIGN_Kp * tx - DriveConstants.min_command; //Kp is a number that sets the speed, and the min command is the minimum needed for it to react
          //steering_adjust = -.05; //to test speed and inversion
          m_driveSubsystem.setDriveSpeedArcade(0, steering_adjust); //turns the turret
          
      }
    
      else if (tx < -.5 ) //if to the left of center, turns right; to test deadzone
      {

          steering_adjust = DriveConstants.ALIGN_Kp * tx + DriveConstants.min_command;
          //steering_adjust = .05; //to test speed and inversion
          m_driveSubsystem.setDriveSpeedArcade(0, steering_adjust);//turns the turret
          
      }
  
      else{m_driveSubsystem.setDriveSpeedArcade(0, 0);}}
  

    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
