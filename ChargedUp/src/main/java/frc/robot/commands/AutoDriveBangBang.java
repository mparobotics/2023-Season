// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveBangBang extends CommandBase {
  private DriveSubsystem m_driveSubsystem;
  private double m_setpoint;
  private double startEncoderL;
  private double startEncoderR;
  private double m_speed;
  /** Creates a new AutoDriveBangBang. */
  public AutoDriveBangBang(DriveSubsystem driveSub, double setpoint, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSub;
    m_setpoint = setpoint;
    m_speed = speed;
    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubsystem.encoderReset();
    startEncoderL = m_driveSubsystem.getEncoderPositionL();
    startEncoderR = m_driveSubsystem.getEncoderPositionR();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.driveStraight(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.setDriveSpeedArcade(0, 0);
  }                           

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_setpoint > 0) {
    return(m_driveSubsystem.getEncoderPositionL() - startEncoderL >= m_setpoint);}
    else {return ((m_driveSubsystem.getEncoderPositionL() + startEncoderL) <= m_setpoint);

    }
  }
}
