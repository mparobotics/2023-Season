// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
  private static DriveSubsystem m_driveSubsystem;
  private DoubleSupplier m_sForward;
  private DoubleSupplier m_sTurning;
  public ArcadeDrive(DriveSubsystem driveSub, DoubleSupplier sForward, DoubleSupplier sTurning) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_sForward = sForward;
    m_driveSubsystem = driveSub;
    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}