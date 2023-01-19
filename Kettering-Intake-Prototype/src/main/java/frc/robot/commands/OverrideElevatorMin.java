// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class OverrideElevatorMin extends CommandBase {
  private final ElevatorSubsystem m_ElevatorSubsystem;
  /** Command to continue moving the elevator beyond the minimum setpoint (at a slower speed). */
  public OverrideElevatorMin(ElevatorSubsystem elevSub) {
    m_ElevatorSubsystem = elevSub;
    addRequirements(elevSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //disable minimum height limit
    m_ElevatorSubsystem.overrideMin = true;
    //move at a slower speed so we don't break anything
    m_ElevatorSubsystem.setElevatorSpeed(-.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop the motors when the button is released
    m_ElevatorSubsystem.overrideMin = false;
    m_ElevatorSubsystem.setElevatorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
