// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class OverrideElevatorMax extends CommandBase {
  private final ElevatorSubsystem m_ElevatorSubsystem;
  /** Command to continue moving the elevator beyond the maximum setpoint (at a slower speed). */
  public OverrideElevatorMax(ElevatorSubsystem elevSub) {
    m_ElevatorSubsystem = elevSub;
    addRequirements(elevSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //disable the maximum height limit
    m_ElevatorSubsystem.overrideMax = true;
    //move at a slower speed so we don't break the robot
    m_ElevatorSubsystem.setElevatorSpeed(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop the motor when the button is released
    m_ElevatorSubsystem.overrideMax = false;
    m_ElevatorSubsystem.setElevatorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
