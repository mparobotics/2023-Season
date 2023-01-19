package frc.robot.commands;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;


import frc.robot.subsystems.DriveSubsystem;


public class ArcadeDrive extends CommandBase {
  /** Creates a new TankDrive. */
  private final DriveSubsystem m_DriveSubsystem;
  private final DoubleSupplier m_turnSpeed;
  private final DoubleSupplier m_forwardSpeed;
  

  public ArcadeDrive(DriveSubsystem driveSub,DoubleSupplier turnSpeed, DoubleSupplier forwardSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_DriveSubsystem = driveSub;
    this.m_turnSpeed = turnSpeed;
    this.m_forwardSpeed = forwardSpeed;
    addRequirements(driveSub);
  }

 
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveSubsystem.setDriveSpeedArcade(m_forwardSpeed.getAsDouble(), m_turnSpeed.getAsDouble());
  }
}

