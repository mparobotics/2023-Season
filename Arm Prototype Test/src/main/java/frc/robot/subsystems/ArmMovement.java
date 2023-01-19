// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmMovement extends SubsystemBase {
  /** Creates a new ArmMovement. */
  WPI_TalonFX armMotor = new WPI_TalonFX(Constants.ARM_ID);
  public ArmMovement() {
    
  }

  public void moveArm(double speed){
    double m_speed = speed;
    armMotor.set(m_speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
