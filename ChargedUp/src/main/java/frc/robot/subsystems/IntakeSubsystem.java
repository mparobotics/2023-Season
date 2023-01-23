// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.script.Bindings;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase 
{


  /** Creates a new IntakeSubsystem. */
  // creating motors
  private final CANSparkMax intakeMotorR = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_R_ID, MotorType.kBrushless);
  private final CANSparkMax intakeMotorL = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_L_ID, MotorType.kBrushless);

  //Inverting left motor so wheels spin in same direction 
  public IntakeSubsystem()
  {
  intakeMotorR.setInverted(false);
  intakeMotorL.setInverted(true);
  }
  
  //setting speed of motors for intake
  public CommandBase intakeForward()
  {
    return runOnce(
    () -> {
      /* one-time action goes here*/
      intakeMotorL.set(IntakeConstants.INTAKE_SPEED);
      intakeMotorR.set(IntakeConstants.INTAKE_SPEED);
    }
    );
  }
  //setting speed of motors for outtake
  public CommandBase outtakeForward()
  {
    return runOnce(
    () -> {
      /* one-time action goes here*/
      intakeMotorL.set(IntakeConstants.OUTTAKE_SPEED);
      intakeMotorR.set(IntakeConstants.OUTTAKE_SPEED);
    }
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
