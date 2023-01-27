// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  intakeMotorR.setInverted(true);
  intakeMotorL.setInverted(false);

  intakeMotorL.setIdleMode(IdleMode.kBrake);
  intakeMotorR.setIdleMode(IdleMode.kBrake);
  }
  
  //setting speed of motors for intake
  public void intakeForward(double speed)
  {
      /* one-time action goes here*/
      intakeMotorL.set(speed);
      intakeMotorR.set(speed);
    }
  //setting speed of motors for outtake
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
