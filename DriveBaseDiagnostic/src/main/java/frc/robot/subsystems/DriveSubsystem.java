// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax motorFR = new CANSparkMax(DriveConstants.MOTOR_FR_ID, MotorType.kBrushless);
  private final CANSparkMax motorFL = new CANSparkMax(DriveConstants.MOTOR_FL_ID, MotorType.kBrushless);
  private final CANSparkMax motorBR = new CANSparkMax(DriveConstants.MOTOR_BR_ID, MotorType.kBrushless);
  private final CANSparkMax motorBL = new CANSparkMax(DriveConstants.MOTOR_BL_ID, MotorType.kBrushless);
  /** Creates a new DriveSubsystem. */
  public RelativeEncoder encoderFL = motorFL.getEncoder();
  public RelativeEncoder encoderFR = motorFR.getEncoder();
  public RelativeEncoder encoderBL = motorBL.getEncoder();
  public RelativeEncoder encoderBR = motorBR.getEncoder();
  public DriveSubsystem() {
    
  }


  public CommandBase RunFL() 
  {
    return runOnce(
      () -> {
        motorFL.setVoltage(12);
      });
  }
  
    public CommandBase RunFR() 
  {
    return runOnce(
      () -> {
        motorFR.setVoltage(12);
      });
  }
    public CommandBase RunBL() 
  {
    return runOnce(
      () -> {
        motorBL.setVoltage(12);
      });
  }
  
    public CommandBase RunBR() 
  {
    return runOnce(
      () -> {
        motorBR.setVoltage(12);
      });
  }

      public CommandBase StopMotors() 
  {
    return runOnce(
      () -> {
        motorFL.setVoltage(0);
        motorFR.setVoltage(0);
        motorBL.setVoltage(0);
        motorBR.setVoltage(0);
      });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("FL Encoder Velocity", encoderFL.getVelocity());
    SmartDashboard.putNumber("FR Encoder Velocity", encoderFR.getVelocity());
    SmartDashboard.putNumber("BL Encoder Velocity", encoderBL.getVelocity());
    SmartDashboard.putNumber("BR Encoder Velocity", encoderBR.getVelocity());
  }
}
