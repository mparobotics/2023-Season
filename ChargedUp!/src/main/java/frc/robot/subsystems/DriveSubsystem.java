// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  //motors
  private static final CANSparkMax motorFR = new CANSparkMax(Constants.MOTOR_FR_ID, MotorType.kBrushless);
  private static final CANSparkMax motorFL = new CANSparkMax(Constants.MOTOR_FL_ID, MotorType.kBrushless);
  private static final CANSparkMax motorBR = new CANSparkMax(Constants.MOTOR_BR_ID, MotorType.kBrushless);
  private static final CANSparkMax motorBL = new CANSparkMax(Constants.MOTOR_BL_ID, MotorType.kBrushless);
  //differential drive
  private static final DifferentialDrive differentialDrive = new DifferentialDrive(motorFR, motorFL);

  //PH compressor
  private static final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

  //solenoid
  private static final Solenoid shiftSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.SOLENOID_CHANNEL);
  


  public DriveSubsystem() {
    //back motors follow front motors
    motorBR.follow(motorFR);
    motorBL.follow(motorFL);
    //invert left motors
    motorFL.setInverted(true);
    motorBL.setInverted(true);
    //turn on compressor
    compressor.enableDigital();
    //set solenoid to OFF
    shiftSolenoid.set(false);
  }

  public void setDriveSpeedArcade(double sForward, double sTurning){
    differentialDrive.arcadeDrive(sForward, sTurning);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
