// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  //drive base motors
  private final WPI_TalonSRX motorFR = new WPI_TalonSRX(DriveConstants.MOTOR_FR_ID);
  private final WPI_TalonSRX motorFL = new WPI_TalonSRX(DriveConstants.MOTOR_FL_ID);
  private final WPI_TalonSRX motorBR = new WPI_TalonSRX(DriveConstants.MOTOR_BR_ID);
  private final WPI_TalonSRX motorBL = new WPI_TalonSRX(DriveConstants.MOTOR_BL_ID);
  //encoders to measure driving speed
  //public final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
  //public final int kCPR = 8192;
  motorFL.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,10);
  motorFR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,10);
  public RelativeEncoder encoderL = motorFL.
  public RelativeEncoder encoderR = 
  
  
  //setting ramp

  //create motor controller groups
  private final MotorControllerGroup SCG_R = new MotorControllerGroup(motorFR, motorBR);
  private final MotorControllerGroup SCG_L = new MotorControllerGroup(motorFL, motorBL);

  //differential drive to control the motors
  private final DifferentialDrive differentialDrive = new DifferentialDrive(motorFL, motorFR);
  


  
  public WPI_Pigeon2 pigeon = new WPI_Pigeon2(DriveConstants.PIGEON_ID);
  //odometry object 
  private final DifferentialDriveOdometry m_odometry;
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
