// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; 

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  //creating the motors
  private final WPI_TalonSRX motorFL = new WPI_TalonSRX(Constants.MOTOR_FL_ID);
  private final WPI_TalonSRX motorBL = new WPI_TalonSRX(Constants.MOTOR_BL_ID);
  private final WPI_TalonSRX motorFR = new WPI_TalonSRX(Constants.MOTOR_FR_ID);
  private final WPI_TalonSRX motorBR = new WPI_TalonSRX(Constants.MOTOR_BR_ID);
  /*
  private final CANCoder enocderL = new CANCoder(Constants.ENCODER_L_ID);
  private final CANCoder enocderR = new CANCoder(Constants.ENCODER_R_ID);
  */
  
  DifferentialDrive differentialDrive = new DifferentialDrive(motorFL,motorFR);

  public DriveSubsystem() {
    motorBR.follow(motorFR); //make back motors follow front motors
    motorBL.follow(motorFL);
    //invert left motor and keep right motor uninverted
    motorFL.setInverted(true);
    motorFR.setInverted(false);
    //set following motors
    motorBL.setInverted(InvertType.FollowMaster);
    motorBR.setInverted(InvertType.FollowMaster);

    motorFL.configOpenloopRamp(0.4);
    motorFR.configOpenloopRamp(0.4);
    motorBL.configOpenloopRamp(0.4);
    motorBR.configOpenloopRamp(0.4);


  
  }

  public void setDriveSpeedTank(double leftSpeed, double rightSpeed){
    //speed limiter
    leftSpeed *= Constants.DRIVE_SPEED;
    rightSpeed *= Constants.DRIVE_SPEED;
    //deadbanding 
    if (Math.abs(leftSpeed) < .1){leftSpeed = 0;}
    if (Math.abs(rightSpeed) < .1){rightSpeed = 0;}
    
    differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }
  public void setDriveSpeedArcade(double forwardSpeed, double turnSpeed){
    
    forwardSpeed *= Constants.DRIVE_SPEED;
    turnSpeed *= Constants.TURN_SPEED;
    
    differentialDrive.arcadeDrive(forwardSpeed, turnSpeed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}