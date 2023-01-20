// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  //drive base motors
  private static final CANSparkMax motorFR = new CANSparkMax(Constants.DriveTrainConstants.MOTOR_FR_ID, MotorType.kBrushless);
  private static final CANSparkMax motorFL = new CANSparkMax(Constants.DriveTrainConstants.MOTOR_FL_ID, MotorType.kBrushless);
  private static final CANSparkMax motorBR = new CANSparkMax(Constants.DriveTrainConstants.MOTOR_BR_ID, MotorType.kBrushless);
  private static final CANSparkMax motorBL = new CANSparkMax(Constants.DriveTrainConstants.MOTOR_BL_ID, MotorType.kBrushless);
  //encoders to measure driving speed
  private static final RelativeEncoder encoderL = motorFL.getEncoder();
  private static final RelativeEncoder encoderR = motorFL.getEncoder();

  
  //differential drive
  private static final DifferentialDrive differentialDrive = new DifferentialDrive(motorFR, motorFL);

  //PH compressor
  private static final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

  //solenoids to control gear shifting
  private static final Solenoid shiftSolenoidL = new Solenoid(PneumaticsModuleType.REVPH, Constants.DriveTrainConstants.LEFT_SOLENOID_CHANNEL);
  private static final Solenoid shiftSolenoidR = new Solenoid(PneumaticsModuleType.REVPH, Constants.DriveTrainConstants.RIGHT_SOLENOID_CHANNEL);
  


  public DriveSubsystem() {
    //back motors follow front motors
    motorBR.follow(motorFR);
    motorBL.follow(motorFL);
    //invert left motors
    motorFL.setInverted(true);
    motorBL.setInverted(true);
    //dont invert right motors
    motorFR.setInverted(false);
    motorBR.setInverted(false);
    //turn on compressor
    compressor.enableDigital();
    //set solenoid to OFF
    shiftSolenoidL.set(false);
    shiftSolenoidR.set(false);
  }
  public void upShift(){
    shiftSolenoidL.set(true);
    shiftSolenoidR.set(true);
  }
  public void downShift(){
    shiftSolenoidL.set(false);
    shiftSolenoidR.set(false);
  }
  public void setDriveSpeedArcade(double sForward, double sTurning){
    
    
    differentialDrive.arcadeDrive(sForward * Constants.DriveTrainConstants.DRIVE_SPEED, sTurning * Constants.DriveTrainConstants.TURNING_SPEED);
    

  }
  
  public CommandBase ShiftUp(){
    return runOnce(() -> upShift());
  }
  public CommandBase ShiftDown(){
    return runOnce(() -> downShift());
  }
  @Override
  public void periodic() {
    //Automatic gear shifting
    //the speed of the left motor in RPMs
    double lvelocity = encoderL.getVelocity();
    //the speed of the right motor in RPMs
    double rvelocity = encoderR.getVelocity();
    //check if the robot is turning - if the speeds of the left and right motors are different
    boolean isTurning = Math.abs(lvelocity - rvelocity) > Constants.DriveTrainConstants.TURN_THRESHOLD;

    //check if automatic shifitng is enabling and the robot IS NOT turning
    if(Constants.DriveTrainConstants.AUTO_SHIFT_ENABLED && !isTurning){
      //if either motor exceeds the velocity threshold then shift into high gear
      if(Math.abs(lvelocity) > Constants.DriveTrainConstants.UPSHIFT_THRESHOLD
      || Math.abs(rvelocity) > Constants.DriveTrainConstants.UPSHIFT_THRESHOLD){
        upShift();
      }
      //if both motors are below the downshift threshold then shift down
      if(Math.abs(lvelocity) > Constants.DriveTrainConstants.DOWNSHIFT_THRESHOLD
      || Math.abs(rvelocity) > Constants.DriveTrainConstants.DOWNSHIFT_THRESHOLD){
        downShift();
      }
    }
  }
}
