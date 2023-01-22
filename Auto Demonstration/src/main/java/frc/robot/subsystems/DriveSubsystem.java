// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj.SPI;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  //drive base motors
  private final CANSparkMax motorFR = new CANSparkMax(Constants.DriveConstants.MOTOR_FR_ID, MotorType.kBrushless);
  private final CANSparkMax motorFL = new CANSparkMax(Constants.DriveConstants.MOTOR_FL_ID, MotorType.kBrushless);
  private final CANSparkMax motorBR = new CANSparkMax(Constants.DriveConstants.MOTOR_BR_ID, MotorType.kBrushless);
  private final CANSparkMax motorBL = new CANSparkMax(Constants.DriveConstants.MOTOR_BL_ID, MotorType.kBrushless);

  private final MotorControllerGroup SCG_R = new MotorControllerGroup(motorFR, motorBR); 
  private final MotorControllerGroup SCG_L = new MotorControllerGroup(motorFL, motorBL); 
  //encoders to measure driving speed
  public RelativeEncoder encoderL = motorFL.getEncoder();
  public RelativeEncoder encoderR = motorFL.getEncoder();

  //differential drive to control the motors
  private final DifferentialDrive differentialDrive = new DifferentialDrive(motorFR, motorFL);

  //PH compressor powers the solenoids
  private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

  //solenoids to control gear shifting
  private final Solenoid shiftSolenoidL = new Solenoid(PneumaticsModuleType.REVPH, Constants.DriveConstants.LEFT_SOLENOID_CHANNEL);
  private final Solenoid shiftSolenoidR = new Solenoid(PneumaticsModuleType.REVPH, Constants.DriveConstants.RIGHT_SOLENOID_CHANNEL);

  public AHRS navx = new AHRS(SPI.Port.kMXP);

  private final DifferentialDriveOdometry m_odometry;

  


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

    m_odometry = new DifferentialDriveOdometry(navx.getRotation2d(), 
      encoderL.getPosition() * DriveConstants.ROTATIONS_TO_METERS,
      encoderR.getPosition() * DriveConstants.ROTATIONS_TO_METERS);
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
    differentialDrive.arcadeDrive(sForward * Constants.DriveConstants.DRIVE_SPEED, sTurning * Constants.DriveConstants.TURNING_SPEED);
  }
  
  public CommandBase ShiftUp(){
    return runOnce(() -> upShift());
  }
  public CommandBase ShiftDown(){
    return runOnce(() -> downShift());
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();//gets location on playing feild
  }



  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    double leftWheelSpeedMetersPerSecond = DriveConstants.ROTATIONS_TO_METERS * encoderL.getVelocity() * (1 / 60); //constant changing rotations to meters * rpm of output shaft * converting to m/s
    double rightWheelSpeedMetersPerSecond = DriveConstants.ROTATIONS_TO_METERS * encoderR.getVelocity() * (1 / 60);
    return new DifferentialDriveWheelSpeeds(leftWheelSpeedMetersPerSecond, rightWheelSpeedMetersPerSecond);
  }

    /**
   * Sets the Left and Right Voltage for the Drive Speed Controll Groups
   *
   * @param leftVolts Left half of the Drive Train Volts
   * @param rightVolts Right half of the drive Train Volts.
   * @param differentialDrive.feed.
   * @return TankDriveVolts.
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    SCG_L.setVoltage(leftVolts);
    SCG_R.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  public void encoderReset() {
    encoderL.setPosition(0);
    encoderR.setPosition(0);
    
  }

  public void resetOdometry(Pose2d pose) {
    encoderReset();
    m_odometry.resetPosition(navx.getRotation2d(), encoderL.getPosition(), encoderR.getPosition(), pose);
  }

  @Override
  public void periodic() {
    //Automatic gear shifting
    //the speed of the left motor in RPMs
    double lvelocity = encoderL.getVelocity();
    //the speed of the right motor in RPMs
    double rvelocity = encoderR.getVelocity();
    //check if the robot is turning - if the speeds of the left and right motors are different
    boolean isTurning = Math.abs(lvelocity - rvelocity) > Constants.DriveConstants.TURN_THRESHOLD;

    //check if automatic shifitng is enabling and the robot IS NOT turning
    if(Constants.DriveConstants.AUTO_SHIFT_ENABLED && !isTurning){
      //if either motor exceeds the velocity threshold then shift into high gear
      if(Math.abs(lvelocity) > Constants.DriveConstants.UPSHIFT_THRESHOLD
      || Math.abs(rvelocity) > Constants.DriveConstants.UPSHIFT_THRESHOLD){
        upShift();
      }
      //if both motors' speeds are below the downshift threshold then shift down
      if(Math.abs(lvelocity) < Constants.DriveConstants.DOWNSHIFT_THRESHOLD
      && Math.abs(rvelocity) < Constants.DriveConstants.DOWNSHIFT_THRESHOLD){
        downShift();
      }
    }
  }
}
