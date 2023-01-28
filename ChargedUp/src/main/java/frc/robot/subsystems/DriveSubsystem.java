// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

import frc.robot.Constants.DriveConstants;


import edu.wpi.first.wpilibj.SPI;

public class DriveSubsystem extends SubsystemBase {
  
  //drive base motors
  private final CANSparkMax motorFR = new CANSparkMax(DriveConstants.MOTOR_FR_ID, MotorType.kBrushless);
  private final CANSparkMax motorFL = new CANSparkMax(DriveConstants.MOTOR_FL_ID, MotorType.kBrushless);
  private final CANSparkMax motorBR = new CANSparkMax(DriveConstants.MOTOR_BR_ID, MotorType.kBrushless);
  private final CANSparkMax motorBL = new CANSparkMax(DriveConstants.MOTOR_BL_ID, MotorType.kBrushless);
  //encoders to measure driving speed
  public RelativeEncoder encoderL = motorFL.getEncoder();
  public RelativeEncoder encoderR = motorFR.getEncoder();

  //create motor controller groups
  private final MotorControllerGroup SCG_R = new MotorControllerGroup(motorFR, motorBR);
  private final MotorControllerGroup SCG_L = new MotorControllerGroup(motorFL, motorBL);

  //differential drive to control the motors
  private final DifferentialDrive differentialDrive = new DifferentialDrive(motorFR, motorFL);

  //PH compressor powers the solenoids
  private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

  //solenoids to control gear shifting
  private Solenoid shiftSolenoidL = new Solenoid(PneumaticsModuleType.REVPH, DriveConstants.LEFT_SOLENOID_CHANNEL);
  private Solenoid shiftSolenoidR = new Solenoid(PneumaticsModuleType.REVPH, DriveConstants.RIGHT_SOLENOID_CHANNEL);
  
  public PigeonIMU pige = new AHRS(SPI.Port.kMXP);
  //odometry object 
  private final DifferentialDriveOdometry m_odometry;
  /** Creates a new DriveSubsystem. */
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

    //create odometry object using motor positions
    //drive position = motor rotations * rotations to meters conversion constant
    m_odometry = new DifferentialDriveOdometry(navx.getRotation2d(), 
    encoderL.getPosition() * DriveConstants.ROTATIONS_TO_METERS, 
    encoderR.getPosition() * DriveConstants.ROTATIONS_TO_METERS);
    
  }
  /** shifts the gearbox into high gear */
  public void upShift(){
    //shift into high gear by extending both solenoids
    shiftSolenoidL.set(true);
    shiftSolenoidR.set(true);
  }
  /** shifts the gearbox into low gear */
  public void downShift(){
    //shift into low gear by retracting both solenoids
    shiftSolenoidL.set(false);
    shiftSolenoidR.set(false);
  }

   /** sets the driving speed of the robot ]
    * @param sForward  (Double) - the speed to drive forward
    * @param sTurning  (Double) - the speed to drive forward
   */
  public void setDriveSpeedArcade(double sForward, double sTurning){
    //set the driving speed based on a forward speed and turning speed - controlled in ArcadeDrive.jave
    differentialDrive.arcadeDrive(sForward * DriveConstants.DRIVE_SPEED, sTurning * DriveConstants.TURNING_SPEED);
  }
  public void encoderReset() {
    encoderL.setPosition(0);
    encoderR.setPosition(0);
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public void resetOdometry(Pose2d pose) {
    encoderReset();
    m_odometry.resetPosition(navx.getRotation2d(), 0,0,pose);
    //we know that the encoders' positions are both zero since we just set them in encoderReset();
  }
  public void tankDriveVolts(double leftVolts, double rightVolts){
    SCG_L.setVoltage(leftVolts);
    SCG_R.setVoltage(rightVolts);
    differentialDrive.feed();
  }
  public CommandBase ShiftUp(){
    return runOnce(() -> upShift());
  }
  public CommandBase ShiftDown(){
    return runOnce(() -> downShift());
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    //Driving speed in meters/second = 1/60 minutes per second * roatations-meters conversion constant * motor RPMs (encoder velocity)
    Double leftWheelMetersPerSecond = (1/60) * DriveConstants.ROTATIONS_TO_METERS * encoderL.getVelocity();
    Double rightWheelMetersPerSecond = (1/60) * DriveConstants.ROTATIONS_TO_METERS * encoderL.getVelocity();
    return new DifferentialDriveWheelSpeeds(leftWheelMetersPerSecond, rightWheelMetersPerSecond);
  }
  @Override
  public void periodic() {
    //Automatic gear shifting
    //the speed of the left motor in RPMs
    double lvelocity = encoderL.getVelocity();
    //the speed of the right motor in RPMs
    double rvelocity = encoderR.getVelocity();

    //* Automatic gear shifter - automatically shifts into high gear when the robot is driving fast enough and shifts into low gear when the robot slows down */
    //check if the robot is turning - if the speeds of the left and right motors are different
    boolean isTurning = Math.abs(lvelocity - rvelocity) < DriveConstants.TURN_THRESHOLD;
    //check if automatic shifitng is enabling and the robot IS NOT turning
    if(DriveConstants.AUTO_SHIFT_ENABLED && !isTurning){
      //if either motor exceeds the velocity threshold then shift into high gear
      if(Math.abs(lvelocity) > DriveConstants.UPSHIFT_THRESHOLD
      || Math.abs(rvelocity) > DriveConstants.UPSHIFT_THRESHOLD){
        upShift();
      }
      //if both motors' speeds are below the downshift threshold then shift down
      if(Math.abs(lvelocity) < DriveConstants.DOWNSHIFT_THRESHOLD
      && Math.abs(rvelocity) < DriveConstants.DOWNSHIFT_THRESHOLD){
        downShift();
      }
    }
  }
}
