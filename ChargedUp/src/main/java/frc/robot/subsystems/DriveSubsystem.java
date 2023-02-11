// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;



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
  private final DifferentialDrive differentialDrive = new DifferentialDrive(motorFL, motorFR);
// error for driving straight
  public double error;

  //solenoids to control gear shifting
  private Solenoid shiftSolenoid = new Solenoid(PneumaticsModuleType.REVPH, DriveConstants.SHIFT_SOLENOID_CHANNEL);
  public Boolean inHighGear = false;
  
  public WPI_Pigeon2 pigeon = new WPI_Pigeon2(0);
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

    motorFR.setSmartCurrentLimit(30, 60);
    motorBR.setSmartCurrentLimit(30, 60);
    motorFL.setSmartCurrentLimit(30, 60);
    motorBL.setSmartCurrentLimit(30, 60);
    //turn on compressor

  

    //create odometry object using motor positions
    //drive position = motor rotations * rotations to meters conversion constant
    m_odometry = new DifferentialDriveOdometry(pigeon.getRotation2d(), 
    encoderL.getPosition() * DriveConstants.ROTATIONS_TO_METERS, 
    encoderR.getPosition() * DriveConstants.ROTATIONS_TO_METERS);
    
  }
  /** shifts the gearbox into high gear */
  public void upShift(){
    //shift into high gear by extending both solenoids
    shiftSolenoid.set(true);
    inHighGear = true;

  }
  /** shifts the gearbox into low gear */
  public void downShift(){
    //shift into low gear by retracting both solenoids
    shiftSolenoid.set(false);
    inHighGear = false;

  }

   /** sets the driving speed of the robot ]
    * @param xSpeed  (Double) - the speed to drive forward
    * @param zRotation  (Double) - the speed to drive forward
   */
  public void setDriveSpeedArcade(double xSpeed, double zRotation){
    //set the driving speed based on a forward speed and turning speed - controlled in ArcadeDrive.jave
    if (Math.abs(xSpeed) < .1) {xSpeed = 0;}//deadzones
    if (Math.abs(zRotation) < .1) {zRotation = 0;}//deadzones
    if (zRotation == 0 ){
      driveStraight(xSpeed);}

      else{
    
        if (inHighGear){differentialDrive.arcadeDrive(xSpeed * DriveConstants.DRIVE_SPEED, zRotation * DriveConstants.TURNING_SPEED_HIGH);}
        else{differentialDrive.arcadeDrive(xSpeed * DriveConstants.DRIVE_SPEED, zRotation * DriveConstants.TURNING_SPEED_LOW);}
  }}

  public double driveTrainP() {
    error = encoderL.getVelocity() - encoderR.getVelocity();
    //integral += error*.02;
    //if error < 0, rvelocity > lvelocity; if error > 0, lvelocity > rvelocity
    return DriveConstants.DRIVE_STRAIGHT_P * error;
  }

  public void driveStraight(double xSpeed) {
    differentialDrive.arcadeDrive(xSpeed, driveTrainP());
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
    m_odometry.resetPosition(pigeon.getRotation2d(), 0,0,pose);
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
  public void AutoBalance(){
    double pitch_error = pigeon.getPitch();
    double balance_kp = .01;
    double position_adjust = 0.0;
    double min_command = .01;
    if (pitch_error > 2.0)
    {
            position_adjust = balance_kp * pitch_error - min_command;
    }
    else if (pitch_error < 2.0)
    {
            position_adjust = balance_kp * pitch_error + min_command;
    }
    differentialDrive.arcadeDrive(position_adjust, 0);
  }
  @Override
  public void periodic() {
    //Automatic gear shifting
    //the speed of the left motor in RPMs
    double lvelocity = encoderL.getVelocity();
    //the speed of the right motor in RPMs
    double rvelocity = encoderR.getVelocity();

    SmartDashboard.putBoolean("HighGear?", inHighGear);
    SmartDashboard.putNumber("LeftWheelSpeeds", lvelocity);
    SmartDashboard.putNumber("RightWheelSpeeds", rvelocity);


    //* Automatic gear shifter - automatically shifts into high gear when the robot is driving fast enough and shifts into low gear when the robot slows down */
    //check if the robot is turning - if the speeds of the left and right motors are different
    //boolean izRotation = Math.abs(lvelocity - rvelocity) < DriveConstants.TURN_THRESHOLD;
    //check if automatic shifitng is enabling and the robot IS NOT turning
/*     if(DriveConstants.AUTO_SHIFT_ENABLED && !izRotation){
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
    } */
  }
}
