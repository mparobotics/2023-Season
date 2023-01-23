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

public class DriveSubsystem extends SubsystemBase { // creates a drive subsystem class
  /** Creates a new DriveSubsystem. */
  //drive base motors
  private final CANSparkMax motorFR = new CANSparkMax(Constants.DriveTrainConstants.MOTOR_FR_ID, MotorType.kBrushless); //  uses predefined variable CANSparkMax called motor FR and defines it using a predefined constructor. in the constructor it uses constants to give its pin number and motor type.
  private final CANSparkMax motorFL = new CANSparkMax(Constants.DriveTrainConstants.MOTOR_FL_ID, MotorType.kBrushless); //  uses predefined variable CANSparkMax called motor FL and defines it using a predefined constructor. in the constructor it uses constants to give its pin number and motor type.
  private final CANSparkMax motorBR = new CANSparkMax(Constants.DriveTrainConstants.MOTOR_BR_ID, MotorType.kBrushless); //  uses predefined variable CANSparkMax called motor BR and defines it using a predefined constructor. in the constructor it uses constants to give its pin number and motor type.
  private final CANSparkMax motorBL = new CANSparkMax(Constants.DriveTrainConstants.MOTOR_BL_ID, MotorType.kBrushless); //  uses predefined variable CANSparkMax called motor BL and defines it using a predefined constructor. in the constructor it uses constants to give its pin number and motor type.
  //encoders to measure driving speed
  private final RelativeEncoder encoderL = motorFL.getEncoder();  // uses a predetermined varaibsble  to make a variable for the encoders. it stores the frontleft motor's encoder
  private final RelativeEncoder encoderR = motorFR.getEncoder(); // same as above. I think because the motor is defined as cansparkmax, that includes the .getEncoder function, so that is why we can synthesize it into a variable.


  //differential drive to control the motors
  private final DifferentialDrive differentialDrive = new DifferentialDrive(motorFR, motorFL); //uses predefined variable and assigning it to a constructor. the constructor makes the two front motors into one unit: differential Drive. easier to control and less coding.

  //PH compressor powers the solenoids
  private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH); // constructor that uses preedefined variavble and embedded class + names it and assigns it the type.
  
  //solenoids to control gear shifting
  private final Solenoid shiftSolenoidL = new Solenoid(PneumaticsModuleType.REVPH, Constants.DriveTrainConstants.LEFT_SOLENOID_CHANNEL); // makes a new variable using a predefined variable and calls a constructor from an embedded class and defines it with ints pin number and solenoid type.
  private final Solenoid shiftSolenoidR = new Solenoid(PneumaticsModuleType.REVPH, Constants.DriveTrainConstants.RIGHT_SOLENOID_CHANNEL); // same things as above but for the right solenoid, not the left.
  


  public DriveSubsystem() { // deez.... deez methods.
    //back motors follow front motors. this man has got it.
    motorBR.follow(motorFR);
    motorBL.follow(motorFL);
    //invert left motors. Why? counter cockwise vs cockwise.
    motorFL.setInverted(true);
    motorBL.setInverted(true);
    //dont invert right motors. facts
    motorFR.setInverted(false);
    motorBR.setInverted(false);
    //turn on compressor. idk why enable digital is what its called but ok
    compressor.enableDigital(); 
    shiftSolenoidL.set(false); //set solenoid to OFF. probably whenever drivesubsystem is called this runs and makes sure nobody gets impaled by a pneumatics machine at 60 mph.
    shiftSolenoidR.set(false);
  }
  public void upShift(){ // method that makes it push gear
    shiftSolenoidL.set(true);
    shiftSolenoidR.set(true);
  }
  public void downShift(){ //other mthod that makes it take back the gear by reversing solenoid
    shiftSolenoidL.set(false);
    shiftSolenoidR.set(false);
  }
  public void setDriveSpeedArcade(double sForward, double sTurning){   // method that defines setDriveSpeedArcade as habing sforward and sturning va.ues. the values ae taken from ArcadeDrive which take it from robot container which take it from the input.
    differentialDrive.arcadeDrive(sForward * Constants.DriveTrainConstants.DRIVE_SPEED/*It is set to 1, but subject to change */, sTurning * Constants.DriveTrainConstants.TURNING_SPEED);
  } // arcade drive translates buttons to speed
  
  public CommandBase ShiftUp(){  //... what is this. I know this is only for stuf that runs once, but like what?
    return runOnce(() -> upShift()); //method that returns command. it makes a command
  }
  public CommandBase ShiftDown(){ //... I know what this does. It moves the thing down and you call this command. but idk syntaz. and also where is ShiftDown defined? 2 questions from these two lines.
    return runOnce(() -> downShift());
  }
  @Override
  public void periodic() { //
    //Automatic gear shifting
    //the speed of the left motor in RPMs
    double lvelocity = encoderL.getVelocity(); // defines the left motor velocity variable. It retrieves the velocity by using variable encoder L(previously defined by us) to get the velocity
    //the speed of the right motor in RPMs
    double rvelocity = encoderR.getVelocity(); // same as above but for the right.
    //check if the robot is turning - if the speeds of the left and right motors are different
    boolean isTurning = Math.abs(lvelocity - rvelocity) > Constants.DriveTrainConstants.TURN_THRESHOLD; // finds out how much it turns, and it has to be less than a constant threshold.

    //check if automatic shifitng is enabling and the robot IS NOT turning
    if(Constants.DriveTrainConstants.AUTO_SHIFT_ENABLED && !isTurning){
      //if either motor exceeds the velocity threshold then shift into high gear
      if(Math.abs(lvelocity) > Constants.DriveTrainConstants.UPSHIFT_THRESHOLD // the upshift threshold is in constants
      || Math.abs(rvelocity) > Constants.DriveTrainConstants.UPSHIFT_THRESHOLD){
        upShift(); // this makes sense combined with the above comment. Just like a bike.
      }
      //if both motors' speeds are below the downshift threshold then shift down
      if(Math.abs(lvelocity) < Constants.DriveTrainConstants.DOWNSHIFT_THRESHOLD // downshift threshold is in constants.
      && Math.abs(rvelocity) < Constants.DriveTrainConstants.DOWNSHIFT_THRESHOLD){
        downShift();
      }
    }
  }
}
