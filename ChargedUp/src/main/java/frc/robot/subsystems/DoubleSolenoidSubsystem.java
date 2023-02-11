// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj.DoubleSolenoid;



public class DoubleSolenoidSubsystem extends SubsystemBase {                                                  
  /** Creates a new DoubleSolenoidSubsystem. */
  DoubleSolenoid ShortPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.SHORT_FORWARD,IntakeConstants.SHORT_REVERSE); // makes a new double solenoid class at 1(retracts) and 0(forward)
  DoubleSolenoid LongPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.LONG_FORWARD, IntakeConstants.LONG_REVERSE);


  public DoubleSolenoidSubsystem() {
    //enables the compressor
    //boolean isEnabled = phCompressor.isEnabled(); //boolean = true or false, "isEnabled" holds either true or false value depending on if the compressor is enabled or not
    //boolean pressureSwitch = phCompressor.getPressureSwitchValue(); //again, "pressureSwitch" holds either true or false depending if there is pressure applied to pressure switch or not 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public CommandBase retract() //if forwardSolenoid is called, returns a command to set doublesolenoid forward at port 0
  {
    System.out.println("Retracting!");
    return runOnce(
      () -> {
        ShortPiston.set(Value.kForward); //O
        LongPiston.set(Value.kForward); //O
      });
  }
  
  public CommandBase chuteintake() //if reverseSolenoid is called, returns a command to set doublesolenoid reverse at port 1
  {
    System.out.println("Chute Intake!");
    return runOnce(
      () -> {
        ShortPiston.set(Value.kReverse); //X
        LongPiston.set(Value.kForward); //O
      });
  }

  public CommandBase shoot()
  {
    System.out.println("Shooting!");
    return runOnce(
      () -> {
        LongPiston.set(Value.kForward); //O
        ShortPiston.set(Value.kReverse); //X
      });
  }

  public CommandBase groundintake()
  {
    System.out.println("GroundIntake!");
    return runOnce(
      () -> {
        LongPiston.set(Value.kReverse); //X
        ShortPiston.set(Value.kReverse); //X
      });
  }
}
