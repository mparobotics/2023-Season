// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//Double Solenoid
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.phCompressor; 
import edu.wpi.first.wpilibj.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj;



public class DoubleSolenoidSubsystem extends SubsystemBase {
  /** Creates a new DoubleSolenoidSubsystem. */
  Compressor phCompressorA = new Compressor(1, PneumaticsModuleType.REVPH);
  Compressor phCompressorB = new Compressor(2, PneumaticsModuleType.REVPH); // makes a new compressor class, the electricity port is number 1
  DoubleSolenoid doubleSolenoidA = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1); // makes a new double solenoid class at 1(retracts) and 0(forward)
  DoubleSolenoid doubleSolenoidB = new DoulbeSolenoid (PneumaticsModuleType.REVPH, 2, 3);

  public DoubleSolenoidSubsystem() {
    phCompressorA.enableDigital();
    phCompressorB.enableDigital(); //enables the compressor
    //boolean isEnabled = phCompressor.isEnabled(); //boolean = true or false, "isEnabled" holds either true or false value depending on if the compressor is enabled or not
    //boolean pressureSwitch = phCompressor.getPressureSwitchValue(); //again, "pressureSwitch" holds either true or false depending if there is pressure applied to pressure switch or not 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentA = phCompressorA.getCurrent();
    double currentB = phCompressorB.getCurrent(); //gets current value (pressure) from compressor
    SmartDashboard.putNumber("compressor current", currentA);
    SmartDashboard.putNumber("compressor current", currentB); //displays it on smart dashboard
  }

  public CommandBase retract() //if forwardSolenoid is called, returns a command to set doublesolenoid forward at port 0
  {
    return runOnce(
      () -> {
        doubleSolenoidA.set(Value.kForward); //O
        doubleSolenoidB.set(Value.kForward); //O
      });
  }
  
  public CommandBase chuteintake() //if reverseSolenoid is called, returns a command to set doublesolenoid reverse at port 1
  {
    return runOnce(
      () -> {
        doubleSolenoidA.set(Value.kReverse); //X
        doubleSolenoidB.set(Value.kForward); //O
      });
  }

  public CommandBase shoot()
  {
    return runOnce(
      () -> {
        doubleSolenoidA.set(Value.kForward); //O
        doubleSolenoidB.set(Value.kReverse); //X
      });
  }

  public CommandBase groundintake()
  {
    return runOnce(
      () -> {
        doubleSolenoidA.set(Value.kReverse); //X
        doubleSolenoidB.set(Value.kReverse); //X
      });
  }
}
