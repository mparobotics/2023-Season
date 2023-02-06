// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.plaf.TreeUI;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {

//Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);



  /** Creates a new PneumaticsSubsystem. */
  public PneumaticsSubsystem() {
    phCompressor.enableDigital();
 

    SmartDashboard.putBoolean("solenoidTrigger", false);
    boolean isEnabled = phCompressor.isEnabled();
    boolean pressureSwitch = phCompressor.getPressureSwitchValue();
    

  }

  @Override
  public void periodic() {
    double current = phCompressor.getCurrent();
    
    SmartDashboard.putNumber("compressor current", current);
    // This method will be called once per scheduler run
  }

  public CommandBase forwardSolenoid(){
    return runOnce(
      () -> {
        /* one-time action goes here */
        doubleSolenoid.set(Value.kForward);
        SmartDashboard.putBoolean("solenoidTrigger", true);
      });
    
  }

  public CommandBase reverseSolenoid(){
    return runOnce(
      () -> {
        /* one-time action goes here */
        doubleSolenoid.set(Value.kReverse);
        SmartDashboard.putBoolean("solenoidTrigger", true);
      });
    
  }
}
