// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class DoubleSolenoidSubsystem extends SubsystemBase {
  Compressor phCompressorA = new Compressor(1, PneumaticsModuleType.REVPH);
  Compressor phCompressorB = new Compressor(2, PneumaticsModuleType.REVPH);
  DoubleSolenoid doubleSolenoidA = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
  DoubleSolenoid doubleSolenoidB = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);

  /** Creates a new PneumaticsSubsystem. */
  public DoubleSolenoidSubsystem() {
    phCompressorA.enableDigital();
    phCompressorB.enableDigital();
  }

  @Override
  public void periodic() {

double currentA = phCompressorA.getCurrent();
double currentB = phCompressorB.getCurrent();
SmartDashboard.putNumber("compressor current", currentA);
SmartDashboard.putNumber("compressor current", currentB);

    // This method will be called once per scheduler run
  }

public CommandBase retract()
{ 
  return runOnce(
    () -> {
    doubleSolenoidA.set(Value.kForward);
    doubleSolenoidB.set(Value.kForward);
  });

}
public CommandBase chuteIntake()
{ 
  return runOnce(
    () -> {
    doubleSolenoidA.set(Value.kReverse);
    doubleSolenoidB.set(Value.kForward);
  });

}

public CommandBase shoot()
{ 
  return runOnce(
    () -> {
    doubleSolenoidA.set(Value.kForward);
    doubleSolenoidB.set(Value.kReverse);
  });

}
public CommandBase groundIntake()
{ 
  return runOnce(
    () -> {
    doubleSolenoidA.set(Value.kReverse);
    doubleSolenoidB.set(Value.kReverse);
  });

}


}
