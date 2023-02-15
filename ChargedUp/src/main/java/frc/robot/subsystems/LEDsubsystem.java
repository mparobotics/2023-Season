// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDsubsystem extends SubsystemBase {
  /** Creates a new LEDsubsystem. */
  private CANdle leds = new CANdle(LEDConstants.LED_ID);
  public LEDsubsystem() {

  }
  public void setColor(int[] color){
    leds.setLEDs(color[0], color[1], color[2],0,0,LEDConstants.LED_COUNT);
  }
  public CommandBase Cube(){
    return runOnce(() -> setColor(LEDConstants.PURPLE_RGB));
  }
  public CommandBase Cone(){
    return runOnce(() -> setColor(LEDConstants.YELLOW_RGB));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
