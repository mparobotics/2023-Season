// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;


import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  //if the maximum setpoint check should be overridden
  public boolean overrideMax = false;
  //if the minimum setpoint check should be overridden
  public boolean overrideMin = false;

  //the motor to power the elevator
  private final CANSparkMax elevatorMotor = new CANSparkMax(Constants.ELEVATORMOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax elevatorMotor2 = new CANSparkMax(Constants.ELEVATORMOTOR2_ID, MotorType.kBrushless);
  //an encoder for the elevator motor
  




  //PID Controller
  
  //PID values
  public double kP = 5e-5;
  public double kI = 0;
  public double kD = 0;
  public double kIz = 0;
  public double kFF = .000156;
  public double kMaxOutput = 1;
  public double kMinOutput = -1;
  public double maxRPM = 5700;
  public double maxVel = 2000;
  public double minVel = 0;
  public double maxAcc = 1500;
  public double allowedErr = 10;


  public double m_setPoint;




  public ElevatorSubsystem() {
    elevatorMotor.restoreFactoryDefaults();
 
    elevatorMotor.setIdleMode(IdleMode.kBrake);

   


  }

  //set the elevator to a specific speed (no PID)
  public void setElevatorSpeed(double Speed){
      elevatorMotor.set(Speed);
      elevatorMotor2.set(-Speed);
    }
  

  //reset encoder to 0 rotations
  public void encoderReset(){
  
  }

  @Override
  public void periodic() {
    //display encoder position value
    

    }
  }


