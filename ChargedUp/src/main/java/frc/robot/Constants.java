// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
  public final class IntakeConstants 
  {
  //intake motor ID
  public static final int INTAKE_MOTOR_R_ID = 54;
  public static final int INTAKE_MOTOR_L_ID = 53;
  //intake speed
  public static final double INTAKE_SPEED = .5;
  public static final double OUTTAKE_SPEED = -1;
  public static final double SHOOTING_SPEED = -6;

  }

  public final class DriveConstants
  {
    public static final int MOTOR_FL_ID = 0;
    public static final int MOTOR_FR_ID = 0;
    public static final int MOTOR_BL_ID = 0;
    public static final int MOTOR_BR_ID = 0;
  
  


    //pneumatics compressor
    public static final int COMPRESSOR_ID = 0;

    public static final int SOLENOID_CHANNEL = 0;
  
    //gear shifter solenoids
    public static final int LEFT_SOLENOID_CHANNEL = 0;
    public static final int RIGHT_SOLENOID_CHANNEL = 0;

    //driving speeds
    public static final double TURNING_SPEED = 1;
    public static final double DRIVE_SPEED = 1;

    //if the motor speed (RPMs) exceeds this value, then shift into high gear
    public static final double UPSHIFT_THRESHOLD = 5000;
    //if the motor speed (RPMs) gets below this value, then shift into low gear
    public static final double DOWNSHIFT_THRESHOLD = 1000;
    //difference between motor speeds needs to be below this threshold (RPMs)
    public static final double TURN_THRESHOLD = 50;
    //if the robot should shift gears automatically
    public static final boolean AUTO_SHIFT_ENABLED = true;
    
    //how many encoder rotations = 1 meter of robot travel
    public static final double ROTATIONS_TO_METERS = 8 * Math.PI * 0.0254; //8in wheel diameter * PI * 0.0254 inches/meter

    //Autonomous trajectory following constants
    public static final double DRIVE_P_GAIN = 0;
    
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;

    public static final double DRIVE_KS = 0.6057;
    public static final double DRIVE_KV = 2.4263;
    public static final double DRIVE_KA = 0.37369;

    public static final double TRACK_WIDTH_METERS = 0.73253;
    
    public static final double Drive_Kp = 1.4948;
    public static final double Drive_KpTest = 0;
    public static final double Drive_Kd = 0.0;
    } 
    public final class OperatorConstants
    {
    public static final int XBOX_CONTROLLER_PORT = 0;
    public static final int BOX_ID = 1;
    //public static final int kDrivercontrollerPort = 0; 
    }

    public final class AutoSelectorConstants
    {
      public static final String Pick_and_Score = "Pick and Score";
      public static final String Leave = "Leave";
      public static final String Balance = "Balance";
      public static final String Example = "Example";

    }
  
  
    

  


  
}
