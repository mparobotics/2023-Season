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
public final class Constants {
  public final class DriveTrainConstants{
    //!  ALL PLACEHOLDERS - NOT TESTED!
    //motor IDs
    public static final int MOTOR_FL_ID = 0;
    public static final int MOTOR_FR_ID = 0;
    public static final int MOTOR_BL_ID = 0;
    public static final int MOTOR_BR_ID = 0;

    //pneumatics compressor
    public static final int COMPRESSOR_ID = 0;

    //gear shifter solenoids
    public static final int LEFT_SOLENOID_CHANNEL = 0;
    public static final int RIGHT_SOLENOID_CHANNEL = 0;

    //driving speeds
    public static final double TURNING_SPEED = 1;
    public static final double DRIVE_SPEED = 1;

    //if the drive speed exceeds this value, then shift into high gear
    public static final double UPSHIFT_THRESHOLD = 5000;
    //if the drive speed gets below this value, then shift into low gear
    public static final double DOWNSHIFT_THRESHOLD = 1000;
    //difference between motor speeds needs to be below this threshold (RPMs)
    public static final double TURN_THRESHOLD = 50;

    public static final boolean AUTO_SHIFT_ENABLED = true;
  }
  public final class OperatorConstants{
    public static final int XBOX_CONTROLLER_PORT = 0;
  }
  
  
    

  


  
}
