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
  public static class OperatorConstants {
    
  }
  public final class DriveConstants
  {
    public static final int MOTOR_FL_ID = 11;
    public static final int MOTOR_FR_ID = 13;
    public static final int MOTOR_BL_ID = 10;
    public static final int MOTOR_BR_ID = 12;
  
    public static final int PIGEON_ID = 17;
    

    //pneumatics compressor
    public static final int COMPRESSOR_ID = 0;

    public static final int SOLENOID_CHANNEL = 0;
  
    //gear shifter solenoids
    public static final int SHIFT_SOLENOID_CHANNEL = 9;
    public static final double MAX_DRIVE_SPEED = 4000;
   

    //driving speeds
    public static final double TURNING_SPEED_LOW = -.7;
    public static final double DRIVE_SPEED = -1;

    //if the motor speed (RPMs) exceeds this value, then shift into high gear
    public static final double UPSHIFT_THRESHOLD = 5000;
    //if the motor speed (RPMs) gets below this value, then shift into low gear
    public static final double DOWNSHIFT_THRESHOLD = 1000;
    //difference between motor speeds needs to be below this threshold to be considered "not turning" (RPMs)
    public static final double TURN_THRESHOLD = 50;
    //if the robot should shift gears automatically
    public static final boolean AUTO_SHIFT_ENABLED = false;
    
    //how many encoder rotations = 1 meter of robot travel
    public static final double ROTATIONS_TO_METERS = 8 * Math.PI * 0.0254; //8in wheel diameter * PI * 0.0254 meters/inch

    //Autonomous trajectory following constants
    //! NOT TESTED or LAST YEAR'S values
    public static final double DRIVE_P_GAIN = 0;
    
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;

    public static final double DRIVE_KS = 0.6057;
    public static final double DRIVE_KV = 2.4263;
    public static final double DRIVE_KA = 0.37369;
    //the distance between the left and right sides of the robot (meters)
    public static final double TRACK_WIDTH_METERS = 0.73253;
    //Porportional gain value for driving straight
    public static final double DRIVE_STRAIGHT_P = 0.0007;
  
    } 
}