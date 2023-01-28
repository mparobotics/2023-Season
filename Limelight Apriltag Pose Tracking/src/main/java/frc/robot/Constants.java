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

    //motor positions for each height - picking up rings, low pillar, medium pillar, high pillar 
    //the numbers for the elevator hights use a 40:1 gear ratio - only used with PID

    public static final double ELEVATORGROUND = 0;//placeholder
    public static final double ELEVATORLOW = 191;//placeholder
    public static final double ELEVATORMID = 407;//placeholder
    public static final double ELEVATORHIGH = 624;//placeholder

    //limits to prevent the motor from going too high or too low
    public static final double MAX_ELEVATOR_SETPOINT = 173;
    public static final double MIN_ELEVATOR_SETPOINT = -50;

    //amount to increment the elevator by when the B or Y buttons are pressed - only used with PID
    public static final double ELEVATOR_INCREMENT_AMOUNT = 10;
    

    //xbox controller ID
    public static final int CONTROLLER_PORT = 0;
    //box ID
    public static final int BOX_ID = 1;
    //drive motor IDs
    public static final int MOTOR_FL_ID = 34;
    public static final int MOTOR_BL_ID = 18;
    public static final int MOTOR_FR_ID = 33;
    public static final int MOTOR_BR_ID = 06;
    //elevator motor ID
    public static final int ELEVATORMOTOR_ID = 53;
    public static final int ELEVATORMOTOR2_ID = 54;
    //driving speed
    public static final double DRIVE_SPEED = 0.7;
    public static final double TURN_SPEED = 0.55;
    //elevator speeds
    public static final double ELEVATOR_UP_SPEED = 0.75;
    public static final double ELEVATOR_OVERRIDE_SPEED = 1;
    public static final double ELEVATOR_DOWN_SPEED = -0.2;

    //If we decide to use the PID controller just set this to true 
    public static final boolean ELEVATOR_IS_PID = false;



    

}
