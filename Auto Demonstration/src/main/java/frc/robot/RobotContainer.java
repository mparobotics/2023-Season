// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;




import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...


  public static final CommandXboxController xbox = new CommandXboxController(Constants.OperatorConstants.XBOX_CONTROLLER_PORT);
  //the drive subsystem
  public static final DriveSubsystem driveSub = new DriveSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    driveSub.setDefaultCommand(new ArcadeDrive(driveSub, 
    () -> xbox.getLeftY(), 
    () -> xbox.getRightX()));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new JoystickButton(xbox, XboxController.Button.kA.value).onTrue(new ShiftUp(driveSub));
    //new JoystickButton(xbox, XboxController.Button.kB.value).onTrue(new ShiftDown(driveSub));
    xbox.button(Button.kA.value).onTrue(driveSub.ShiftUp());
    xbox.button(Button.kB.value).onTrue(driveSub.ShiftDown());

    driveSub.setDefaultCommand(new ArcadeDrive(driveSub, 
    () -> xbox.getLeftY(), () -> xbox.getRightX()));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
    var leftReference = table.getEntry("left_reference");
    var leftMeasurement = table.getEntry("left_measurement");
    var rightReference = table.getEntry("right_reference");
    var rightMeasurement = table.getEntry("right_measurement");

    var leftController = new PIDController(DriveConstants.Drive_Kp, 0, 0);
    var rightController = new PIDController(DriveConstants.Drive_Kp, 0, 0);

    Trajectory trajectoryOne = new Trajectory();
  
    String trajectoryFileOne = "pathplanner/generatedJSON/Path1.wpilib.json";


    
    try{
      Path trajectoryPathOne = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFileOne);
      trajectoryOne = TrajectoryUtil.fromPathweaverJson(trajectoryPathOne);
    } catch(IOException ex) {
        DriverStation.reportError("Unable to open trajectory:" + trajectoryFileOne, ex.getStackTrace());
    }


    //Call concatTraj in Ramsete to run both trajectories back to back as one
    //var concatTraj = trajectoryTwo.concatenate(trajectoryThree);


    RamseteCommand ramseteCommandOne =
    new RamseteCommand(
        trajectoryOne,
        driveSub::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            DriveConstants.Drive_Ks,
            DriveConstants.Drive_Kv,
            DriveConstants.Drive_Ka),
          DriveConstants.kDriveKinematics,
          driveSub::getWheelSpeeds,
          leftController,
          rightController,
        // RamseteCommand passes volts to the callback
        driveSub::tankDriveVolts,
        driveSub);


    leftMeasurement.setNumber(driveSub.getWheelSpeeds().leftMetersPerSecond);
    leftReference.setNumber(leftController.getSetpoint());
      
    rightMeasurement.setNumber(driveSub.getWheelSpeeds().rightMetersPerSecond);
    rightReference.setNumber(rightController.getSetpoint());
        
    // Reset odometry to the starting pose of the trajectory.
    driveSub.resetOdometry(trajectoryOne.getInitialPose());

    return ramseteCommandOne.andThen(() -> driveSub.tankDriveVolts(0, 0));

  }



}
