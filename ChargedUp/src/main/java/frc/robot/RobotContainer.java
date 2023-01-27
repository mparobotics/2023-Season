// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;






import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DoubleSolenoidSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
  //the pneumatics to control the arm
  private final DoubleSolenoidSubsystem m_doublesolenoidSubsystem = new DoubleSolenoidSubsystem(); //replicating a double solenoid subsystem
  // Replace with CommandPS4Controller or CommandJoystick if needed

  //xbox controller
  private CommandXboxController xbox = new CommandXboxController(Constants.OperatorConstants.XBOX_CONTROLLER_PORT);
  private CommandJoystick box = new CommandJoystick(OperatorConstants.BOX_ID);

  //the drive subsystem
  private DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  //moving the drive kinematics from Constants to DriveSubsystem fixed the static issue
  private  DifferentialDriveKinematics DRIVE_KINEMATICS =
            new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH_METERS);
  
  //individual pid controllers for the left and right sides of the robot
  PIDController leftController; 
  PIDController rightController; 
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the trigger bindings
    configureBindings();
    m_driveSubsystem.setDefaultCommand(new ArcadeDrive(m_driveSubsystem, 
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
    //new JoystickButton(xbox, XboxController.Button.kA.value).onTrue(new ShiftUp(m_driveSubsystem));
    //new JoystickButton(xbox, XboxController.Button.kB.value).onTrue(new ShiftDown(m_driveSubsystem));
    //A button shifts the gearbox into high gear
    xbox.button(Button.kA.value).onTrue(m_driveSubsystem.ShiftUp());
    //B button shifts the gearbox into low gear
    xbox.button(Button.kB.value).onTrue(m_driveSubsystem.ShiftDown());

    m_driveSubsystem.setDefaultCommand(new ArcadeDrive(m_driveSubsystem, 
    () -> xbox.getLeftY(), () -> xbox.getRightX()));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    box.button(1).whileTrue(m_doublesolenoidSubsystem.retract()); // when b is pressed, it calls the forwardSolenoid command that is inside the double solenoid subsystem which makes it go forward.
    box.button(4).whileTrue(m_doublesolenoidSubsystem.chuteintake()); 
    box.button(2).whileTrue(m_doublesolenoidSubsystem.shoot()); // when b is pressed, it calls the forwardSolenoid command that is inside the double solenoid subsystem which makes it go forward.
    box.button(3).whileTrue(m_doublesolenoidSubsystem.groundintake());// when a is pressed, it calls the reverseSolenoid command that is inside the double solenoid subsystem which makes it go backward.

  }

  /**
   takes a location of the JSON file as an input
   generates a ramsete command from the file
   */
  private RamseteCommand makeRamseteCommand(String filePath){
    //trajectory object
    Trajectory pTrajectory = new Trajectory();
    //try to open the file
    try{
      Path TrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filePath);
      pTrajectory = TrajectoryUtil.fromPathweaverJson(TrajectoryPath);
    }
    catch(IOException ex){
      DriverStation.reportError("Unable to open trajectory:" + filePath, ex.getStackTrace());
    }
    
    //creates the ramsete command
    RamseteCommand rCommand = new RamseteCommand(
        pTrajectory,
        m_driveSubsystem::getPose,
        new RamseteController(DriveConstants.RAMSETE_B, DriveConstants.RAMSETE_ZETA),
        new SimpleMotorFeedforward(DriveConstants.DRIVE_KS, DriveConstants.DRIVE_KV, DriveConstants.DRIVE_KA),
        DRIVE_KINEMATICS,
        m_driveSubsystem::getWheelSpeeds,
        leftController,
        rightController,
        // RamseteCommand passes volts to the callback
        m_driveSubsystem::tankDriveVolts,
        m_driveSubsystem
      );
    
    // Reset odometry to the starting pose of the trajectory.
    m_driveSubsystem.resetOdometry(pTrajectory.getInitialPose());
   
    return rCommand;

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //troubleshooting info
    var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
    var leftReference = table.getEntry("left reference");
    var leftMeasurement = table.getEntry("left measurement");
    var rightReference = table.getEntry("right reference");
    var rightMeasurement = table.getEntry("right measurement");
    
    //Set PID controllers
    leftController = new PIDController(DriveConstants.DRIVE_P_GAIN, 0, 0);
    rightController = new PIDController(DriveConstants.DRIVE_P_GAIN, 0, 0);
    
    //the loaction of a JSON file of the test path
    String Trajectory_Leave = "pathplanner/generatedJSON/1,2,3 - Leave.wpilib.json";
    //String Trajectory_Balance = "pathplanner/generatedJSON/2 - Balance.wpilib.json";
    
    //display values in the table
    leftMeasurement.setNumber(m_driveSubsystem.getWheelSpeeds().leftMetersPerSecond);
    leftReference.setNumber(leftController.getSetpoint());
    rightMeasurement.setNumber(m_driveSubsystem.getWheelSpeeds().rightMetersPerSecond);
    rightReference.setNumber(rightController.getSetpoint());
    
    //execute a command with the first trajectory, then stop the robot
    return makeRamseteCommand(Trajectory_Leave) //Starts Trajectory_Leave
          .andThen(m_doublesolenoidSubsystem.retract()) //Retracts Arm
          .andThen()
          .andThen(() -> m_driveSubsystem.tankDriveVolts(0,0)); //Stops Robot
  }
};
