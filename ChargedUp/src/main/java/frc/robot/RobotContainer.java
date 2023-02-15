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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DoubleSolenoidSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutoSelectorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.Intake;
import frc.robot.subsystems.IntakeSubsystem;

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
  
  private final Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  
  // The robot's subsystems and commands are defined here...
  //Creating instance of IntakeSubsystem called m_intakeSubsystem
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();


  // Replace with CommandPS4Controller or CommandJoystick if needed

  //xbox controller
  private CommandXboxController xbox = new CommandXboxController(Constants.OperatorConstants.XBOX_CONTROLLER_ID);
  private CommandJoystick box = new CommandJoystick(OperatorConstants.BOX_ID);

  //the drive subsystem
  private DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  //moving the drive kinematics from Constants to DriveSubsystem fixed the static issue
  private  DifferentialDriveKinematics DRIVE_KINEMATICS =
       new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH_METERS);
  
  //individual pid controllers for the left and right sides of the robot
  PIDController leftController; 
  PIDController rightController; 

  
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  
  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    phCompressor.enableDigital();

    m_chooser.setDefaultOption("Simple Test Trajectory", AutoSelectorConstants.Test_Auto_1);
    m_chooser.addOption("Curve Test Trajectory", AutoSelectorConstants.Test_Auto_1);
    m_chooser.addOption("Pick & Score", AutoSelectorConstants.Pick_and_Score);
    m_chooser.addOption("Score Low &Leave", AutoSelectorConstants.Score_Low_and_Leave);
    m_chooser.addOption("Balance1" , AutoSelectorConstants.Balance);
    m_chooser.addOption("Leave", AutoSelectorConstants.Leave);
    SmartDashboard.putData("Auto choices", m_chooser);
    // Configure the trigger bindings
    configureBindings();


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
    
    //new JoystickButton(xbox, XboxController.Button.kA.value).onTrue(new ShiftUp(m_driveSubsystem));
    //new JoystickButton(xbox, XboxController.Button.kB.value).onTrue(new ShiftDown(m_driveSubsystem));
    //A button shifts the gearbox into high gear
    xbox.button(Button.kRightBumper.value).onTrue(m_driveSubsystem.ShiftUp());
    //B button shifts the gearbox into low gear
    xbox.button(Button.kLeftBumper.value).onTrue(m_driveSubsystem.ShiftDown());
    
    
    m_driveSubsystem.setDefaultCommand(new ArcadeDrive(m_driveSubsystem, 
    () -> xbox.getLeftY(), () -> xbox.getRightX()));
    
    box.button(1).whileTrue(m_doublesolenoidSubsystem.retract()); // when b is pressed, it calls the forwardSolenoid command that is inside the double solenoid subsystem which makes it go forward.
    box.button(2).whileTrue(m_doublesolenoidSubsystem.shoot()); // when b is pressed, it calls the forwardSolenoid command that is inside the double solenoid subsystem which makes it go forward.
    box.button(3).whileTrue(m_doublesolenoidSubsystem.groundintake());
    box.button(4).whileTrue(m_doublesolenoidSubsystem.chuteintake()); 
    box.button(5).whileTrue(new Intake(m_intakeSubsystem, IntakeConstants.INTAKE_SPEED));
    box.button(6).whileTrue(new Intake(m_intakeSubsystem, IntakeConstants.SHOOTING_SPEED));
  }

  /**
   takes a location of the JSON file as an input
   and generates a ramsete command from the file
   */
  private RamseteCommand followTrajectory(String filePath){
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
  
  
 
  private Command runIntaking(double seconds){
    return new AutoIntake(m_intakeSubsystem, IntakeConstants.INTAKE_SPEED).withTimeout(seconds);
  }
  private Command runOuttaking(double seconds){
    return new AutoIntake(m_intakeSubsystem, IntakeConstants.OUTTAKE_SPEED).withTimeout(seconds);
  }
  private Command runShooting(double seconds){
    return new AutoIntake(m_intakeSubsystem, IntakeConstants.SHOOTING_SPEED).withTimeout(seconds);
  }
  private Command setArmGround(){
    return m_doublesolenoidSubsystem.groundintake();
  }
  private Command setArmRetracted(){
    return m_doublesolenoidSubsystem.retract();
  }
  private Command setArmShoot(){
    return m_doublesolenoidSubsystem.shoot();
  }


  private Command stopRobot(){
    return m_driveSubsystem.setVolts(0, 0);
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
    
    //the loaction of a JSON trajectory file
    String Trajectory_pickandscore1 = "pathplanner/generatedJSON/1,2,3 - Pick Up & Score (1).wpilib.json";
    String Trajectory_pickandscore2 = "pathplanner/generatedJSON/1,2,3 - Pick Up & Score (2).wpilib.json";
    String Trajectory_leave = "pathplanner/generatedJSON/1,2,3 - Leave.wpilib.json";
    String Test_Auto = "pathplanner/generatedJSON/Test Auto.json";
    
    //display values in the table
    leftMeasurement.setNumber(m_driveSubsystem.getWheelSpeeds().leftMetersPerSecond);
    leftReference.setNumber(leftController.getSetpoint());
    rightMeasurement.setNumber(m_driveSubsystem.getWheelSpeeds().rightMetersPerSecond);
    rightReference.setNumber(rightController.getSetpoint());
    
    
     m_autoSelected = m_chooser.getSelected();
     System.out.println("Auto Selected: " + m_autoSelected);
    switch (m_autoSelected)
    {
      case AutoSelectorConstants.Pick_and_Score:
        return new SequentialCommandGroup(
          setArmGround(), //Arm moves to groundintake position
          runOuttaking(2), //Starts outtaking for 2 seconds (for pre-loaded cargo)
          setArmRetracted(), //Arm moves to retract position
          followTrajectory(Trajectory_pickandscore1), //Runs "Trajectory_pickandscore1" file
          stopRobot(), //stops robot
          Commands.parallel(runIntaking(3), //Starts intaking for 3 seconds
            followTrajectory(Trajectory_pickandscore2)),//Runs "Trajectory_pickandscore2" as soon as robot starts intaking 
          setArmGround(), //Arm moves to groundintake position
          runOuttaking(2), //Starts outtaking for 2 seconds
          stopRobot()); //stop robot
      
      case AutoSelectorConstants.Leave:
        return new SequentialCommandGroup(
          followTrajectory(Trajectory_leave), 
          stopRobot());
      
      case AutoSelectorConstants.Balance:
        return new SequentialCommandGroup(
          setArmGround(), 
          runOuttaking(2), 
          setArmRetracted(),
          followTrajectory(Trajectory_leave));
      
      case AutoSelectorConstants.Test_Auto_1:
        return new SequentialCommandGroup(
          followTrajectory(Test_Auto)
        );
          
    } 

      return null;
    
    


  }
  
};