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
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DoubleSolenoidSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoDriveBalance;
import frc.robot.commands.AutoDriveBangBang;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoIntakeInstant;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.Intake;
import frc.robot.commands.NullCommand;
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
  private CommandXboxController xbox = new CommandXboxController(0);
  private CommandXboxController helms = new CommandXboxController(2);
  private CommandJoystick box = new CommandJoystick(1);

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
  
  //these auto selector strings can be anything as long as they are unique
  private final String Pick_and_Score = "1";
  private final String Balance2Cube= "2";
  private final String TwoPiecesNoBalance = "3";
  private final String DoNothing = "4";
  private final String JustShoot = "5";
  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    phCompressor.enableDigital();
    
    
    m_chooser.addOption("Score 2 Pieces", TwoPiecesNoBalance);
    m_chooser.addOption("Score 2 Cubes & Balance", Balance2Cube);
    m_chooser.addOption("Do Nothing", DoNothing);
    m_chooser.setDefaultOption("Just Shoot", JustShoot);
    

    SmartDashboard.putData("Auto Chooser", m_chooser);
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
    //xbox.button(Button.kLeftStick.value).whileTrue(new AutoTurn(m_driveSubsystem, 0));
    //xbox.button(Button.kRightStick.value).whileTrue(new AutoTurn(m_driveSubsystem, -180));
    xbox.button(Button.kA.value).whileTrue(m_driveSubsystem.setBrakeCommand()); // when b is pressed, it calls the forwardSolenoid command that is inside the double solenoid subsystem which makes it go forward.
    xbox.button(Button.kB.value).whileTrue(m_driveSubsystem.setCoastCommand()); // when b is pressed, it calls the forwardSolenoid command that is inside the double solenoid subsystem which makes it go forward.
    xbox.button(Button.kX.value).whileTrue(m_doublesolenoidSubsystem.shoot());
    xbox.button(Button.kY.value).whileTrue(m_doublesolenoidSubsystem.retract());
    
    
    m_driveSubsystem.setDefaultCommand(new ArcadeDrive(m_driveSubsystem, 
    () -> xbox.getLeftY(), () -> xbox.getRightX()));

    xbox.axisGreaterThan(Axis.kRightTrigger.value, 0.5).onTrue(m_driveSubsystem.setBrakeCommand());
    xbox.axisGreaterThan(Axis.kRightTrigger.value, 0.5).onFalse(m_driveSubsystem.setCoastCommand());
    box.button(1).whileTrue(m_doublesolenoidSubsystem.retract()); // when b is pressed, it calls the forwardSolenoid command that is inside the double solenoid subsystem which makes it go forward.
    box.button(2).whileTrue(new Intake(m_intakeSubsystem, IntakeConstants.SHOOTING_SPEED));
    box.button(3).whileTrue(m_doublesolenoidSubsystem.chuteintake());
    box.button(4).whileTrue(new Intake(m_intakeSubsystem, IntakeConstants.OUTTAKE_SPEED));
    box.button(5).whileTrue(m_doublesolenoidSubsystem.groundintake());
    box.button(6).whileTrue(new Intake(m_intakeSubsystem, IntakeConstants.INTAKE_SPEED));

    helms.button(Button.kB.value).onTrue(m_driveSubsystem.gyroReset());
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

  private Command runShootingSlow(double seconds){
    return new AutoIntake(m_intakeSubsystem, -.8).withTimeout(seconds);
  }
  private Command AutoDrive(double setpoint, double speed){
    return new AutoDriveBangBang(m_driveSubsystem, setpoint, speed);
  }

  private Command AutoDrive1(double setpoint, double speed){
    return new AutoDriveBangBang(m_driveSubsystem, setpoint, speed);
  }
  public Command SetCoast(){
    return m_driveSubsystem.setCoastCommand();
  }

  private Command autoDriveBalance(){
    return new AutoDriveBalance(m_driveSubsystem);
  }

  private Command setArmGround(){
    return m_doublesolenoidSubsystem.groundintake();
  }

  private Command encoderReset(){
    return m_driveSubsystem.encoderResetCommand();
  }

  private Command setArmRetracted(){
    return m_doublesolenoidSubsystem.retract();
  }
  private Command setArmShoot(){
    return m_doublesolenoidSubsystem.shoot();
  }

private Command autoIntakeInstant(double speed){
  return new AutoIntakeInstant(m_intakeSubsystem, speed);
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
/*     String Trajectory_pickandscore1 = "pathplanner/generatedJSON/1,2,3 - Pick Up & Score (1).wpilib.json";
    String Trajectory_pickandscore2 = "pathplanner/generatedJSON/1,2,3 - Pick Up & Score (2).wpilib.json";
    String Trajectory_leave = "pathplanner/generatedJSON/1,2,3 - Leave.wpilib.json";
    String Test_Auto = "pathplanner/generatedJSON/Test Auto.json"; */
    
    //display values in the table
    leftMeasurement.setNumber(m_driveSubsystem.getWheelSpeeds().leftMetersPerSecond);
    leftReference.setNumber(leftController.getSetpoint());
    rightMeasurement.setNumber(m_driveSubsystem.getWheelSpeeds().rightMetersPerSecond);
    rightReference.setNumber(rightController.getSetpoint());
    
    
     m_autoSelected = m_chooser.getSelected();
     System.out.println("Auto Selected: " + m_autoSelected);
    
    //if (m_autoSelected != null){
      switch (m_autoSelected)
      {
        //why do we always encoder reset before driving?
        //can we make the encoderReset() a part of AutoDrive1() or will that break something?
          case TwoPiecesNoBalance:
          //set against grid
            return new SequentialCommandGroup(m_driveSubsystem.setBrakeCommand(), m_driveSubsystem.ShiftDown(),runShooting(.5),autoIntakeInstant(IntakeConstants.INTAKE_SPEED),
            setArmGround(), encoderReset(),
            AutoDrive(220, .75), setArmRetracted(), encoderReset(), AutoDrive1(-200, -.75),
            runShooting(.7), encoderReset(), setArmGround(), AutoDrive(180, .8));

          case Balance2Cube:
          //set against charging station
            return new SequentialCommandGroup(m_driveSubsystem.setBrakeCommand(), m_driveSubsystem.ShiftDown(),runShooting(1), encoderReset(),
            setArmGround(), autoIntakeInstant(IntakeConstants.INTAKE_SPEED), AutoDrive(200, .6),  
            autoIntakeInstant(0), setArmRetracted(), encoderReset(), AutoDrive1(-141, -.6),
            autoIntakeInstant(IntakeConstants.SHOOTING_SPEED), (autoDriveBalance()), new NullCommand().withTimeout(1),
            autoIntakeInstant(0));
      
        case DoNothing:
          return null;

        case JustShoot:
          return runShooting(2);
        }
    //} 
     
    //else {return null;}
    return null;
    


  }
  
};