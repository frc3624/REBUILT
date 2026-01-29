// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.IntakeSubsystem;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.IntakeMoveToDown;
import frc.robot.commands.IntakeMoveToUp;
import frc.robot.commands.SysidCommand;
import frc.robot.Constants.ArmConstants;
import java.io.File;
import java.util.HashMap;
// import swervelib.SwerveInputStream;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import frc.robot.commands.Intake;
import frc.robot.commands.IntakeMoveUp;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController driverXbox2 = new CommandXboxController(1);
  
  IntakeSubsystem intake = new IntakeSubsystem();

  Intake moveUp = new Intake(intake, 0.1);
  Intake moveDown = new Intake(intake, -0.1);

  // The robot's subsystems and commands are defined here...
  
  //Subsystems
  // private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/neo"));
 
  //Commands
 
  

 
  
  //Climb
  


  /**R
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  // SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
  //                                                               () -> driverXbox.getLeftY() * -1,
  //                                                               () -> driverXbox.getLeftX() * -1)
  //                                                           .withControllerRotationAxis(() -> driverXbox.getRightX())
  //                                                           .deadband(OperatorConstants.DEADBAND)
  //                                                           .scaleTranslation(0.8)
  //                                                           .allianceRelativeControl(true);
  // SwerveInputStream driveAngularAprilTag = SwerveInputStream.of(drivebase.getSwerveDrive(),
  //                                                               () -> driverXbox.getLeftY() * -1,
  //                                                               () -> driverXbox.getLeftX() * -1)
  //                                                           .withControllerRotationAxis(() -> driverXbox.getLeftY())
  //                                                           .deadband(OperatorConstants.DEADBAND)

  //                                                           .scaleTranslation(0.8)
  //                                                           .allianceRelativeControl(true);

  // SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
  //                                                                  () -> -driverXbox.getLeftY(),
  //                                                                  () -> -driverXbox.getLeftX())
  //                                                              .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
  //                                                              .deadband(OperatorConstants.DEADBAND)
  //                                                              .scaleTranslation(0.8)
  //                                                              .allianceRelativeControl(true);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

   
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    //Driver

    //Arm


    //Drive
    // driverXbox.leftBumper().toggleOnTrue((Commands.runOnce(drivebase::zeroGyro)));

    //Operator

    // driverXbox.povRight().onTrue(Commands.parallel(Commands.runOnce(() -> driveAngularVelocity.scaleTranslation(1))));
    // driverXbox.povUp().onTrue(Commands.parallel(Commands.runOnce(() -> driveAngularVelocity.scaleTranslation(0.3))));
   

    // driverXbox.a()
    // .whileTrue(drivebase.driveWithAutoRotation(
    //     () -> driverXbox.getLeftY() * -1,  // Forward/backward (matches your current setup)
    //     () -> driverXbox.getLeftX() * -1   // Left/right strafe (matches your current setup)
    // ));

    // Command driveFieldOrientedAnglularVelocity    = drivebase.driveFieldOriented(driveAngularVelocity);
    // Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);

    // if (RobotBase.isSimulation())
    // {
    //   drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocitySim);
    // } else
    // {
    //   drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // }

    // if (Robot.isSimulation())
    // {
    //   driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    //   //driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    // }
    // if (DriverStation.isTest())
    // {
    //   drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above! 
    // }
    
      // Normal controls
      
      
      

      driverXbox.rightBumper().onTrue(new IntakeMoveToUp(intake));
      driverXbox.leftBumper().onTrue(new IntakeMoveToDown(intake));
      
      // ===== SYSID TESTING (COMMENT OUT FOR COMPETITION!) =====
      
      // D-Pad for SysId tests
      //driverXbox.leftTrigger().onTrue(Commands.runOnce(SignalLogger::start));
  //driverXbox.rightTrigger().onTrue(Commands.runOnce(SignalLogger::stop));
      driverXbox.leftTrigger().whileTrue(moveUp);
      driverXbox.rightTrigger().whileTrue(moveDown);
/*
 * Joystick Y = quasistatic forward
 * Joystick A = quasistatic reverse
 * Joystick B = dynamic forward
 * Joystick X = dyanmic reverse
 * 
 *
 */
    IntakeSubsystem i = new IntakeSubsystem();
    SysidCommand s = new SysidCommand(i.getSysID());
    driverXbox.y().whileTrue(s.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    driverXbox.a().whileTrue(s.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    driverXbox.b().whileTrue(s.sysIdDynamic(SysIdRoutine.Direction.kForward));
    driverXbox.x().whileTrue(s.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    // return drivebase.getAutonomousCommand("Copy of FrontGHAuto");
    return null;
  }

  public void setMotorBrake(boolean brake)
  {
    // drivebase.setMotorBrake(brake);
  }

  
}