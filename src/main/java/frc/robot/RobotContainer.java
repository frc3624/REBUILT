// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import javax.sql.rowset.JoinRowSet;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.LimelightHelpers;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterLoopCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;

import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveConstants;
// Add imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class RobotContainer {
    //Swerve Stuff
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    //Subsystem
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Conveyor conveyor = new Conveyor();

    //Commands``````````````````

    private final IntakeCommand runIntake = new IntakeCommand(intake, Constants.IntakeConstants.SPEED);
    /*private final ConveyorCommand runConveyor = new ConveyorCommand(conveyor, -Constants.ShooterConstants.conveyorSpeed);
    private final ShooterCommand runShooter = new ShooterCommand(shooter, .5);
    private final ShooterLoopCommand runShooterWithLoop = new ShooterLoopCommand(shooter, Constants.ShooterConstants.rpm);*/
    private final LimelightHelpers limelight = new LimelightHelpers();

        double kP_Distance = 0.1; // Tuning constant for forward speed
    double TARGET_TY = -7.57;//The 'ty' value when you are at the perfect distance
    double kP_Rotation = 0.4;

    double TARGET2_TY = -3.8;
    //private final SendableChooser<Command> autoChooser;  // ADD THIS
    Command align = drivetrain.applyRequest(() -> {
    // No tag -> stop
    if (!LimelightHelpers.getTV("limelight")) {
        return drive.withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0);
    }

    double tx = LimelightHelpers.getTX("limelight");
    double ty = LimelightHelpers.getTY("limelight");

    double distanceError = ty - TARGET_TY;
    double rotationError = tx + 6; // if your offset is intentional

    return drive.withVelocityX(distanceError * kP_Distance * (MaxSpeed-10))
                .withVelocityY(0)
                .withRotationalRate(rotationError * kP_Rotation);
})
.until(() -> {
    if (!LimelightHelpers.getTV("limelight")) {
        return false; // don't finish if no target
    }

    double tx = LimelightHelpers.getTX("limelight");
    double ty = LimelightHelpers.getTY("limelight");

    return Math.abs(tx + 6) < 1.0   // rotation tolerance
        && Math.abs(ty - TARGET_TY) < 0.8; // distance tolerance
});
    public RobotContainer() {
        configureBindings();
        drivetrain.getPigeon2().setYaw(180);

        // Register named commands BEFORE building chooser
        NamedCommands.registerCommand("Shoot", shootSequenceOneCommand().withTimeout(2));
        NamedCommands.registerCommand("RunIntake", new IntakeCommand(intake, Constants.IntakeConstants.SPEED).withTimeout(4));
        NamedCommands.registerCommand("align", align);
        NamedCommands.registerCommand("intake", new IntakeCommand(intake, Constants.IntakeConstants.SPEED).withTimeout(5));
        //autoChooser = AutoBuilder.buildAutoChooser();  // ADD THIS
        //SmartDashboard.putData("Auto Chooser", autoChooser);  // ADD THIS
    }
    private void configureBindings() {
        //Swerve Buttons

        drivetrain.setDefaultCommand(
    drivetrain.applyRequest(() ->
        drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
             .withVelocityY(-joystick.getLeftX() * MaxSpeed)
             .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
    )
);

//intake.setDefaultCommand(Commands.runOnce(() -> intake.runFeedFoward(), intake));

// Constants - Adjust these for your robot's physical mounting

    joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> {
        // 1. Calculate the 'distance error'
        // If you are at the right spot, error = 0, so velocityX = 0
        double distanceError = LimelightHelpers.getTY("limelight") - TARGET_TY;

        
        if (LimelightHelpers.getTV("limelight")){
            return drive.withVelocityX(distanceError * kP_Distance * (MaxSpeed-5))
                    .withVelocityY(joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate((LimelightHelpers.getTX("limelight") +6) * kP_Rotation);
        }
        return drive.withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0);
        
    }));
        
        joystick.y().whileTrue(drivetrain.applyRequest(() -> {
        // 1. Calculate the 'distance error'
        // If you are at the right spot, error = 0, so velocityX = 0
        double distanceError = LimelightHelpers.getTY("limelight") - TARGET2_TY;

        /*if (distanceError < .01){
            //return drive.withVelocityX(0)
        return drive.withVelocityX(joystick.getRightX() * MaxSpeed)
                    .withVelocityY(0)
                    .withRotationalRate(LimelightHelpers.getTX("limelight") * kP_Rotation);
        }*/
        // 2. Drive with velocity relative to the error
        //return drive.withVelocityX(LimelightHelpers.getTY("limelight") * kP_Distance)
        if (LimelightHelpers.getTV("limelight")){
            return drive.withVelocityX(distanceError * kP_Distance * (MaxSpeed-5))
                    .withVelocityY(joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate((LimelightHelpers.getTX("limelight") +6) * kP_Rotation);
        }
        return drive.withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0);
    }));
    //joystick.rightBumper().whileTrue(drivetrain.alignDrive(joystick, () -> DriveConstants.getHubPose().toPose2d()));



        joystick.leftBumper().onTrue(
            new InstantCommand(() -> drivetrain.getPigeon2().setYaw(0), drivetrain)
        );



        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        //Intake Buttons
        
        //shooter
        // joystick.b().toggleOnTrue(runConveyor);
        // joystick.y().toggleOnTrue(runShooterWithLoop);
        //joystick.x().toggleOnTrue(shootSequenceTwo());

        //joystick.rightBumper().whileTrue(new InstantCommand(() -> shooter.setShooterSpeed(.5), shooter));
        joystick.x().toggleOnTrue(shootSequenceOneCommand());

        //joystick.povUp().toggleOnTrue(new SequentialCommandGroup(new InstantCommand(() -> shooter.setVelocity(ShooterConstants.rpm), new WaitUntilCommand(() -> shooter.atSpeed()), new InstantCommand(shooter.setConveyorSpeed(.4), shooter))));


        joystick.leftTrigger().whileTrue(stallSecquenceCommand()).whileFalse(Commands.runOnce( ()-> intake.setArmSpeed(0)));
        joystick.rightTrigger().whileTrue(Commands.runOnce( ()-> intake.setArmSpeed(0.1))).whileFalse(Commands.runOnce( ()-> intake.setArmSpeed(0)));
        //joystick.leftTrigger().toggleOnTrue(Commands.runOnce(() -> intake.setPosition(0)));
        
        //joystick.leftTrigger().toggleOnTrue(moveDown);
        //joystick.rightTrigger().toggleOnTrue(moveUp);
   
        joystick.a().toggleOnTrue(Commands.parallel(runIntake, Commands.runOnce(() -> joystick.setRumble(GenericHID.RumbleType.kRightRumble,1.0))).finallyDo(() -> joystick.setRumble(GenericHID.RumbleType.kRightRumble, 0.0)));
    }

/////
public Command shootSequenceOneCommand() { 
    return Commands.sequence(
        Commands.runOnce(() -> shooter.setVelocity(-ShooterConstants.rpm), shooter),
        Commands.waitUntil(shooter::atSpeed),
        Commands.waitSeconds(.5),
        Commands.parallel(
            Commands.run(() -> shooter.setVelocity(-ShooterConstants.rpm), shooter),
            Commands.run(() -> conveyor.setVelocity(-4000, .25), conveyor),
             Commands.runOnce(() -> joystick.setRumble(GenericHID.RumbleType.kLeftRumble, 1.0))
        ).finallyDo(interrupted -> {
            conveyor.setSpeed(0, 0);
            joystick.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
            shooter.setVelocity(0);
        })
    );
}


public Command shootAutoCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> shooter.setVelocity(-ShooterConstants.rpm), shooter),
        Commands.waitUntil(shooter::atSpeed),
        Commands.waitSeconds(0.5),
        shootSequenceOneCommand().withTimeout(1.0)
    );
}


public Command stallSecquenceCommand() {
     return Commands.sequence(
         Commands.runOnce(() -> intake.setArmSpeed(-.1)),Commands.waitSeconds(0.285),Commands.runOnce(() ->intake.setArmSpeed(0.0005))
     );
 }


    public Command getAutonomousCommand() {
        return drivetrain.getAutonomousCommand("StraightAutoTest");
    }
    
}
