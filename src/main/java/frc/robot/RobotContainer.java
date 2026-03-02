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
import frc.robot.commands.IntakeArmCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterLoopCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;

import frc.robot.commands.ConveyorCommand;
import frc.robot.subsystems.Conveyor;

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
    private final IntakeArmCommand moveUp = new IntakeArmCommand(intake, -.2);
    private final IntakeArmCommand stallArmCommand = new IntakeArmCommand(intake, -0.05);
    private final IntakeArmCommand moveDown = new IntakeArmCommand(intake,.1);
    private final IntakeCommand runIntake = new IntakeCommand(intake, Constants.IntakeConstants.SPEED);
    private final ConveyorCommand runConveyor = new ConveyorCommand(conveyor, -Constants.ShooterConstants.conveyorSpeed);
    private final ShooterCommand runShooter = new ShooterCommand(shooter, .5);
    private final ShooterLoopCommand runShooterWithLoop = new ShooterLoopCommand(shooter, Constants.ShooterConstants.rpm);
    private final LimelightHelpers limelight = new LimelightHelpers();
    
    public RobotContainer() {
        configureBindings();
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
        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(LimelightHelpers.getTY("limelight") * -0.1).withVelocityY(joystick.getLeftX() * MaxSpeed).withRotationalRate(LimelightHelpers.getTX("limelight") * -0.05)));
        intake.setDefaultCommand(stallArmCommand);

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

        joystick.rightTrigger().whileTrue(moveUp);
        joystick.leftTrigger().whileTrue(moveDown);
        joystick.a().toggleOnTrue(runIntake);

    }

/////
public Command shootSequenceOneCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> shooter.setVelocity(-ShooterConstants.rpm), shooter),
        Commands.waitUntil(shooter::atSpeed),
        Commands.waitSeconds(.5),
        Commands.parallel(
            Commands.run(() -> shooter.setVelocity(-ShooterConstants.rpm), shooter),
            Commands.run(() -> conveyor.setSpeed(-0.25), conveyor)
        ).finallyDo(interrupted -> {
            conveyor.setSpeed(0);
            shooter.setVelocity(0);
        })
    );
}

public Command stallSecquenceCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> intake.setArmSpeed(.1)),Commands.waitSeconds(0.15),Commands.runOnce(() ->intake.setArmSpeed(-0.075))
    );
}


    public Command getAutonomousCommand() {
        return shootSequenceOneCommand();
    }
}
