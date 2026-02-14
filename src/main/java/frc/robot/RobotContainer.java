// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.commands.IntakeArmCommand;
import frc.robot.commands.IntakeCommand;

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
    private final Intake intakeSubsystem = new Intake();

    //Commands
    private final IntakeArmCommand moveUp = new IntakeArmCommand(intakeSubsystem, Constants.IntakeArmConstants.speed);
    private final IntakeArmCommand moveDown = new IntakeArmCommand(intakeSubsystem, -Constants.IntakeArmConstants.speed);
    private final IntakeCommand runIntake = new IntakeCommand(intakeSubsystem, Constants.IntakeConstants.SPEED);

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

        joystick.leftBumper().onTrue(
            new InstantCommand(() -> drivetrain.getPigeon2().setYaw(0), drivetrain)
        );


        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        //Intake Buttons
        joystick.rightTrigger().whileTrue(moveUp);
        joystick.leftTrigger().whileTrue(moveDown);
        joystick.a().toggleOnTrue(runIntake);
    }

    public Command getAutonomousCommand() {
        final var idle = new SwerveRequest.Idle();
        return drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero))
            .andThen(
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0)
                ).withTimeout(5.0)
            )
            .andThen(drivetrain.applyRequest(() -> idle));
    }
}
