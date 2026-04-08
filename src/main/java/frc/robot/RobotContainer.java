// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DrivetrainCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.util.Limelight;
import frc.robot.util.LimelightHelpers;

public class RobotContainer {
    //Swerve
    private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(MaxSpeed * 0.1)
                    .withRotationalDeadband(MaxAngularRate * 0.1)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //Subsystems
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Conveyor conveyor = new Conveyor();
    private final Limelight limelight = new Limelight(drivetrain);

    //Joysticks
    private final CommandXboxController joystick = new CommandXboxController(0);

    public RobotContainer() {
        configureBindings();

        //Swerve Orientation
        CommandScheduler.getInstance().schedule(DrivetrainCommands.flipYaw180(drivetrain));

        //Limelight
        limelight.useLimelight(true);
        LimelightHelpers.SetIMUMode("limelight", 1);
    }

    private void configureBindings() {

        //DriveTrain Bindings
        drivetrain.setDefaultCommand(DrivetrainCommands.teleopDrive(drivetrain, drive, joystick, MaxSpeed, MaxAngularRate));
        joystick.leftBumper().onTrue(DrivetrainCommands.flipYaw180(drivetrain));
        RobotModeTriggers.disabled().whileTrue(DrivetrainCommands.disabledIdle(drivetrain));
        DrivetrainCommands.registerTelemetry(drivetrain, logger);

        //Vision Bindings
        joystick.rightBumper().whileTrue(DrivetrainCommands.alignToTag(drivetrain, drive, joystick, MaxSpeed, VisionConstants.shortDistance, VisionConstants.ALIGN_TX_OFFSET_DEG));
        joystick.y().whileTrue(DrivetrainCommands.alignToTag(drivetrain, drive, joystick, MaxSpeed, VisionConstants.longDistance, VisionConstants.ALIGN_TX_OFFSET_DEG));

        //Shooter Bindings
        joystick.x().toggleOnTrue(ShooterCommands.shootSequence(shooter, conveyor, joystick));

        //Intake Bindings
        joystick.leftTrigger().whileTrue(IntakeCommands.armDown(intake));
        joystick.rightTrigger().whileTrue(IntakeCommands.armUp(intake));
        joystick.a().toggleOnTrue(IntakeCommands.intakeToggleWithRightRumble(intake, joystick));
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("MidShoot");
    }
}
