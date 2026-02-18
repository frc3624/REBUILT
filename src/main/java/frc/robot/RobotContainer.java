// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.commands.IntakeArmCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Conveyor;


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
    private final Shooter shooter = new Shooter();

    //Commands
    private final IntakeArmCommand moveUp = new IntakeArmCommand(intakeSubsystem, Constants.IntakeArmConstants.speed);
    private final IntakeArmCommand moveDown = new IntakeArmCommand(intakeSubsystem, -Constants.IntakeArmConstants.speed);
    private final IntakeCommand runIntake = new IntakeCommand(intakeSubsystem, Constants.IntakeConstants.SPEED);
    private final Conveyor runConveyor = new Conveyor(shooter, Constants.ShooterConstants.conveyorSpeed);
    private final ShooterCommand runShooter = new ShooterCommand(shooter, .5);

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

        //shooter
        joystick.b().toggleOnTrue(runConveyor);
        joystick.y().toggleOnTrue(runShooter);
        joystick.x().whileTrue(new InstantCommand(() -> shooter.setBothSpeed(-.5, .3), shooter)).whileFalse(new InstantCommand(() -> shooter.setBothSpeed(0, 0), shooter));
        //joystick.rightBumper().whileTrue(new InstantCommand(() -> shooter.setShooterSpeed(.5), shooter));
        joystick.povUp().toggleOnTrue(shootSequence());
        //joystick.povUp().toggleOnTrue(new SequentialCommandGroup(new InstantCommand(() -> shooter.setVelocity(ShooterConstants.rpm), new WaitUntilCommand(() -> shooter.atSpeed()), new InstantCommand(shooter.setConveyorSpeed(.4), shooter))));

    }
    public Command shootSequence() {
        // 1. A command to set the shooter to a desired speed and keep it running
        //    until the whole sequence is finished. This should require the shooter subsystem.
        Command setShooterSpeed = Commands.startEnd(
            () -> shooter.setShooterSpeed(-.5),
            () -> shooter.setShooterSpeed(0), // Stop the shooter when the command group ends
           shooter
        );

        // 2. A command that waits until the shooter is on target speed. 
        //    This condition is checked periodically via the isFinished() method.
        Command waitUntilOnSpeed = new WaitUntilCommand(() -> shooter.atSpeed());

        // 3. A command to run the conveyor, which should require the conveyor subsystem.
        //    This command should run for a certain duration (e.g., 2 seconds) or until 
        //    a sensor indicates the ball has been shot.
        Command runConveyor = Commands.startEnd(
            () -> shooter.setConveyorSpeed(.4),
            () -> shooter.setConveyorSpeed(0),
            shooter
        ).withTimeout(2.0); // Adjust the timeout as needed, or use a sensor check

        // 4. Combine the commands in sequence using andThen().
        //    The `setShooterSpeed` command runs first and stays running (because it doesn't
        //    end on its own) while the `waitUntilOnSpeed` command waits.
        //    Once `waitUntilOnSpeed` finishes, the `runConveyor` command starts.
        //    You will need to ensure that `setShooterSpeed` and `runConveyor` can run 
        //    at the same time without conflicting requirements.

        return setShooterSpeed.andThen(waitUntilOnSpeed).andThen(runConveyor);
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
