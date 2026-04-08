package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Telemetry;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.util.LimelightHelpers;

public final class DrivetrainCommands {

    private DrivetrainCommands() {}

    public static Command teleopDrive(
            CommandSwerveDrivetrain drivetrain, 
            SwerveRequest.FieldCentric drive,
            CommandXboxController joystick,
            double maxSpeed,
            double maxAngularRate) {
        return drivetrain.applyRequest(
                () ->
                        drive.withVelocityX(-joystick.getLeftY() * maxSpeed)
                                .withVelocityY(-joystick.getLeftX() * maxSpeed)
                                .withRotationalRate(-joystick.getRightX() * maxAngularRate));
    }

    public static Command alignToTag(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive, CommandXboxController joystick, double maxSpeed, double targetTy, double angleOffset) {
        final double kP_Distance = VisionConstants.kP_Distance;
        final double kP_Rotation = VisionConstants.kP_Rotation;
        final String ll = VisionConstants.FRONT_LIMELIGHT;
        return drivetrain.applyRequest(
                () -> {
                    SwerveRequest request;
                    double distanceError = LimelightHelpers.getTY(ll) - targetTy;
                    if (LimelightHelpers.getTV(ll)) {
                        request =
                                drive.withVelocityX(distanceError * kP_Distance * (maxSpeed - 5))
                                        .withVelocityY(joystick.getLeftX() * maxSpeed)
                                        .withRotationalRate(
                                                (LimelightHelpers.getTX(ll) + angleOffset)
                                                        * kP_Rotation);
                    } else {
                        request =
                                drive.withVelocityX(0)
                                        .withVelocityY(0)
                                        .withRotationalRate(0);
                    }
                    return request;
                });
    }

    public static Command flipYaw180(CommandSwerveDrivetrain drivetrain) {
        return new InstantCommand(
                () ->
                        drivetrain
                                .getPigeon2()
                                .setYaw(
                                        drivetrain.getPigeon2().getYaw().getValueAsDouble() + 180),
                drivetrain);
    }

    public static Command disabledIdle(CommandSwerveDrivetrain drivetrain) {
        final var idle = new SwerveRequest.Idle();
        return drivetrain.applyRequest(() -> idle).ignoringDisable(true);
    }

    public static void registerTelemetry(CommandSwerveDrivetrain drivetrain, Telemetry logger) {
        drivetrain.registerTelemetry(logger::telemeterize);
    }
}
