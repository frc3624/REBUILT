package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

/** Shooter + conveyor shooting sequences (feeder is part of the shot). */
public final class ShooterCommands {

    private ShooterCommands() {}

    public static Command shootSequence(Shooter shooter, Conveyor conveyor, CommandGenericHID hid) {
        return Commands.sequence(
                Commands.runOnce(() -> shooter.setVelocity(ShooterConstants.rpm), shooter),
                Commands.waitUntil(shooter::atSpeed),
                Commands.parallel(
                                Commands.run(() -> shooter.setVelocity(ShooterConstants.rpm), shooter),
                                Commands.run(() -> conveyor.setVelocity(ShooterConstants.conveyorRPM, ShooterConstants.indexSpeed), conveyor),
                                Commands.runOnce(
                                        () -> hid.setRumble(GenericHID.RumbleType.kLeftRumble, 1.0)))
                .finallyDo(
                        interrupted -> {
                            conveyor.setSpeed(0, 0);
                            hid.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
                            shooter.setVelocity(0);
                        }));
    }
}
