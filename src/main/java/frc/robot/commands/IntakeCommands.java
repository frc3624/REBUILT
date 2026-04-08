package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Intake;

public final class IntakeCommands {

    private IntakeCommands() {
    }

    public static Command holdArmSpeed(Intake intake, double speed) {
        return Commands.startEnd(
                () -> intake.setArmSpeed(speed),
                () -> intake.setArmSpeed(0),
                intake);
    }

    public static Command armUp(Intake intake) {
        return holdArmSpeed(intake, ArmConstants.upSpeed);
    }

    public static Command armDown(Intake intake) {
        return holdArmSpeed(intake, ArmConstants.downSpeed);
    }

    public static Command stallSequence(Intake intake) {
        return Commands.sequence(
                Commands.runOnce(() -> intake.setArmSpeed(ArmConstants.initialSpeed), intake),
                Commands.waitSeconds(ArmConstants.waitTime),
                Commands.runOnce(() -> intake.setArmSpeed(ArmConstants.finalSpeed), intake));
    }

    public static Command runIntakeSpeedOnce(Intake intake, double speed) {
        return Commands.startEnd(
                () -> intake.setIntakeSpeed(speed),
                () -> intake.setIntakeSpeed(0),
                intake);
    }

    public static Command runIntakeSpeed(Intake intake) {
        return runIntakeSpeedOnce(intake, Constants.IntakeConstants.SPEED);
    }

    public static Command intakeToggleWithRightRumble(Intake intake, CommandGenericHID hid) {
        return Commands.parallel(
                        runIntakeSpeed(intake),
                        Commands.runOnce(() -> hid.setRumble(GenericHID.RumbleType.kRightRumble, 1.0)))
                .finallyDo(() -> hid.setRumble(GenericHID.RumbleType.kRightRumble, 0.0));
    }
}