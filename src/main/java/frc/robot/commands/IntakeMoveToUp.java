package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.swervedrive.IntakeSubsystem;

public class IntakeMoveToUp extends Command {

    private final IntakeSubsystem intake;

    public IntakeMoveToUp(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPosition(ArmConstants.INTAKE_UP_POSITION);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(
            intake.getPosition() - ArmConstants.INTAKE_UP_POSITION
        ) < ArmConstants.ARM_POSITION_TOLERANCE;
    }
}
