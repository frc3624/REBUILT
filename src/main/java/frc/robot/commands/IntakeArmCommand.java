package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeArmCommand extends Command {

    private final Intake intake;
    private final double speed;

    public IntakeArmCommand(Intake intake, double speed) {
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
