package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSubsystemCommand extends Command {

    private final IntakeSubsystem subsystem;
    private final double speed;

    public IntakeSubsystemCommand(IntakeSubsystem subsystem, double speed) {
        this.subsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
