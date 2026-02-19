package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyor;
import frc.robot.Constants.ShooterConstants;


public class ConveyorCommand extends Command {

    private final Conveyor conveyor;
    private final double speed;

    public ConveyorCommand(Conveyor conveyor, double speed) {
        this.conveyor = conveyor;
        this.speed = speed;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.setSpeed(-speed);
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
