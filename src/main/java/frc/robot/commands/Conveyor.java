package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ShooterConstants;
public class Conveyor extends Command {

    private final Shooter shooter;
    private final double speed;

    public Conveyor(Shooter shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setConveyorSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setConveyorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
