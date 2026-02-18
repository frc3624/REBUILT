package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ShooterConstants;
public class ShooterCommand extends Command {

    private final Shooter shooter;
    private final double speed;

    public ShooterCommand(Shooter shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setVelocity(-speed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setVelocity(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
