package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.ShooterConstants;


public class armCommand extends Command {

    private final Intake intake;
    private final double speed;
    private final double position;
    public armCommand(Intake intake, double speed, double position) {
        this.intake = intake;
        this.speed = speed;
        this.position=position;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPosition(position);
    }
    @Override
    public void execute()
    {
            if(intake.getPosition() > position && intake.getPosition() < .9){
      intake.setArmSpeed(speed);
    }
    else if (intake.getPosition() < position || intake.getPosition() > .9){
      intake.setArmSpeed(-1*speed);
    }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setArmSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return intake.reached();
    }
}
