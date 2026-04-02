package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.IntakeArmConstants;;
public class IntakeFeedfowardCommand extends Command {
    private final Intake intake;

    public IntakeFeedfowardCommand(Intake intake){
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        //intake.runFeedFoward();

    }


    @Override
    public void end(boolean interrupted) {
        intake.setArmSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
