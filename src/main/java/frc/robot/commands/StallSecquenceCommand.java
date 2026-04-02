package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;

public class StallSecquenceCommand extends Command {

    private final Intake intake;

    
    //joystick.leftTrigger().whileTrue(stallSecquenceCommand()).whileFalse(ff);
    //joystick.rightTrigger().whileTrue(Commands.runOnce( ()-> intake.setArmSpeed(0.15))).whileFalse(ff);
    //joystick.leftTrigger().toggleOnTrue(Commands.runOnce(() -> intake.setPosition(0)));
    public StallSecquenceCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        Commands.sequence(
         Commands.runOnce(() -> intake.setArmSpeed(-.15)),Commands.waitSeconds(0.285),Commands.runOnce(() ->intake.setArmSpeed(0.0005))
     );
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
