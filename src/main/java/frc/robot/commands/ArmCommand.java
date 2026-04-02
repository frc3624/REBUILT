package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ArmCommand extends Command {

    private final Intake intake;
    private final double speed;

    
    //joystick.leftTrigger().whileTrue(stallSecquenceCommand()).whileFalse(ff);
    //joystick.rightTrigger().whileTrue(Commands.runOnce( ()-> intake.setArmSpeed(0.15))).whileFalse(ff);
    //joystick.leftTrigger().toggleOnTrue(Commands.runOnce(() -> intake.setPosition(0)));
    public ArmCommand(Intake intake, double speed) {
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setArmSpeed(speed);
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
