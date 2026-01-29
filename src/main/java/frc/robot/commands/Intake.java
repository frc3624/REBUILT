package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.swervedrive.IntakeSubsystem;

public class Intake extends Command {
  private final IntakeSubsystem intake;
  private double speed;

  public Intake(IntakeSubsystem intake, double speed) {
    this.intake = intake;
    this.speed = speed;
    //addRequirements(intake);
  }

  @Override
  public void initialize() {}

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
