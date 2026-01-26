package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;
import frc.robot.util.ShotModel;

public class ShooterFromDashboard extends Command {
  private final Shooter shooter;
  private final ShotModel model;

  // Simple filter to reduce jitter
  private double filteredDistance = 2.5;

  public ShooterFromDashboard(Shooter shooter, ShotModel model) {
    this.shooter = shooter;
    this.model = model;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("Sim/DistanceMeters", 2.5);
    SmartDashboard.putBoolean("Sim/EnableShooter", false);
  }

  @Override
  public void execute() {
    boolean enabled = SmartDashboard.getBoolean("Sim/EnableShooter", false);
    double d = SmartDashboard.getNumber("Sim/DistanceMeters", 2.5);

    // 1-pole low-pass filter
    filteredDistance = 0.85 * filteredDistance + 0.15 * d;

    if (!enabled) {
      shooter.setTargetRpm(0);
      return;
    }

    double rpm = model.rpmForDistance(filteredDistance);
    shooter.setTargetRpm(rpm);
  }

  @Override
  public boolean isFinished() {
    return false; // default command style
  }
}
