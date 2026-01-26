package frc.robot;

import frc.robot.commands.ShooterFromDashboard;
import frc.robot.subsystems.Shooter;
import frc.robot.util.ShotModel;

public class RobotContainer {
  private final Shooter shooter = new Shooter();
  private final ShotModel shotModel = new ShotModel();

  public RobotContainer() {
    // Runs continuously, and its initialize() will publish Sim/* keys
    shooter.setDefaultCommand(new ShooterFromDashboard(shooter, shotModel));
  }

  // Called from Robot.simulationPeriodic()
  public void simulationPeriodic() {
    shooter.simulationPeriodic();
  }
}
