package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private RobotContainer container;

  @Override
  public void robotInit() {
    container = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void simulationPeriodic() {
    // IMPORTANT: Subsystem simulationPeriodic() is NOT auto-called.
    // We forward it ourselves via the container.
    container.simulationPeriodic();
  }
}
