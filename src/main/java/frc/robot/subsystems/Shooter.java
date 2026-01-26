package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final SparkFlex motor = new SparkFlex(Constants.ShooterConstants.kSparkFlexCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();

  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(
      Constants.ShooterConstants.kS,
      Constants.ShooterConstants.kV,
      Constants.ShooterConstants.kA
  );

  private final PIDController pid = new PIDController(
      Constants.ShooterConstants.kP,
      Constants.ShooterConstants.kI,
      Constants.ShooterConstants.kD
  );

  private double targetRpm = 0.0;
  private double lastVoltsCommanded = 0.0;

  // Simulation plant
  private final FlywheelSim sim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
          DCMotor.getNeoVortex(1),
          Constants.ShooterConstants.kJ,
          Constants.ShooterConstants.kGearing
      ),
      DCMotor.getNeoVortex(1),
      0.0
  );

  public Shooter() {
    pid.setTolerance(Constants.ShooterConstants.kRpmTolerance);
  }

  public void setTargetRpm(double rpm) {
    targetRpm = Math.max(0.0, rpm);
  }

  public double getTargetRpm() {
    return targetRpm;
  }

  public double getRpm() {
    if (RobotBase.isSimulation()) {
      return sim.getAngularVelocityRPM();
    }
    return encoder.getVelocity(); // REV relative encoder velocity is typically RPM
  }

  public boolean atSpeed() {
    return pid.atSetpoint();
  }

  @Override
  public void periodic() {
    double measuredRpm = getRpm();

    // PID output in volts (since we treat the PID output as a voltage term)
    double pidVolts = pid.calculate(measuredRpm, targetRpm);

    // FF in volts (RPM-based guesses for now; replace with SysId later)
    double ffVolts = ff.calculate(targetRpm);

    double volts = MathUtil.clamp(pidVolts + ffVolts, -Constants.ShooterConstants.kMaxVolts, Constants.ShooterConstants.kMaxVolts);
    lastVoltsCommanded = volts;

    motor.setVoltage(volts);

    SmartDashboard.putNumber("Shooter/TargetRPM", targetRpm);
    SmartDashboard.putNumber("Shooter/MeasuredRPM", measuredRpm);
    SmartDashboard.putNumber("Shooter/Volts", volts);
    SmartDashboard.putBoolean("Shooter/AtSpeed", atSpeed());
  }

  @Override
  public void simulationPeriodic() {
    sim.setInputVoltage(lastVoltsCommanded);
    sim.update(0.02);
  }
}
