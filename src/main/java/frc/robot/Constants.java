package frc.robot;

public final class Constants {
  public static final class OperatorConstants{
    public static final int kDriverControllerPort = 0;
  }

  public static final class ShooterConstants {
    public static final int kSparkFlexCanId = 1;

    // 1:1 motor to wheel
    public static final double kGearing = 1.0;

    // Two 4" Colsons (starter estimate). Refine later with SysId.
    public static final double kJ = 0.00155; // kg*m^2

    public static final double kMaxVolts = 12.0;

    // Feedforward starter guesses (RPM-based). Replace with SysId later.
    public static final double kS = 0.20;     // volts
    public static final double kV = 0.0020;   // volts per RPM
    public static final double kA = 0.00020;  // volts per (RPM/s)

    // PID starter guesses (RIO-side)
    public static final double kP = 0.0345;
    public static final double kI = 0.0;
    public static final double kD = 0.0003;

    public static final double kRpmTolerance = 75.0;
  }
}
