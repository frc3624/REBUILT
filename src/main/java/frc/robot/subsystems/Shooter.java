package frc.robot.subsystems;
 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
 
import frc.robot.Constants.ShooterConstants;
 
public class Shooter extends SubsystemBase {
 
    private final SparkFlex leader =
        new SparkFlex(ShooterConstants.leadShooterID, MotorType.kBrushless);
        private final SparkFlex follower =
        new SparkFlex(ShooterConstants.followShooterID, MotorType.kBrushless);
    private final SparkFlex conveyor = new SparkFlex(ShooterConstants.conveyorID, MotorType.kBrushless);
 
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;
 
    private double targetRPM = 0;
 
    public Shooter() {
        SparkFlexConfig leaderConfig = new SparkFlexConfig();
 
        leaderConfig.idleMode(IdleMode.kCoast)
              .smartCurrentLimit(ShooterConstants.kCurrentLimit)
              .voltageCompensation(ShooterConstants.kVoltageComp)
              .inverted(false);
 
        // Configure PIDF for velocity control
        leaderConfig.closedLoop
              .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
              .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD).feedForward
              .kV(ShooterConstants.kFF);
 
        
            leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
 
        // Get encoder and PID controller
        encoder = leader.getEncoder();
        pidController = leader.getClosedLoopController();
    }
    
    public void setBothSpeed(double speed1, double speed2){
        leader.set(speed1);
        conveyor.set(speed2);
    }

    public void configureMotors(){
      SparkFlexConfig leaderConfig = new SparkFlexConfig();
        SparkFlexConfig followConfig = new SparkFlexConfig();
        leaderConfig.idleMode(IdleMode.kCoast)
              .smartCurrentLimit(ShooterConstants.kCurrentLimit)
              .voltageCompensation(ShooterConstants.kVoltageComp)
              .inverted(false);
 
        // Configure PIDF for velocity control
        leaderConfig.closedLoop
              .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
              .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD).feedForward
              .kV(ShooterConstants.kFF);
 
            followConfig.follow(ShooterConstants.leadShooterID, true);
            leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            follower.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Set shooter velocity in RPM using closed-loop control
     */
    public void setVelocity(double rpm) {
        targetRPM = rpm;
        pidController.setSetpoint(targetRPM, ControlType.kVelocity);
    }
 
    /**
     * Run shooter at open-loop percentage (for testing)
     */
    public void runOpenLoop(double percent) {
         targetRPM = 0;
         leader.set(percent);
    }
 
    /**
     * Stop the shooter
     */
    public void stop() {
        targetRPM = 0;
        leader.stopMotor();
    }

    public void setConveyorSpeed(double speed)
    {
      conveyor.set(speed);
    }
 
    /**
     * Check if shooter is at target speed
     */
    public boolean atSpeed() {
        return Math.abs(encoder.getVelocity() - targetRPM) < ShooterConstants.kToleranceRPM;
    }
 
    /**
     * Get current velocity in RPM
     */
    public double getVelocity() {
        return encoder.getVelocity();
    }
 
    /**
     * Get target velocity in RPM
     */
    public double getTargetVelocity() {
        return targetRPM;
    }
 
    /**
     * Get motor current draw in amps
     */
    public double getCurrent() {
        return leader.getOutputCurrent();
    }
 
    /**
     * Get velocity error in RPM
     */
    public double getError() {
        return targetRPM - encoder.getVelocity();
    }
    
    public void setShooterSpeed(double speed){
      leader.set(speed);
    }

    @Override
    public void periodic() {
        // Update dashboard for tuning
        SmartDashboard.putNumber("Flywheel/Current RPM", getVelocity());
        SmartDashboard.putNumber("Flywheel/Target RPM", targetRPM);
        SmartDashboard.putNumber("Flywheel/Error RPM", getError());
        SmartDashboard.putBoolean("Flywheel/At Speed", atSpeed());
        SmartDashboard.putNumber("Flywheel/Current Amps", getCurrent());
    }
}