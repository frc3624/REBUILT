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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import frc.robot.Constants.ShooterConstants;
 
public class Shooter extends SubsystemBase {
 
    private final SparkFlex leader = new SparkFlex(ShooterConstants.leadShooterID, MotorType.kBrushless);
    private final SparkFlex follower = new SparkFlex(ShooterConstants.followShooterID, MotorType.kBrushless);
 
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

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
        
        //Interpolating Tree Map, MAKE SURE THIS JAWN IS IN DOUBLES
        rpmMap.put(1.0, 1500.0);
    }
    
    public void setBothSpeed(double speed1, double speed2){
        leader.set(speed1);

    }

    public void configureMotors(){
      SparkFlexConfig leaderConfig = new SparkFlexConfig();
        SparkFlexConfig followConfig = new SparkFlexConfig();
        leaderConfig.idleMode(IdleMode.kCoast)
              .smartCurrentLimit(ShooterConstants.kCurrentLimit)
              .voltageCompensation(ShooterConstants.kVoltageComp)
              .inverted(false);

        leaderConfig.closedLoop
              .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
              .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD).feedForward
              .kV(ShooterConstants.kFF);
 
        followConfig.follow(ShooterConstants.leadShooterID, true);
        leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        follower.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setVelocity(double rpm) {
        targetRPM = rpm;
        pidController.setSetpoint(targetRPM, ControlType.kVelocity);
    }
 
    public void runOpenLoop(double percent) {
         targetRPM = 0;
         leader.set(percent);
    }

    public boolean atSpeed() {
        return Math.abs(encoder.getVelocity() - targetRPM) < ShooterConstants.kToleranceRPM;
    }
 
    public double getVelocity() {
        return encoder.getVelocity();
    }
 
    public double getTargetVelocity() {
        return targetRPM;
    }
 
    public double getCurrent() {
        return leader.getOutputCurrent();
    }
 
    public double getError() {
        return targetRPM - encoder.getVelocity();
    }
    
    public void setShooterSpeed(double speed){
      leader.set(speed);
    }

    public double getTargetRpm(double distance){
        return rpmMap.get(distance);
    }

    public void setRpm(double rpm){
        targetRPM = rpm;
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