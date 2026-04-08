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
 
public class Conveyor extends SubsystemBase {
 
    private final SparkFlex conveyor_m = new SparkFlex(ShooterConstants.conveyorID, MotorType.kBrushless);
    private final SparkFlex followConveyor_m = new SparkFlex(ShooterConstants.followConveyorID, MotorType.kBrushless);
    private final SparkFlex index_m = new SparkFlex(ShooterConstants.indexID, MotorType.kBrushless);

    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;
  
    public Conveyor() {
        configureMotors();
        pidController = conveyor_m.getClosedLoopController();
        encoder = conveyor_m.getEncoder();
    }

    public void configureMotors(){
        SparkFlexConfig leaderConfig = new SparkFlexConfig();
        SparkFlexConfig followConfig = new SparkFlexConfig();
        SparkFlexConfig indexConfig = new SparkFlexConfig();

        leaderConfig.closedLoop
              .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
              .pid(ShooterConstants.kCP, ShooterConstants.kCI, ShooterConstants.kCD).feedForward
              .kV(ShooterConstants.kCFF);
        leaderConfig.idleMode(IdleMode.kCoast)
              .smartCurrentLimit(ShooterConstants.kCurrentLimit)
              .voltageCompensation(ShooterConstants.kVoltageComp)
              .inverted(false);

        followConfig.follow(ShooterConstants.conveyorID, true);

        indexConfig.idleMode(IdleMode.kCoast)
              .smartCurrentLimit(ShooterConstants.kCurrentLimit)
              .voltageCompensation(ShooterConstants.kVoltageComp)
              .inverted(false);
        
        conveyor_m.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followConveyor_m.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        index_m.configure(indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);   
    }

     public void setVelocity(double rpm, double indexSpeed) {
        pidController.setSetpoint(rpm, ControlType.kVelocity);
        index_m.set(indexSpeed);
    }
    
    public void setSpeed(double conveyorSpeed, double indexSpeed){
        conveyor_m.set(conveyorSpeed);
        index_m.set(indexSpeed);
    }

    public double getConveyorSpeed(){
        return encoder.getVelocity();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Conveyor/Current RPM", getConveyorSpeed());
    }
  
}