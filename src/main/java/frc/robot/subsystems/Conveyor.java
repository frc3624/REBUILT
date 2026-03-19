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
 
    private final SparkFlex conveyor = new SparkFlex(ShooterConstants.conveyorID, MotorType.kBrushless);
    private final SparkFlex followConveyor = new SparkFlex(ShooterConstants.followConveyorID, MotorType.kBrushless);
    private final SparkFlex preIndexer = new SparkFlex(23, MotorType.kBrushless);

    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;
 
    private double targetRPM = 0;
 
    public Conveyor() {
        SparkFlexConfig leaderConfig = new SparkFlexConfig();
        SparkFlexConfig followConfig = new SparkFlexConfig();
        SparkFlexConfig indexConfig = new SparkFlexConfig();
        leaderConfig.closedLoop
              .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
              .pid(ShooterConstants.kCP, ShooterConstants.kCI, ShooterConstants.kCD).feedForward
              .kV(ShooterConstants.kCFF);
       pidController = conveyor.getClosedLoopController();
 

        leaderConfig.idleMode(IdleMode.kCoast)
              .smartCurrentLimit(ShooterConstants.kCurrentLimit)
              .voltageCompensation(ShooterConstants.kVoltageComp)
              .inverted(false);
        indexConfig.idleMode(IdleMode.kCoast)
              .smartCurrentLimit(ShooterConstants.kCurrentLimit)
              .voltageCompensation(ShooterConstants.kVoltageComp)
              .inverted(false);
        // Configure PIDF for velocity control
        //indexConfig.follow(ShooterConstants.conveyorID, true);
        followConfig.follow(ShooterConstants.conveyorID, true);
        

        conveyor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followConveyor.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        preIndexer.configure(indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // Get encoder and PID controller
        encoder = conveyor.getEncoder();
        
        
    }

     public void setVelocity(double rpm, double speed) {
        targetRPM = rpm;
        pidController.setSetpoint(targetRPM, ControlType.kVelocity);
        preIndexer.set(speed);
    }
    
    public void setSpeed(double speed1, double speed2){
        conveyor.set(speed1);
        preIndexer.set(speed2);
    }


    public void configureMotors(){
      SparkFlexConfig leaderConfig = new SparkFlexConfig();

        leaderConfig.idleMode(IdleMode.kCoast)
              .smartCurrentLimit(ShooterConstants.kCurrentLimit)
              .voltageCompensation(ShooterConstants.kVoltageComp)
              .inverted(false);
 
       
 
            
            conveyor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
           
    }

    public double getConveyorSpeed()
    {
        return encoder.getVelocity();
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Conveyor/Current RPM", getConveyorSpeed());
    }
  
}