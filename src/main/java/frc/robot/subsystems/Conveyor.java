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
 
    private final RelativeEncoder encoder;
    
 
    private double targetRPM = 0;
 
    public Conveyor() {
        SparkFlexConfig leaderConfig = new SparkFlexConfig();
 
        leaderConfig.idleMode(IdleMode.kCoast)
              .smartCurrentLimit(ShooterConstants.kCurrentLimit)
              .voltageCompensation(ShooterConstants.kVoltageComp)
              .inverted(false);
 
        // Configure PIDF for velocity control
     
 
        
            conveyor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
 
        // Get encoder and PID controller
        encoder = conveyor.getEncoder();
        
    }
    
    public void setSpeed(double speed1){
        conveyor.set(speed1);
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