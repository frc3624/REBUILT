package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.StatusSignal;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX talonFX;
    private final SparkFlex motor13;
    private final SparkFlex motor14;
    private double lastSpeed = 0.0;

    public IntakeSubsystem() {
        motor13 = new SparkFlex(14, MotorType.kBrushless);
        motor14 = new SparkFlex(15, MotorType.kBrushless);
        talonFX = new TalonFX(0); // adjust to real motor ID
        configureMotors();
    }

    private void configureMotors() {
        SparkFlexConfig config = new SparkFlexConfig();
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();

        config.smartCurrentLimit(40);
        config.openLoopRampRate(0.5); 
        config.inverted(false);
        SparkFlexConfig config2 = new SparkFlexConfig();
        config2.follow(14, true);
        motor13.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motor14.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void setIntakeSpeed(double speed)
    {
        talonFX.set(speed);
    }

    public void setSpeed(double speed) {
        lastSpeed += (speed - lastSpeed) * 0.1;
        motor13.set(lastSpeed);
    }

    @Override
    public void periodic() {}
}
