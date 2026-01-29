package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.swervedrive.IntakeSubsystem;

public class SysidCommand {
    private final SysIdRoutine m_SysIdRoutine;
    public SysidCommand(SysIdRoutine SysIdRoutine)
    {
        this.m_SysIdRoutine = SysIdRoutine;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.quasistatic(direction);
     }
     
     public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.dynamic(direction);
     }
}
