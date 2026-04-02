// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.Limelight;
import frc.robot.util.LimelightHelpers;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override

    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        var drivetrainState = m_robotContainer.drivetrain.getState();

        // Current robot yaw/rate from drivetrain state
        double gyroYawDeg = drivetrainState.Pose.getRotation().getDegrees();
        double omegaDegPerSec = Math.toDegrees(drivetrainState.Speeds.omegaRadiansPerSecond);

        // Required for MegaTag2
        LimelightHelpers.SetRobotOrientation(
            "limelight",
            gyroYawDeg,
            omegaDegPerSec,
            0, 0, 0, 0
        );

        // Get pose estimate in the correct alliance frame
        LimelightHelpers.PoseEstimate llMeasurement;
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
        } else {
            llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        }

        if (llMeasurement == null) return;
        if (llMeasurement.tagCount <= 0) return;

        // Basic rejection filters used by many teams
        boolean spinningTooFast = Math.abs(omegaDegPerSec) > 540.0;
        boolean badSingleTag = llMeasurement.tagCount == 1
                && (llMeasurement.avgTagArea < 0.1 || llMeasurement.rawFiducials[0].ambiguity > 0.7);

        if (spinningTooFast || badSingleTag) return;

        // Optional: tune these based on confidence
        if (llMeasurement.tagCount >= 2) {
            if (llMeasurement.isMegaTag2) {
                m_robotContainer.drivetrain.setVisionMeasurementStdDevs(
                    edu.wpi.first.math.VecBuilder.fill(0.4, 0.4, 60000000)
                );
            }
            else {
                m_robotContainer.drivetrain.setVisionMeasurementStdDevs(
                    edu.wpi.first.math.VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(20))
                );
            }
        } else {
            if (llMeasurement.isMegaTag2) {
                m_robotContainer.drivetrain.setVisionMeasurementStdDevs(
                    edu.wpi.first.math.VecBuilder.fill(0.7, 0.7, 60000000)
                );
            }
            else{
                m_robotContainer.drivetrain.setVisionMeasurementStdDevs(
                    edu.wpi.first.math.VecBuilder.fill(0.9, 0.9, Units.degreesToRadians(30))
                );
            }
        }

        m_robotContainer.drivetrain.addVisionMeasurement(
            llMeasurement.pose,
            llMeasurement.timestampSeconds
        );
    }
    
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
