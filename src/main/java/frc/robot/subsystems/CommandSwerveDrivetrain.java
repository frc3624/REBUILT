package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.LimelightHelpers;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004;

    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private double distance;

    private final Field2d field = new Field2d();

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
        new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
        new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
        new SwerveRequest.SysIdSwerveRotation();

    private final SwerveRequest.FieldCentric alignRequest = new SwerveRequest.FieldCentric()
        .withDeadband(DriveConstants.maxSpeed * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(7),
            null,
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(Math.PI / 6).per(Second),
            Volts.of(Math.PI),
            null,
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        SmartDashboard.putData("Field", field);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        SmartDashboard.putData("Field", field);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        SmartDashboard.putData("Field", field);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configurePathPlanner() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        AutoBuilder.configure(
            () -> getState().Pose,
            this::resetPose,
            () -> getState().Speeds,
            (speeds, feedforwards) -> setControl(
                new SwerveRequest.ApplyRobotSpeeds()
                    .withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
            ),
            new PPHolonomicDriveController(
                new PIDConstants(8.0, 0, 0),
                new PIDConstants(8.0, 0, 0)
            ),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            },
            this
        );
    }

    public Command getAutonomousCommand(String autoName) {
        return new PathPlannerAuto(autoName);
    }

    public Field2d getField() {
        return field;
    }

    public Distance getShotDistance(Translation2d targetPose) {
        Pose2d drivePose = getState().Pose;
        double centerToTargetMeters = drivePose.getTranslation().getDistance(targetPose);
        double centerToShooterMeters = DriveConstants.shooterSideOffset.in(Units.Meters);
        double shooterToTargetMeters =
            Math.sqrt(Math.pow(centerToTargetMeters, 2.0) - Math.pow(centerToShooterMeters, 2.0));
        return Units.Meters.of(shooterToTargetMeters);
    }

    public Distance getShotDistance() {
        return getShotDistance(DriveConstants.getHubPose().toPose2d().getTranslation());
    }

    public Distance getFerryDistance() {
        return getShotDistance(
            DriveConstants.getFerryPose(getState().Pose.getTranslation()).toPose2d().getTranslation()
        );
    }

    public Command alignDrive(CommandXboxController controller, Supplier<Pose2d> targetPoseSupplier) {
        return applyRequest(() -> {
            double controllerVelX = -controller.getLeftY();
            double controllerVelY = -controller.getLeftX();

            Pose2d drivePose = getState().Pose;
            Pose2d targetPose = targetPoseSupplier.get();
            double shooterOffset = -DriveConstants.shooterSideOffset.in(Units.Meters);
            double targetDistance = drivePose.getTranslation().getDistance(targetPose.getTranslation());
            double shooterAngleRads = Math.acos(shooterOffset / targetDistance);
            Rotation2d shooterAngle = Rotation2d.fromRadians(shooterAngleRads);
            Rotation2d offsetAngle = Rotation2d.kCCW_90deg.minus(shooterAngle);
            Rotation2d shooterAngleOffset = Rotation2d.fromDegrees(2);
            Rotation2d desiredAngle = offsetAngle
                .plus(drivePose.relativeTo(targetPose).getTranslation().getAngle())
                .plus(Rotation2d.k180deg)
                .plus(shooterAngleOffset);
            Rotation2d currentAngle = drivePose.getRotation();
            Rotation2d deltaAngle = currentAngle.minus(desiredAngle);
            double wrappedAngleDeg = MathUtil.inputModulus(deltaAngle.getDegrees(), -180.0, 180.0);

            if (Math.abs(wrappedAngleDeg) < DriveConstants.epsilonAngleToGoal.in(Units.Degrees)
                && Math.hypot(controllerVelX, controllerVelY) < 0.1) {
                return new SwerveRequest.SwerveDriveBrake();
            } else {
                double rotationalRate = DriveConstants.rotationController.calculate(
                    currentAngle.getRadians(),
                    desiredAngle.getRadians()
                );
                return alignRequest
                    .withVelocityX(controllerVelX * DriveConstants.maxSpeed)
                    .withVelocityY(-controller.getLeftX() * DriveConstants.maxSpeed)
                    .withRotationalRate(-rotationalRate * DriveConstants.maxAngularRate);
            }
        });
    }

    public Command debugLimelight() {
        final String LL = "limelight";
        return run(() -> {
            boolean hasTarget = LimelightHelpers.getTV(LL);
            double tx = LimelightHelpers.getTX(LL);
            double[] rawPose = LimelightHelpers.getTargetPose_CameraSpace(LL);

            SmartDashboard.putBoolean("LL HasTarget", hasTarget);
            SmartDashboard.putNumber("LL tx", tx);
            SmartDashboard.putNumber("LL raw[0]", rawPose.length > 0 ? rawPose[0] : -999);
            SmartDashboard.putNumber("LL raw[1]", rawPose.length > 1 ? rawPose[1] : -999);
            SmartDashboard.putNumber("LL raw[2]", rawPose.length > 2 ? rawPose[2] : -999);
            SmartDashboard.putNumber("LL raw[3]", rawPose.length > 3 ? rawPose[3] : -999);
            SmartDashboard.putNumber("LL raw[4]", rawPose.length > 4 ? rawPose[4] : -999);
            SmartDashboard.putNumber("LL raw[5]", rawPose.length > 5 ? rawPose[5] : -999);
            SmartDashboard.putNumber("LL array length", rawPose.length);
        }).withName("DebugLimelight");
    }

    public Command followAprilTagLimelight() {
        final String LIMELIGHT_NAME = "limelight";
        final double kTurnP = 0.04;
        final double kForwardP = 0.5;
        final double targetDistance = 1.0;
        final double deadband = 0.2;

        SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

        return run(() -> {
            if (!LimelightHelpers.getTV(LIMELIGHT_NAME)) {
                setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
                return;
            }

            double tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
            double currentDistance = LimelightHelpers.getTargetPose3d_CameraSpace(LIMELIGHT_NAME)
                .getTranslation()
                .getZ();

            distance = currentDistance;

            double distanceError = currentDistance - targetDistance;
            double forwardSpeed = Math.abs(distanceError) < deadband
                ? 0
                : MathUtil.clamp(distanceError * kForwardP, -2.0, 2.0);

            double rotation = MathUtil.clamp(-tx * kTurnP, -1.5, 1.5);

            SmartDashboard.putNumber("LL Follow Distance", currentDistance);
            SmartDashboard.putNumber("LL Distance Error", distanceError);
            SmartDashboard.putNumber("LL Forward Speed", forwardSpeed);

            setControl(
                request.withVelocityX(forwardSpeed)
                    .withVelocityY(0)
                    .withRotationalRate(rotation)
            );
        }).withName("FollowAprilTag");
    }

    public Command driveWithAutoRotation(DoubleSupplier translationX, DoubleSupplier translationY) {
        final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        final double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond);
        final String LIMELIGHT_NAME = "limelight";
        final double kTurnP = 0.03;

        final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1);

        return run(() -> {
            double xSpeed = translationX.getAsDouble() * MaxSpeed;
            double ySpeed = translationY.getAsDouble() * MaxSpeed;

            Translation2d scaledTranslation = new Translation2d(xSpeed, ySpeed).times(0.8);

            double rotation = 0.0;
            if (LimelightHelpers.getTV(LIMELIGHT_NAME)) {
                double tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
                rotation = MathUtil.clamp(tx * kTurnP, -2.5, 2.5);
            }

            setControl(
                drive.withVelocityX(scaledTranslation.getX())
                    .withVelocityY(scaledTranslation.getY())
                    .withRotationalRate(rotation)
            );
        }).withName("DriveWithAutoRotation");
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        Pose2d odomPose = getState().Pose;

        field.setRobotPose(odomPose);
        SmartDashboard.putNumber("Odom X", odomPose.getX());
        SmartDashboard.putNumber("Odom Y", odomPose.getY());
        SmartDashboard.putNumber("Odom Heading Deg", odomPose.getRotation().getDegrees());
        SmartDashboard.putNumber("horizontalDistance to April Tag", distance);

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(
            visionRobotPoseMeters,
            Utils.fpgaToCurrentTime(timestampSeconds),
            visionMeasurementStdDevs
        );
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
}