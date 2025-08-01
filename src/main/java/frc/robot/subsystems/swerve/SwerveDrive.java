package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Rotation;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {
    private SwerveModule[] modules;
    private SwerveModulePosition[] modulePositions;
    
    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator poseEstimator;

    private Gyro gyro;

    public SwerveDrive() {
        modules = new SwerveModule[] {
            new SwerveModule(new SwerveModuleIOSpark(0), 0),
            new SwerveModule(new SwerveModuleIOSpark(1), 1),
            new SwerveModule(new SwerveModuleIOSpark(2), 2),
            new SwerveModule(new SwerveModuleIOSpark(3), 3)
        };
        modulePositions = new SwerveModulePosition[4];
        kinematics = new SwerveDriveKinematics(
            new Translation2d(SwerveConstants.robotSizeMeters / 2, SwerveConstants.robotSizeMeters / 2),
            new Translation2d(SwerveConstants.robotSizeMeters / 2, -SwerveConstants.robotSizeMeters / 2),
            new Translation2d(-SwerveConstants.robotSizeMeters / 2, SwerveConstants.robotSizeMeters / 2),
            new Translation2d(-SwerveConstants.robotSizeMeters / 2, -SwerveConstants.robotSizeMeters / 2)
        );

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getYawHeading(), modulePositions, new Pose2d());
    }

    public Command runControllerInputs(
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier omegaSupplier,
        DoubleSupplier masterXSupplier,
        DoubleSupplier masterYSupplier,
        DoubleSupplier masterOmegaSupplier
    ) {
        return run(() -> {
            double masterX = masterXSupplier.getAsDouble();
            double masterY = masterYSupplier.getAsDouble();
            double masterOmega = masterOmegaSupplier.getAsDouble();

            double deadband = 0.2;
            boolean useMaster = Math.sqrt(Math.pow(masterX, 2) + Math.pow(masterY, 2)) > deadband 
                                || Math.abs(masterOmega) > deadband;

            double x =  useMaster ? masterX : xSupplier.getAsDouble();
            double y =  useMaster ? masterY : ySupplier.getAsDouble();
            double omega = useMaster ? masterOmega : omegaSupplier.getAsDouble();
            y *= -1; // rotated because joystick y is reversed on xbox
            omega *= -1; // rotation is reversed due to coordinate plane

            double vx = y;
            double vy = -x;

            omega = adjustAxisInput(omega, deadband);

            double magnitude = adjustAxisInput(Math.hypot(vx, vy), deadband) * SwerveConstants.maxVelocityMetersPerSec;
            double angle = Math.atan2(vy, vx);
            applyDriveInputs(Math.cos(angle) * magnitude, Math.sin(angle) * magnitude, omega, false);
        });
    }

    public Command runControllerInputs(
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier omegaSupplier
    ) {
        return run(() -> {
            double deadband = 0.2;

            double x = xSupplier.getAsDouble();
            double y = ySupplier.getAsDouble();
            double omega = omegaSupplier.getAsDouble();
            y *= -1; // rotated because joystick y is reversed on xbox
            omega *= -1; // rotation is reversed due to coordinate plane

            // convert to NWU
            double vx = y;
            double vy = -x;

            omega = adjustAxisInput(omega, deadband);

            double magnitude = adjustAxisInput(Math.hypot(vx, vy), deadband) * SwerveConstants.maxVelocityMetersPerSec;
            double angle = Math.atan2(vy, vx);
            applyDriveInputs(Math.cos(angle) * magnitude, Math.sin(angle) * magnitude, omega, false);
        });
    }

    public double adjustAxisInput(double x, double deadband) {
        return Math.abs(x) > deadband ?
            (x - Math.signum(x) * deadband) / (1-deadband)
        : 0; // leave linear, adjust for deadband
    }

    public void applyDriveInputs(double vx, double vy, double omega, boolean robotCentric) {
        boolean toX = true;
        boolean atRest = vx == 0 && vy == 0 && omega == 0;
        
        SwerveModuleState[] states;

        if(toX && atRest) {
            states = SwerveConstants.xPosStates;
        } else {
            ChassisSpeeds speeds;

            if (robotCentric) {
                speeds = new ChassisSpeeds(vx, vy, omega);
            } else {
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, gyro.getYawHeading());
            }
            states = kinematics.toSwerveModuleStates(speeds);
        }

        for (int i = 0; i < modules.length; i++) {
            states[i].optimize(modules[i].getTurnPosition());
            states[i].cosineScale(modules[i].getTurnPosition());
            
            modules[i].setModuleState(states[i], !toX && atRest);
        }
    }

    @Override
    public void periodic() {
        gyro.periodic();
        Arrays.stream(modules).forEach(module -> module.periodic());

        for (int i = 0; i < modules.length; i++) {
            modulePositions[i] = modules[i].getPosition();
        }

        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), gyro.getYawHeading(), modulePositions);
    }
}