package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Rotation;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

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
    // private SwerveDrivePoseEstimator poseEstimator;

    private Gyro gyro;

    public SwerveDrive() {
        modules = new SwerveModule[] {
            new SwerveModule(new SwerveModuleIOSpark(0), 0),
            new SwerveModule(new SwerveModuleIOSpark(1), 1),
            new SwerveModule(new SwerveModuleIOSpark(2), 2),
            new SwerveModule(new SwerveModuleIOSpark(3), 3)
        };
        modulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d())
        };

        double halfWheelDistance = SwerveConstants.wheelDistanceMeters / 2;
        double wheelRadius = SwerveConstants.wheelRadiusMeters;

        kinematics = new SwerveDriveKinematics(
            new Translation2d(halfWheelDistance - wheelRadius, halfWheelDistance - wheelRadius),
            new Translation2d(halfWheelDistance - wheelRadius, -halfWheelDistance + wheelRadius),
            new Translation2d(-halfWheelDistance + wheelRadius, halfWheelDistance - wheelRadius),
            new Translation2d(-halfWheelDistance + wheelRadius, -halfWheelDistance + wheelRadius)
        );

        gyro = new Gyro(new GyroIOPigeon());

        // poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getYawHeading(), modulePositions, new Pose2d());
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

            double magnitude = adjustAxisInput(Math.hypot(vx, vy), deadband) * SwerveConstants.maxWheelSpeedMetersPerSec;
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

            double magnitude = adjustAxisInput(Math.hypot(vx, vy), deadband) * SwerveConstants.maxChassisSpeedMagnitudeMetersPerSec;
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
            speeds = ChassisSpeeds.discretize(speeds, LoggedRobot.defaultPeriodSecs); // make the speeds realistic for a time period
            states = kinematics.toSwerveModuleStates(speeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.maxChassisSpeedMagnitudeMetersPerSec);

            Logger.recordOutput("Drive/ChassisSpeeds/Setpoints", speeds);
        }

        Logger.recordOutput("Drive/States/Setpoints", states);

        for (int i = 0; i < modules.length; i++) {
            states[i].optimize(modules[i].getTurnPosition());
            states[i].cosineScale(modules[i].getTurnPosition());
            
            modules[i].setModuleState(states[i], !toX && atRest);
        }
    }

    @Override
    public void periodic() {
        /* Call Interface Periodics */
        gyro.periodic();
        Arrays.stream(modules).forEach(module -> module.periodic());

        SwerveModuleState[] states = new SwerveModuleState[4];

        for (int i = 0; i < modules.length; i++) {
            modulePositions[i] = modules[i].getPosition();

            states[i] = modules[i].getCurrentState();
        }

        Logger.recordOutput("Drive/ActualStates", states);


        // poseEstimator.updateWithTime(Timer.getFPGATimestamp(), gyro.getYawHeading(), modulePositions);
    }
}