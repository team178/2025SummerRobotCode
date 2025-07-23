package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Rotation;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {
    private SwerveModule[] modules;
    
    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator poseEstimator;

    private Gyro gyro;

    public SwerveDrive() {
        kinematics = new SwerveDriveKinematics(
            new Translation2d(SwerveConstants.robotSizeMeters / 2, SwerveConstants.robotSizeMeters / 2),
            new Translation2d(SwerveConstants.robotSizeMeters / 2, -SwerveConstants.robotSizeMeters / 2),
            new Translation2d(-SwerveConstants.robotSizeMeters / 2, SwerveConstants.robotSizeMeters / 2),
            new Translation2d(-SwerveConstants.robotSizeMeters / 2, -SwerveConstants.robotSizeMeters / 2)
        );
    }

    public Command runControllerInputs(
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier omegaSupplier
    ) {
        return run(() -> {
            double x =  xSupplier.getAsDouble();
            double y =  -ySupplier.getAsDouble();//
            double omega = omegaSupplier.getAsDouble();//
            omega *= -1; // rotation is reversed due to coordinate plane

            double vx = y;
            double vy = -x;
            
            double deadband = 0.2;

            omega = adjustAxisInput(omega, deadband);

            double magnitude = adjustAxisInput(Math.hypot(vx, vy), deadband) * SwerveConstants.maxVelocityMetersPerSec;
            double angle = Math.atan2(vy, vx);
            applyDriveInputs(Math.cos(angle) * magnitude, Math.sin(angle) * magnitude, omega, false);//
        });
    }

    public double adjustAxisInput(double x, double deadband) {
        return Math.abs(x) > deadband ?
            (x - Math.signum(x) * deadband) / (1-deadband)
        : 0; // leave linear, adjust for deadband
    }

    public void applyDriveInputs(double vx, double vy, double omega, boolean robotCentric) {//
        boolean toX = true;

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, gyro.getYawHeading());
        if (robotCentric) speeds = new ChassisSpeeds(vx, vy, omega);//
        
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        if(toX && vx == 0 && vy == 0 && omega == 0) {
            states = new SwerveModuleState[]{
                new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
                new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
                new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
                new SwerveModuleState(0, new Rotation2d(Math.PI / 4))
            };
        }
        for (int i = 0; i < modules.length; i++) {
            states[i].optimize(modules[i].getTurnPosition());
            states[i].cosineScale(modules[i].getTurnPosition());
            
            modules[i].setModuleState(states[i]);
        }
    }

    @Override
    public void periodic() {
        gyro.periodic();
        Arrays.stream(modules).forEach(module -> module.periodic());
    }
}