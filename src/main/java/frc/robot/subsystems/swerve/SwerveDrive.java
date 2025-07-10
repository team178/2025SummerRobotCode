package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private SwerveModule[] modules;
    
    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator poseEstimator;

    public SwerveDrive() {

    }

    public Command runControllerInputs(
        DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier
    ) {
        return run(() -> {
            
        });
    }

    public double adjustAxisInput(double x) {
        return x; // leave linear
    }


}