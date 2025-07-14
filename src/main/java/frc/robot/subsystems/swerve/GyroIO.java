package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        Rotation2d angle = new Rotation2d();
    }

    // update the inputs
    public void updateInputs(GyroIOInputs inputs);
}
