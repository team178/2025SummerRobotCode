package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public Rotation2d turnPosition = new Rotation2d();
        
    }

    // update the inputs
    public void updateInputs(SwerveModuleIOInputs inputs);

    // set turn position
    public void setTurnPosition(Rotation2d rot);
}