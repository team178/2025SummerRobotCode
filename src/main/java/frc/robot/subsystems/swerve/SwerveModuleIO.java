package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public Rotation2d turnPosition = new Rotation2d();
        public double speedMetersPerSecond = 0;
        public double drivePositionMeters = 0;
    }

    /** Update and push the inputs from the module to the higher level code */
    public void updateInputs(SwerveModuleIOInputs inputs);

    /** Set the turn position of the module */
    public void setTurnPosition(Rotation2d angle);

    /** Set the drive velocity
     * 
     * @param revolutionsPerMinute Speed in meters per second
     */
    public void setDriveVelocity(double revolutionsPerMinute);
}