package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule {
    private SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    private SwerveModuleIO io;

    public void setModuleState(SwerveModuleState state) {
        // io.setTurnPosition();
        // io.setDriveVelocity();
    }

    public void periodic() {
        io.updateInputs(inputs);
        // Logger.processInputs(inputs);
    }
}