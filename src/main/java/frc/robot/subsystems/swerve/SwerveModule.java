package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule {
    private SwerveModuleIOInputsAutoLogged inputs;

    private SwerveModuleIO io;
    private int index;

    public SwerveModule(SwerveModuleIO io, int index) {
        this.io = io;
        this.index = index;

        inputs = new SwerveModuleIOInputsAutoLogged();
    }

    public void setModuleState(SwerveModuleState state) {
        io.setTurnPosition(state.angle);
        io.setDriveVelocity(state.speedMetersPerSecond);
        Logger.recordOutput("Module" + index + "/State", state);
    }

    public Rotation2d getTurnPosition() {
        return inputs.turnPosition;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Module" + index + "/Inputs", inputs);
    }
}