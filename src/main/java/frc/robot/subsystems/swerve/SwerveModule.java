package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private SwerveModuleIOInputsAutoLogged inputs;

    private SwerveModuleIO io;
    private int index;

    private SwerveModuleState desiredModuleState;

    public SwerveModule(SwerveModuleIO io, int index) {
        this.io = io;
        this.index = index;

        inputs = new SwerveModuleIOInputsAutoLogged();
        desiredModuleState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    }

    public void setModuleState(SwerveModuleState state, boolean atRest) {
        // optimize the state
        state.optimize(inputs.turnPosition);
        state.cosineScale(inputs.turnPosition);

        // record the state for reference
        desiredModuleState = state;
        Logger.recordOutput("Module" + index + "/State", state);

        // pass stuff to io's
        io.setTurnPosition(state.angle);
        io.setDriveVelocity(state.speedMetersPerSecond / SwerveConstants.wheelRadiusMeters); // meters/sec -> rad/sec
    }

    public Rotation2d getTurnPosition() {
        return inputs.turnPosition;
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
            inputs.driveVelocityRadPerSec * SwerveConstants.wheelRadiusMeters, //  rad/sec -> meters/sec
            inputs.turnPosition
        );
    }

    public SwerveModuleState getDesiredModuleState() {
        return desiredModuleState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.drivePositionRad * SwerveConstants.wheelRadiusMeters, // position (rad) -> position (meters)
            inputs.turnPosition
        );
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Module" + index + "/Inputs", inputs);
    }
}