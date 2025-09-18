package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private SwerveModuleIOInputsAutoLogged inputs;

    private SwerveModulePosition position;

    private SwerveModuleIO io;
    private int index;

    public SwerveModule(SwerveModuleIO io, int index) {
        this.io = io;
        this.index = index;

        inputs = new SwerveModuleIOInputsAutoLogged();
    }

    public void setModuleState(SwerveModuleState state, boolean atRest) {
        double speedMetersPerMinute = state.speedMetersPerSecond * 60;
        double wheelRPM = speedMetersPerMinute / SwerveConstants.wheelCircumference;
        double motorRPM = wheelRPM * SwerveConstants.gearRatio; //how many motor rotations for each wheel rotation

        io.setDriveVelocity(motorRPM);
        Logger.recordOutput("Module" + index + "/State", state);
    }

    public Rotation2d getTurnPosition() {
        return inputs.turnPosition;
    }

    public SwerveModulePosition getPosition() {
        return position;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Module" + index + "/Inputs", inputs);

        position = new SwerveModulePosition(inputs.drivePositionMeters, inputs.turnPosition);
    }
}