package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;

public class Gyro {
    private GyroIO io;
    private GyroIOInputsAutoLogged inputs;
    
    public Gyro(GyroIO io) {
        this.io = io;

        inputs = new GyroIOInputsAutoLogged();
    }

    public Rotation2d getYawHeading() {
        return inputs.angle;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Gyro", inputs);
    }
}
