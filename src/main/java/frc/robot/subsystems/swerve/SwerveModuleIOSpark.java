package frc.robot.subsystems.swerve;


import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants.SwerveModuleConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SwerveModuleIOSpark implements SwerveModuleIO {
    private SparkMax turnMotor;
    private SparkMax driveMotor;

    public SwerveModuleIOSpark(int index) {
        turnMotor = new SparkMax(SwerveModuleConstants.turnCANIds[index], MotorType.kBrushless);
        driveMotor = new SparkMax(SwerveModuleConstants.driveCANIds[index], MotorType.kBrushless);

        turnMotor.configure(SwerveModuleConstants.turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setTurnPosition(Rotation2d rot) {
        turnMotor.getClosedLoopController().setReference(rot.getRadians(), ControlType.kPosition);
    }

    public void setDriveVelocity(double vel) {
        driveMotor.getClosedLoopController().setReference(vel, ControlType.kVelocity);
    }

    // public SwerveModuleState optimizeState(SwerveModuleState defaultState) {
    //     double angle = defaultState.angle.getRadians();
    //     double speed = defaultState.speedMetersPerSecond;
    //     turnMotor.getAbsoluteEncoder().getPosition();


    // }

   @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        // inputs.turnPosition = 
    }
}