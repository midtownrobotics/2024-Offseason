package frc.robot.subsystems.Drivetrain.SwerveModuleIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private final DCMotorSim m_driveMotor = new DCMotorSim(DCMotor.getNEO(1), Constants.NeoSwerveModuleConstants.DRIVING_MOTOR_REDUCTION, 0.025);
    private final DCMotorSim m_turnMotor = new DCMotorSim(DCMotor.getNEO(1), Constants.NeoSwerveModuleConstants.TURNING_MOTOR_REDUCTION, 0.004);        

    private final PIDController m_drivingPIDController = new PIDController(0.4, 0, 0, 0.02);
	private final PIDController m_turningPIDController = new PIDController(Constants.NeoSwerveModuleConstants.TURNING_P.get(), 0, 0, 0.02);

    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    private double driveAppliedVolts;
    private double turnAppliedVolts;

    public SwerveModuleIOSim() {
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        m_driveMotor.update(0.02);
        m_turnMotor.update(0.02);

        inputs.turningCurrentAmps = m_turnMotor.getCurrentDrawAmps();
        inputs.turningVelocityRPM = m_turnMotor.getAngularVelocityRPM();
        inputs.turningIsOn = Math.abs(m_turnMotor.getAngularVelocityRPM()) > 0.01;
        inputs.turningVoltage = turnAppliedVolts;

        inputs.drivingCurrentAmps = m_driveMotor.getCurrentDrawAmps();
        inputs.drivingVelocityRPM = m_driveMotor.getAngularVelocityRPM();
        inputs.drivingIsOn = Math.abs(m_driveMotor.getAngularVelocityRPM()) > 0.01;
        inputs.drivingVoltage = driveAppliedVolts;

        inputs.currentState = getState();
        inputs.desiredState = getDesiredState();
    }

    @Override
    public SwerveModuleState getDesiredState() {
        return m_desiredState;
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turnMotor.getAngularPositionRad()));

        m_desiredState = optimizedState;

        if (Math.abs(optimizedState.speedMetersPerSecond) < 0.001 && Math.abs(optimizedState.angle.getRadians() - m_turnMotor.getAngularPositionRad()) < Rotation2d.fromDegrees(1).getRadians()) {
            m_driveMotor.setInputVoltage(0);
            m_turnMotor.setInputVoltage(0);
            return;
        }

        double wheelRadius = Constants.NeoSwerveModuleConstants.WHEEL_DIAMETER_METERS / 2;
        double velocityRadsPerSec = optimizedState.speedMetersPerSecond / wheelRadius;
        double driveVolts = m_drivingPIDController.calculate(m_driveMotor.getAngularVelocityRadPerSec(), velocityRadsPerSec);
        driveAppliedVolts = MathUtil.clamp(driveVolts, -12.0, 12.0);
        m_driveMotor.setInputVoltage(driveAppliedVolts);

        double angleRads = optimizedState.angle.getRadians();
        double turnVolts = m_turningPIDController.calculate(m_turnMotor.getAngularPositionRad(), angleRads);
        turnAppliedVolts = MathUtil.clamp(turnVolts, -12.0, 12.0);
        m_turnMotor.setInputVoltage(turnAppliedVolts);
    }

    @Override
    public SwerveModuleState getState() {
        double velocity = m_driveMotor.getAngularVelocityRPM() * Constants.NeoSwerveModuleConstants.WHEEL_DIAMETER_METERS * Math.PI / 60;
        Rotation2d angle = new Rotation2d(m_turnMotor.getAngularPositionRad());
        
        return new SwerveModuleState(velocity, angle);
    }

    @Override
    public SwerveModulePosition getPosition() {
        double distance = m_driveMotor.getAngularPositionRotations() * Constants.NeoSwerveModuleConstants.WHEEL_DIAMETER_METERS * Math.PI;
        Rotation2d angle = new Rotation2d(m_turnMotor.getAngularPositionRad());

        return new SwerveModulePosition(distance, angle);
    }
}
