package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.Alerts;
import frc.robot.utils.Logger;
import frc.robot.utils.SparkMaxUtils;
import java.util.Optional;

public class ShooterSubsytem extends SubsystemBase {

    private final SparkMax lShooter, rShooter;

    public ShooterSubsytem() {
        lShooter = new SparkMax(ShooterConstants.LeftShooter.motorID, MotorType.kBrushless);
        rShooter = new SparkMax(ShooterConstants.RightShooter.motorID, MotorType.kBrushless);

        configureMotor(lShooter, true);
        configureMotor(rShooter, false);
    }

    @Override
    public void periodic() {
        Logger.log("shooter", this);
        Logger.log("shooter", "lShooter", lShooter, Optional.empty());
        Logger.log("shooter", "rShooter", rShooter, Optional.empty());

        Alerts.shooterLMotorDisconnected.set(!SparkMaxUtils.isConnected(lShooter));
        Alerts.shooterRMotorDisconnected.set(!SparkMaxUtils.isConnected(rShooter));
        Alerts.shooterLMotorOverheating.set(lShooter.getMotorTemperature() >= 80);
        Alerts.shooterRMotorOverheating.set(rShooter.getMotorTemperature() >= 80);
        Alerts.shooterLMotorFaults.set(SparkMaxUtils.hasCriticalFaults(lShooter.getFaults()));
        Alerts.shooterRMotorFaults.set(SparkMaxUtils.hasCriticalFaults(rShooter.getFaults()));
        Alerts.shooterLMotorWarnings.set(SparkMaxUtils.hasCriticalWarnings(lShooter.getWarnings()));
        Alerts.shooterRMotorWarnings.set(SparkMaxUtils.hasCriticalWarnings(rShooter.getWarnings()));
    }

    public void setDutyCycle(double dutyCycle) {
        lShooter.set(Math.min(
                Math.max(dutyCycle, -ShooterConstants.LeftShooter.maxDutyCycle),
                ShooterConstants.LeftShooter.maxDutyCycle));
        rShooter.set(Math.min(
                Math.max(dutyCycle, -ShooterConstants.RightShooter.maxDutyCycle),
                ShooterConstants.RightShooter.maxDutyCycle));
    }

    public void setVoltage(double volts) {
        // maybe should clamp in safe range, IDK how the sparkmaxes will handle a very large request
        lShooter.setVoltage(volts);
        rShooter.setVoltage(volts);
    }

    public void stop() {
        lShooter.set(0);
        rShooter.set(0);
    }

    private void configureMotor(SparkMax motor, boolean isLeft) {
        if (motor == null) {
            Logger.reportWarning("Cannot configure null shooter motor", true);
            return;
        }

        SparkMaxConfig config = new SparkMaxConfig();

        if (isLeft) {
            config.idleMode(ShooterConstants.LeftShooter.idleMode);
            config.inverted(ShooterConstants.LeftShooter.inverted);
            config.smartCurrentLimit(ShooterConstants.LeftShooter.maxCurrent);
            if (ShooterConstants.LeftShooter.voltageCompensation.isPresent()) {
                config.voltageCompensation(ShooterConstants.LeftShooter.voltageCompensation.get());
            } else {
                config.disableVoltageCompensation();
            }
            config.closedLoop.outputRange(
                    -ShooterConstants.LeftShooter.maxDutyCycle, ShooterConstants.LeftShooter.maxDutyCycle);
        } else {
            config.idleMode(ShooterConstants.RightShooter.idleMode);
            config.inverted(ShooterConstants.RightShooter.inverted);
            config.smartCurrentLimit(ShooterConstants.RightShooter.maxCurrent);
            if (ShooterConstants.RightShooter.voltageCompensation.isPresent()) {
                config.voltageCompensation(ShooterConstants.RightShooter.voltageCompensation.get());
            } else {
                config.disableVoltageCompensation();
            }
            config.closedLoop.outputRange(
                    -ShooterConstants.RightShooter.maxDutyCycle, ShooterConstants.RightShooter.maxDutyCycle);
        }

        if (motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
                != REVLibError.kOk) {
            Logger.reportError("Failed to configure " + (isLeft ? "left" : "right") + " shooter motor");
            if (isLeft) {
                Alerts.shooterLMotorConfigFail.set(true);
            } else {
                Alerts.shooterRMotorConfigFail.set(true);
            }
        }
    }
}
