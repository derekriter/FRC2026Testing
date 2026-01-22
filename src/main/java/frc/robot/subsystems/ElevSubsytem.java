package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevConstants;
import frc.robot.units.ElevLevel;
import frc.robot.units.ElevPosition;
import frc.robot.utils.Alerts;
import frc.robot.utils.Logger;
import frc.robot.utils.TalonFXUtils;
import java.util.Map;
import java.util.Optional;

public class ElevSubsytem extends SubsystemBase {

    private final TalonFX motor;
    private Optional<ElevPosition> currentTarget;

    public ElevSubsytem() {
        motor = new TalonFX(ElevConstants.motorID);
        currentTarget = Optional.empty();

        configureMotor();
    }

    @Override
    public void periodic() {
        ElevPosition position = getPosition();

        Optional<ElevLevel> targetLvl =
                currentTarget.map((val) -> val.getAsElevLevel()).orElse(Optional.empty());

        String positionInchesString = Double.toString(position.getAsInches());
        String positionLvlString =
                position.getAsElevLevel().map((val) -> val.name()).orElse("none");
        String targetInchesString =
                currentTarget.map((val) -> Double.toString(val.getAsInches())).orElse("none");
        String targetLvlString = targetLvl.map((val) -> val.name()).orElse("none");

        Logger.log("elev", this);
        Logger.log(
                "elev",
                "motor",
                motor,
                Optional.of(Map.ofEntries(
                        Map.entry("positionInches", positionInchesString),
                        Map.entry("positionLevel", positionLvlString),
                        Map.entry("targetInches", targetInchesString),
                        Map.entry("targetLevel", targetLvlString))));

        Alerts.elevMotorDisconnected.set(!motor.isConnected());
        Alerts.elevMotorOverheating.set(motor.getDeviceTemp().getValue().abs(Units.Celsius) >= 80);
        Alerts.elevMotorFaults.set(TalonFXUtils.getAllActiveFaults(motor).hasCriticalFaults());
    }

    public void setTargetPos(ElevPosition target) {
        if (target == null) {
            Logger.reportWarning("Cannot set elev target position to a null target", true);
            return;
        }

        currentTarget = Optional.of(target);

        var req = new PositionVoltage(Angle.ofBaseUnits(target.getAsRotations(), Units.Rotations));
        req.Slot = 0;
        motor.setControl(req);
    }

    public void setDutyCycle(double dutyCycle) {
        currentTarget = Optional.empty();
        motor.set(Math.min(Math.max(dutyCycle, -ElevConstants.maxDutyCycle), ElevConstants.maxDutyCycle));
    }

    public void setVoltage(double volts) {
        currentTarget = Optional.empty();

        var req = new VoltageOut(Voltage.ofBaseUnits(volts, Units.Volts));
        motor.setControl(req);
    }

    public void stop() {
        currentTarget = Optional.empty();

        motor.set(0);
    }

    public ElevPosition getPosition() {
        return ElevPosition.fromRotations(motor.getPosition().getValueAsDouble());
    }

    public Optional<Boolean> isAtTarget() {
        if (currentTarget.isEmpty()) {
            return Optional.empty();
        }

        final double position = getPosition().getAsRotations();
        final double tolerance = ElevConstants.targetTolerance.getAsRotations();
        final double target = currentTarget.get().getAsRotations();

        return Optional.of(position >= target - tolerance && position <= target + tolerance);
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // current limits
        config.CurrentLimits.StatorCurrentLimit = ElevConstants.maxCurrent;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // hardstops
        config.HardwareLimitSwitch.ForwardLimitEnable = ElevConstants.forwardHardLimitEnabled;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable =
                ElevConstants.forwardHardLimitResetValue.isPresent();
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = ElevConstants.forwardHardLimitResetValue
                .orElse(ElevPosition.fromRotations(0))
                .getAsRotations();
        config.HardwareLimitSwitch.ReverseLimitEnable = ElevConstants.reverseHardLimitEnabled;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable =
                ElevConstants.reverseHardLimitResetValue.isPresent();
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = ElevConstants.reverseHardLimitResetValue
                .orElse(ElevPosition.fromRotations(0))
                .getAsRotations();

        // braking mode
        config.MotorOutput.NeutralMode = ElevConstants.neutralMode;
        // polarity
        config.MotorOutput.Inverted = ElevConstants.inverted;
        // duty cycle output limits
        config.MotorOutput.PeakForwardDutyCycle = ElevConstants.maxDutyCycle;
        config.MotorOutput.PeakReverseDutyCycle = -ElevConstants.maxDutyCycle;

        // pid
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        config.Slot0.kP = ElevConstants.kP;
        config.Slot0.kI = ElevConstants.kI;
        config.Slot0.kD = ElevConstants.kD;
        config.Slot0.kG = ElevConstants.kG;

        // softstops
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = ElevConstants.forwardSoftLimit.isPresent();
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevConstants.forwardSoftLimit
                .orElse(ElevPosition.fromRotations(0))
                .getAsRotations();
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = ElevConstants.reverseHardLimitEnabled;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevConstants.reverseSoftLimit
                .orElse(ElevPosition.fromRotations(0))
                .getAsRotations();

        // voltage output limits
        config.Voltage.PeakForwardVoltage = ElevConstants.maxVoltage;
        config.Voltage.PeakReverseVoltage = -ElevConstants.maxVoltage;

        if (motor.getConfigurator().apply(config) != StatusCode.OK) {
            Logger.reportError("Failed to configure elevator motor");
            Alerts.elevMotorConfigFail.set(true);
        }
    }
}
