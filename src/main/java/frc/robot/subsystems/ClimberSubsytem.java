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
import frc.robot.constants.ClimberConstants;
import frc.robot.units.ClimbClampAngle;
import frc.robot.units.ClimbLeverAngle;
import frc.robot.units.ClimbState;
import frc.robot.utils.Alerts;
import frc.robot.utils.Logger;
import frc.robot.utils.TalonFXUtils;
import java.util.Map;
import java.util.Optional;

public class ClimberSubsytem extends SubsystemBase {

    private final TalonFX lever, clamp;
    private Optional<ClimbLeverAngle> currentLeverTarget;
    private Optional<ClimbClampAngle> currentClampTarget;

    public ClimberSubsytem() {
        lever = new TalonFX(ClimberConstants.Lever.motorID);
        clamp = new TalonFX(ClimberConstants.Clamp.motorID);
        currentLeverTarget = Optional.empty();
        currentClampTarget = Optional.empty();

        configureMotor(lever, true);
        configureMotor(lever, false);
    }

    @Override
    public void periodic() {
        ClimbLeverAngle leverAngle = getLeverAngle();
        ClimbClampAngle clampAngle = getClampAngle();

        Optional<ClimbState> leverTargetState =
                currentLeverTarget.map((val) -> val.getAsState()).orElse(Optional.empty());
        Optional<ClimbState> clampTargetState =
                currentClampTarget.map((val) -> val.getAsState()).orElse(Optional.empty());

        String leverAngleMechRotsString = Double.toString(leverAngle.getAsLeverRotations());
        String clampAngleMechRotsString = Double.toString(clampAngle.getAsClampRotations());
        String leverAngleMechDegString = Double.toString(leverAngle.getAsLeverDegrees());
        String clampAngleMechDegString = Double.toString(clampAngle.getAsClampDegrees());
        String leverAngleStateString =
                leverAngle.getAsState().map((val) -> val.name()).orElse("none");
        String clampAngleStateString =
                clampAngle.getAsState().map((val) -> val.name()).orElse("none");
        String leverTargetMechRotsString = currentLeverTarget
                .map((val) -> Double.toString(val.getAsLeverRotations()))
                .orElse("none");
        String clampTargetMechRotsString = currentClampTarget
                .map((val) -> Double.toString(val.getAsClampRotations()))
                .orElse("none");
        String leverTargetMechDegString = currentLeverTarget
                .map((val) -> Double.toString(val.getAsLeverDegrees()))
                .orElse("none");
        String clampTargetMechDegString = currentClampTarget
                .map((val) -> Double.toString(val.getAsClampDegrees()))
                .orElse("none");
        String leverTargetStateString =
                leverTargetState.map((val) -> val.name()).orElse("none");
        String clampTargetStateString =
                clampTargetState.map((val) -> val.name()).orElse("none");

        Logger.log("climb", this);
        Logger.log(
                "climb",
                "lever",
                lever,
                Optional.of(Map.ofEntries(
                        Map.entry("angleMechRots", leverAngleMechRotsString),
                        Map.entry("angleMechDeg", leverAngleMechDegString),
                        Map.entry("angleState", leverAngleStateString),
                        Map.entry("targetMechRots", leverTargetMechRotsString),
                        Map.entry("targetMechDeg", leverTargetMechDegString),
                        Map.entry("targetState", leverTargetStateString))));
        Logger.log(
                "climb",
                "clamp",
                clamp,
                Optional.of(Map.ofEntries(
                        Map.entry("angleMechRots", clampAngleMechRotsString),
                        Map.entry("angleMechDeg", clampAngleMechDegString),
                        Map.entry("angleState", clampAngleStateString),
                        Map.entry("targetMechRots", clampTargetMechRotsString),
                        Map.entry("targetMechDeg", clampTargetMechDegString),
                        Map.entry("targetState", clampTargetStateString))));

        Alerts.climbLeverDisconnected.set(!lever.isConnected());
        Alerts.climbClampDisconnected.set(!clamp.isConnected());
        Alerts.climbLeverOverheating.set(lever.getDeviceTemp().getValue().abs(Units.Celsius) >= 80);
        Alerts.climbClampOverheating.set(clamp.getDeviceTemp().getValue().abs(Units.Celsius) >= 80);
        Alerts.climbLeverFaults.set(TalonFXUtils.getAllActiveFaults(lever).hasCriticalFaults());
        Alerts.climbClampFaults.set(TalonFXUtils.getAllStickyFaults(clamp).hasCriticalFaults());
    }

    public void setLeverTarget(ClimbLeverAngle target) {
        if (target == null) {
            Logger.reportWarning("Cannot set climber lever target angle to a null target", true);
            return;
        }

        currentLeverTarget = Optional.of(target);

        var req = new PositionVoltage(Angle.ofBaseUnits(target.getAsMotorRotations(), Units.Rotations));
        req.Slot = 0;
        lever.setControl(req);
    }

    public void setClampTarget(ClimbClampAngle target) {
        if (target == null) {
            Logger.reportWarning("Cannot set climber clamp target angle to a null target", true);
            return;
        }

        currentClampTarget = Optional.of(target);

        var req = new PositionVoltage(Angle.ofBaseUnits(target.getAsMotorRotations(), Units.Rotations));
        req.Slot = 0;
        clamp.setControl(req);
    }

    public void setLeverDutyCycle(double dutyCycle) {
        currentLeverTarget = Optional.empty();
        lever.set(Math.min(
                Math.max(dutyCycle, -ClimberConstants.Lever.maxDutyCycle), ClimberConstants.Lever.maxDutyCycle));
    }

    public void setClampDutyCycle(double dutyCycle) {
        currentClampTarget = Optional.empty();
        clamp.set(Math.min(
                Math.max(dutyCycle, -ClimberConstants.Clamp.maxDutyCycle), ClimberConstants.Clamp.maxDutyCycle));
    }

    public void setLeverVoltage(double volts) {
        currentLeverTarget = Optional.empty();

        var req = new VoltageOut(Voltage.ofBaseUnits(volts, Units.Volts));
        lever.setControl(req);
    }

    public void setClampVoltage(double volts) {
        currentClampTarget = Optional.empty();

        var req = new VoltageOut(Voltage.ofBaseUnits(volts, Units.Volts));
        clamp.setControl(req);
    }

    public void stopLever() {
        currentLeverTarget = Optional.empty();

        lever.set(0);
    }

    public void stopClamp() {
        currentClampTarget = Optional.empty();

        clamp.set(0);
    }

    public ClimbLeverAngle getLeverAngle() {
        return ClimbLeverAngle.fromMotorRotations(lever.getPosition().getValueAsDouble());
    }

    public ClimbClampAngle getClampAngle() {
        return ClimbClampAngle.fromMotorRotations(clamp.getPosition().getValueAsDouble());
    }

    public Optional<Boolean> isLeverAtTarget() {
        if (currentLeverTarget.isEmpty()) {
            return Optional.empty();
        }

        final double angle = getLeverAngle().getAsMotorRotations();
        final double tolerance = ClimberConstants.Lever.targetTolerance.getAsMotorRotations();
        final double target = currentLeverTarget.get().getAsMotorRotations();

        return Optional.of(angle >= target - tolerance && angle <= target + tolerance);
    }

    public Optional<Boolean> isClampAtTarget() {
        if (currentClampTarget.isEmpty()) {
            return Optional.empty();
        }

        final double angle = getClampAngle().getAsMotorRotations();
        final double tolerance = ClimberConstants.Clamp.targetTolerance.getAsMotorRotations();
        final double target = currentClampTarget.get().getAsMotorRotations();

        return Optional.of(angle >= target - tolerance && angle <= target + tolerance);
    }

    private void configureMotor(TalonFX motor, boolean isLever) {
        if (motor == null) {
            Logger.reportWarning("Cannot configure null climber motor", true);
            return;
        }

        TalonFXConfiguration config = new TalonFXConfiguration();

        if (isLever) {
            // current limits
            config.CurrentLimits.StatorCurrentLimit = ClimberConstants.Lever.maxCurrent;
            config.CurrentLimits.StatorCurrentLimitEnable = true;

            // hardstops
            config.HardwareLimitSwitch.ForwardLimitEnable = ClimberConstants.Lever.forwardHardLimitEnabled;
            config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable =
                    ClimberConstants.Lever.forwardHardLimitResetValue.isPresent();
            config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue =
                    ClimberConstants.Lever.forwardHardLimitResetValue
                            .orElse(ClimbLeverAngle.fromMotorRotations(0))
                            .getAsMotorRotations();
            config.HardwareLimitSwitch.ReverseLimitEnable = ClimberConstants.Lever.reverseHardLimitEnabled;
            config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable =
                    ClimberConstants.Lever.reverseHardLimitResetValue.isPresent();
            config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue =
                    ClimberConstants.Lever.reverseHardLimitResetValue
                            .orElse(ClimbLeverAngle.fromMotorRotations(0))
                            .getAsMotorRotations();

            // braking mode
            config.MotorOutput.NeutralMode = ClimberConstants.Lever.neutralMode;
            // polarity
            config.MotorOutput.Inverted = ClimberConstants.Lever.inverted;
            // duty cycle output limits
            config.MotorOutput.PeakForwardDutyCycle = ClimberConstants.Lever.maxDutyCycle;
            config.MotorOutput.PeakReverseDutyCycle = -ClimberConstants.Lever.maxDutyCycle;

            // pid
            config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
            config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
            config.Slot0.kP = ClimberConstants.Lever.kP;
            config.Slot0.kI = ClimberConstants.Lever.kI;
            config.Slot0.kD = ClimberConstants.Lever.kD;
            config.Slot0.kG = ClimberConstants.Lever.kG;

            // softstops
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = ClimberConstants.Lever.forwardSoftLimit.isPresent();
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.Lever.forwardSoftLimit
                    .orElse(ClimbLeverAngle.fromMotorRotations(0))
                    .getAsMotorRotations();
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = ClimberConstants.Lever.reverseHardLimitEnabled;
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.Lever.reverseSoftLimit
                    .orElse(ClimbLeverAngle.fromMotorRotations(0))
                    .getAsMotorRotations();

            // voltage output limits
            config.Voltage.PeakForwardVoltage = ClimberConstants.Lever.maxVoltage;
            config.Voltage.PeakReverseVoltage = -ClimberConstants.Lever.maxVoltage;
        } else {
            // current limits
            config.CurrentLimits.StatorCurrentLimit = ClimberConstants.Clamp.maxCurrent;
            config.CurrentLimits.StatorCurrentLimitEnable = true;

            // hardstops
            config.HardwareLimitSwitch.ForwardLimitEnable = ClimberConstants.Clamp.forwardHardLimitEnabled;
            config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable =
                    ClimberConstants.Clamp.forwardHardLimitResetValue.isPresent();
            config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue =
                    ClimberConstants.Clamp.forwardHardLimitResetValue
                            .orElse(ClimbClampAngle.fromMotorRotations(0))
                            .getAsMotorRotations();
            config.HardwareLimitSwitch.ReverseLimitEnable = ClimberConstants.Clamp.reverseHardLimitEnabled;
            config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable =
                    ClimberConstants.Clamp.reverseHardLimitResetValue.isPresent();
            config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue =
                    ClimberConstants.Clamp.reverseHardLimitResetValue
                            .orElse(ClimbClampAngle.fromMotorRotations(0))
                            .getAsMotorRotations();

            // braking mode
            config.MotorOutput.NeutralMode = ClimberConstants.Clamp.neutralMode;
            // polarity
            config.MotorOutput.Inverted = ClimberConstants.Clamp.inverted;
            // duty cycle output limits
            config.MotorOutput.PeakForwardDutyCycle = ClimberConstants.Clamp.maxDutyCycle;
            config.MotorOutput.PeakReverseDutyCycle = -ClimberConstants.Clamp.maxDutyCycle;

            // pid
            config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
            config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
            config.Slot0.kP = ClimberConstants.Clamp.kP;
            config.Slot0.kI = ClimberConstants.Clamp.kI;
            config.Slot0.kD = ClimberConstants.Clamp.kD;
            config.Slot0.kG = ClimberConstants.Clamp.kG;

            // softstops
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = ClimberConstants.Clamp.forwardSoftLimit.isPresent();
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.Clamp.forwardSoftLimit
                    .orElse(ClimbClampAngle.fromMotorRotations(0))
                    .getAsMotorRotations();
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = ClimberConstants.Clamp.reverseHardLimitEnabled;
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.Clamp.reverseSoftLimit
                    .orElse(ClimbClampAngle.fromMotorRotations(0))
                    .getAsMotorRotations();

            // voltage output limits
            config.Voltage.PeakForwardVoltage = ClimberConstants.Clamp.maxVoltage;
            config.Voltage.PeakReverseVoltage = -ClimberConstants.Clamp.maxVoltage;
        }

        if (motor.getConfigurator().apply(config) != StatusCode.OK) {
            Logger.reportError("Failed to configure climber " + (isLever ? "lever" : "clamp") + " motor");
            if (isLever) {
                Alerts.climbLeverConfigFail.set(true);
            } else {
                Alerts.climbClampConfigFail.set(true);
            }
        }
    }
}
