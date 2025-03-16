package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.math.Constants;
import frc.robot.math.Constants.ArmPreset;
import frc.robot.math.Constants.ElevatorPreset;

/**
 * The ElevatorArm subsystem manages a two-motor elevator and a single-motor arm
 * for game piece manipulation.
 * 
 * Hardware Configuration:
 * - Elevator: Two TalonFX motors (one leader, one follower)
 * - Arm: One TalonFX motor
 * 
 * The subsystem provides commands for:
 * - Manual control (joystick-based)
 * - Position control (preset positions)
 * - Sequence commands (coordinated movements)
 */
public class ElevatorArm extends SubsystemBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final TalonFX armMotor;
    private boolean isInitialized = false;

    /**
     * Creates a new ElevatorArm subsystem.
     * 
     * @param leftMotorID  The CAN ID of the left elevator motor (leader)
     * @param rightMotorID The CAN ID of the right elevator motor (follower)
     * @param armMotorID   The CAN ID of the arm motor
     */
    public ElevatorArm(int leftMotorID, int rightMotorID, int armMotorID) {
        this.leftMotor = new TalonFX(leftMotorID);
        this.rightMotor = new TalonFX(rightMotorID);
        this.armMotor = new TalonFX(armMotorID);
    }

    /**
     * Initializes the elevator and arm motors with their configurations.
     * 
     * This method should be called once during robot initialization.
     * If called multiple times, subsequent calls will be ignored with a warning.
     */
    public void init() {
        if (isInitialized) {
            System.err
                    .println("WARNING: ElevatorArm subsystem already initialized. Ignoring duplicate initialization.");
            return;
        }

        System.out.println("Initializing elevator and arm...");
        configureElevatorMotors();
        configureArmMotor();
        isInitialized = true;
        System.out.println("Elevator and arm initialized!");
    }

    private void configureElevatorMotors() {
        SoftwareLimitSwitchConfigs elevatorLimits = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(Constants.ELEVATOR_MAX_ROTATIONS)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Constants.ELEVATOR_MIN_ROTATIONS);

        MotorOutputConfigs leftMotorOutput = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive);

        CurrentLimitsConfigs leftCurrentLimits = new CurrentLimitsConfigs();

        TalonFXConfiguration leftConfig = new TalonFXConfiguration()
                .withMotorOutput(leftMotorOutput)
                .withSoftwareLimitSwitch(elevatorLimits)
                .withCurrentLimits(leftCurrentLimits);

        leftConfig.OpenLoopRamps = new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod(Constants.ELEVATOR_TIME_TO_MAX_SPEED);
        leftConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs()
                .withDutyCycleClosedLoopRampPeriod(Constants.ELEVATOR_TIME_TO_MAX_SPEED);

        leftConfig.Slot0.kP = Constants.ELEVATOR_PID_P;
        leftConfig.Slot0.kI = Constants.ELEVATOR_PID_I;
        leftConfig.Slot0.kD = Constants.ELEVATOR_PID_D;
        leftConfig.Slot0.kS = Constants.ELEVATOR_PID_S;
        leftConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        leftConfig.Slot0.kG = Constants.ELEVATOR_PID_G;

        leftConfig.MotorOutput.PeakForwardDutyCycle = Constants.ELEVATOR_MAX_SPEED;
        leftConfig.MotorOutput.PeakReverseDutyCycle = -Constants.ELEVATOR_MAX_SPEED;

        this.leftMotor.getConfigurator().apply(leftConfig);

        this.rightMotor.setControl(new Follower(this.leftMotor.getDeviceID(), false));

        this.leftMotor.setNeutralMode(NeutralModeValue.Brake);
        this.rightMotor.setNeutralMode(NeutralModeValue.Brake);

        this.leftMotor.setPosition(0);
    }

    private void configureArmMotor() {
        SoftwareLimitSwitchConfigs armLimits = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(Constants.ARM_MAX_ROTATIONS)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Constants.ARM_MIN_ROTATIONS);

        MotorOutputConfigs armOutput = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive);

        CurrentLimitsConfigs armCurrentLimits = new CurrentLimitsConfigs();

        FeedbackConfigs armFeedback = new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.ARM_GEAR_RATIO);

        TalonFXConfiguration armConfig = new TalonFXConfiguration()
                .withMotorOutput(armOutput)
                .withSoftwareLimitSwitch(armLimits)
                .withCurrentLimits(armCurrentLimits)
                .withFeedback(armFeedback);

        armConfig.Slot0.kP = Constants.ARM_PID_P;
        armConfig.Slot0.kI = Constants.ARM_PID_I;
        armConfig.Slot0.kD = Constants.ARM_PID_D;
        armConfig.Slot0.kS = Constants.ARM_PID_S;
        armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        armConfig.Slot0.kG = Constants.ARM_PID_G;

        armConfig.MotorOutput.PeakForwardDutyCycle = Constants.ARM_MAX_SPEED;
        armConfig.MotorOutput.PeakReverseDutyCycle = -Constants.ARM_MAX_SPEED;

        armConfig.OpenLoopRamps = new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod(Constants.ARM_TIME_TO_MAX_SPEED);
        armConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs()
                .withDutyCycleClosedLoopRampPeriod(Constants.ARM_TIME_TO_MAX_SPEED);

        this.armMotor.getConfigurator().apply(armConfig);

        this.armMotor.setPosition(ArmPreset.STOW.getPosition());
        this.armMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    // -------------------------------------------------------------------------
    // Basic Motor Control Methods, these should only be used internally by commands
    // -------------------------------------------------------------------------

    private void stopElevator() {
        this.leftMotor.stopMotor();
    }

    private void stopArm() {
        this.armMotor.stopMotor();
    }

    /**
     * Sets the raw elevator speed using duty cycle output
     * 
     * @param speed Speed from -1.0 to 1.0 (duty cycle percentage)
     */
    private void setRawElevatorSpeed(double speed) {
        this.leftMotor.setControl(new DutyCycleOut(speed));
    }

    /**
     * Sets the elevator position using closed-loop control
     * 
     * @param pos Position in rotations
     */
    private void setRawElevatorPos(double pos) {
        this.leftMotor.setControl(new PositionDutyCycle(pos));
    }

    /**
     * Sets the raw arm speed using duty cycle output
     * 
     * @param speed Speed from -1.0 to 1.0 (duty cycle percentage)
     */
    private void setRawArmSpeed(double speed) {
        this.armMotor.setControl(new DutyCycleOut(speed));
    }

    /**
     * Sets the arm position using closed-loop control
     * 
     * @param pos Position in rotations
     */
    private void setRawArmPos(double pos) {
        this.armMotor.setControl(new PositionDutyCycle(pos));
    }

    // -------------------------------------------------------------------------
    // Sendable and Dashboard Methods
    // -------------------------------------------------------------------------

    @Override
    public String getName() {
        return "ElevatorArm";
    }

    @Override
    public String getSubsystem() {
        return "Manipulator";
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("ElevatorArm");

        builder.addDoubleProperty(
                "Elevator Position (Rotations)",
                () -> this.leftMotor.getPosition().getValue().in(Rotations),
                null);
        builder.addDoubleProperty(
                "Elevator Velocity (RPS)",
                () -> this.leftMotor.getVelocity().getValueAsDouble(),
                null);
        builder.addDoubleProperty(
                "Elevator Voltage (V)",
                () -> this.leftMotor.getMotorVoltage().getValueAsDouble(),
                null);

        builder.addDoubleProperty(
                "Arm Position (Rotations)",
                () -> this.armMotor.getPosition().getValueAsDouble(),
                null);
        builder.addDoubleProperty(
                "Arm Velocity (RPS)",
                () -> this.armMotor.getVelocity().getValueAsDouble(),
                null);
        builder.addDoubleProperty(
                "Arm Voltage (V)",
                () -> this.armMotor.getMotorVoltage().getValueAsDouble(),
                null);
    }

    // -------------------------------------------------------------------------
    // Basic Control Commands
    // -------------------------------------------------------------------------

    public Command stopElevatorCommand() {
        return this.runOnce(this::stopElevator)
                .withName("StopElevator");
    }

    public Command stopArmCommand() {
        return this.runOnce(this::stopArm)
                .withName("StopArm");
    }

    // -------------------------------------------------------------------------
    // Manual Control Commands
    // -------------------------------------------------------------------------

    /**
     * @param speed Speed from -1.0 to 1.0 (duty cycle percentage)
     */
    private Command setElevatorManualSpeed(double speed) {
        return this.runOnce(() -> setRawElevatorSpeed(speed))
                .handleInterrupt(this::stopElevator)
                .withName("ElevatorManual(" + speed + ")");
    }

    /**
     * @param speed Speed from -1.0 to 1.0 (duty cycle percentage)
     */
    private Command setArmManualSpeed(double speed) {
        return this.runOnce(() -> setRawArmSpeed(speed))
                .handleInterrupt(this::stopArm)
                .withName("ArmManual(" + speed + ")");
    }

    public Command elevatorUp() {
        return setElevatorManualSpeed(Constants.ELEVATOR_MANUAL_SPEED)
                .withName("ElevatorUp");
    }

    public Command elevatorDown() {
        return setElevatorManualSpeed(Constants.ELEVATOR_MANUAL_DOWN_SPEED)
                .withName("ElevatorDown");
    }

    public Command armUp() {
        return setArmManualSpeed(Constants.ARM_MANUAL_UP_SPEED)
                .withName("ArmUp");
    }

    public Command armDown() {
        return setArmManualSpeed(Constants.ARM_MANUAL_DOWN_SPEED)
                .withName("ArmDown");
    }

    // -------------------------------------------------------------------------
    // Position Control Commands
    // -------------------------------------------------------------------------

    /**
     * Creates a command to move the elevator to a specific position
     */
    public Command setElevatorPosition(Constants.ElevatorPreset preset) {
        return this.runOnce(() -> setRawElevatorPos(preset.getPosition()))
                .handleInterrupt(this::stopElevator)
                .withName("ElevatorTo" + preset.name());
    }

    /**
     * Creates a command to move the arm to a specific position
     */
    public Command setArmPosition(Constants.ArmPreset preset) {
        return this.runOnce(() -> setRawArmPos(preset.getPosition()))
                .handleInterrupt(this::stopArm)
                .withName("ArmTo" + preset.name());
    }

    // -------------------------------------------------------------------------
    // Wait Condition Commands
    // -------------------------------------------------------------------------

    /**
     * Blocks until the arm is above a certain position
     * 
     * @param preset The preset position to exceed
     */
    public Command waitForArmAbove(ArmPreset preset) {
        return this.run(() -> {
        }) // Empty run so command continues to execute
                .until(() -> this.armMotor.getPosition().getValueAsDouble() >= preset.getPosition()
                        - Constants.ARM_TOLERANCE)
                .withName("WaitArmAbove" + preset.name());
    }

    /**
     * Blocks until the arm is below a certain position
     * 
     * @param preset The preset position to be below
     */
    public Command waitForArmBelow(ArmPreset preset) {
        return this.run(() -> {
        })
                .until(() -> this.armMotor.getPosition().getValueAsDouble() <= preset.getPosition()
                        + Constants.ARM_TOLERANCE)
                .withName("WaitArmBelow" + preset.name());
    }

    /**
     * Blocks until the elevator is above a certain position
     * 
     * @param preset The target position to exceed
     */
    public Command waitForElevatorAbove(ElevatorPreset preset) {
        return this.run(() -> {
        })
                .until(() -> this.leftMotor.getPosition().getValueAsDouble() >= preset.getPosition()
                        - Constants.ELEVATOR_TOLERANCE)
                .withName("WaitElevatorAbove" + preset.name());
    }

    /**
     * Blocks until the elevator is below a certain position
     * 
     * @param preset The target position to be below
     */
    public Command waitForElevatorBelow(ElevatorPreset preset) {
        return this.run(() -> {
        })
                .until(() -> this.leftMotor.getPosition().getValueAsDouble() <= preset.getPosition()
                        + Constants.ELEVATOR_TOLERANCE)
                .withName("WaitElevatorBelow" + preset.name());
    }

    // -------------------------------------------------------------------------
    // Complex Sequence Commands
    // -------------------------------------------------------------------------
    /*
     * Sequence to safely pick up game pieces
     * 
     * 1. Arm out until clear of upper clearance
     * 2. Elevator up to transition position
     * 3. Arm down to lower clearance
     * 4. Elevator down to pickup position
     * 5. Elevator up to pickup position
     * 6. Arm out to score position
     * 7. Elevator and arm stow
     */
    public Command pickupSequence() {
        return Commands.sequence(
                setArmPosition(ArmPreset.OUT),
                waitForArmBelow(ArmPreset.UPPER_CLEARANCE),
                setElevatorPosition(ElevatorPreset.TRANSITION),
                waitForElevatorAbove(ElevatorPreset.TRANSITION),
                setArmPosition(ArmPreset.DOWN),
                waitForArmBelow(ArmPreset.DOWN),
                setElevatorPosition(ElevatorPreset.PICKUP),
                waitForElevatorBelow(ElevatorPreset.PICKUP),
                setElevatorPosition(ElevatorPreset.TRANSITION),
                waitForElevatorAbove(ElevatorPreset.TRANSITION),
                setArmPosition(ArmPreset.OUT),
                waitForArmAbove(ArmPreset.LOWER_CLEARANCE),
                setElevatorPosition(ElevatorPreset.STOW),
                setArmPosition(ArmPreset.STOW)).withName("PickupSequence");
    }

    // Coordinates safe movement: position arm first, then move elevator when arm is
    // clear
    private Command scoringSequence(ElevatorPreset elevatorPreset, ArmPreset armPreset) {
        return Commands.sequence(
                setArmPosition(armPreset),
                waitForArmBelow(armPreset),
                setElevatorPosition(elevatorPreset)).withName(elevatorPreset.name() + "Sequence");
    }

    public Command L1Sequence() {
        return scoringSequence(ElevatorPreset.L1, ArmPreset.SCORE_LOW);
    }

    public Command L2Sequence() {
        return scoringSequence(ElevatorPreset.L2, ArmPreset.SCORE_LOW);
    }

    public Command L3Sequence() {
        return scoringSequence(ElevatorPreset.L3, ArmPreset.SCORE_LOW);
    }

    // Uses high arm position due to L4's higher elevation
    public Command L4Sequence() {
        return scoringSequence(ElevatorPreset.L4, ArmPreset.SCORE_HIGH);
    }

    // Safety sequence: lower elevator completely before moving arm to avoid
    // collisions
    public Command stowSequence() {
        return Commands.sequence(
                setElevatorPosition(ElevatorPreset.STOW),
                waitForElevatorBelow(ElevatorPreset.STOW),
                setArmPosition(ArmPreset.STOW)).withName("StowSequence");
    }
}
