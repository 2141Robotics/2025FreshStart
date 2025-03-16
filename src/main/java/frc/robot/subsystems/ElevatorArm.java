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
    // Motor controllers for the elevator and arm
    private final TalonFX leftMotor;  // Elevator leader motor
    private final TalonFX rightMotor; // Elevator follower motor
    private final TalonFX armMotor;   // Arm motor
    
    // Track current command name for dashboard
    private String currentCommandName = "None";
    
    /**
     * Creates a new ElevatorArm subsystem.
     * 
     * @param leftMotorID The CAN ID of the left elevator motor (leader)
     * @param rightMotorID The CAN ID of the right elevator motor (follower)
     * @param armMotorID The CAN ID of the arm motor
     */
    public ElevatorArm(int leftMotorID, int rightMotorID, int armMotorID) {
        this.leftMotor = new TalonFX(leftMotorID);
        this.rightMotor = new TalonFX(rightMotorID);
        this.armMotor = new TalonFX(armMotorID);
    }
    
    /**
     * Initializes the elevator and arm motors with their configurations.
     * This method should be called once during robot initialization.
     */
    public void init() {
        System.out.println("Initializing elevator and arm!");
        
        // Configure and initialize the elevator motors
        configureElevatorMotors();
        
        // Configure and initialize the arm motor
        configureArmMotor();
    }
    
    /**
     * Configures the elevator motors with proper settings
     */
    private void configureElevatorMotors() {
        // Configure software limits for safety
        SoftwareLimitSwitchConfigs elevatorLimits = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(Constants.ELEVATOR_MAX_ROTATIONS)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(Constants.ELEVATOR_MIN_ROTATIONS);

        // Configure the left (leader) motor's output settings
        MotorOutputConfigs leftMotorOutput = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive);

        // Create current limit configuration (using defaults)
        CurrentLimitsConfigs leftCurrentLimits = new CurrentLimitsConfigs();

        // Build the complete configuration
        TalonFXConfiguration leftConfig = new TalonFXConfiguration()
            .withMotorOutput(leftMotorOutput)
            .withSoftwareLimitSwitch(elevatorLimits)
            .withCurrentLimits(leftCurrentLimits);

        // Configure acceleration ramps for smoother motion
        leftConfig.OpenLoopRamps = new OpenLoopRampsConfigs()
            .withDutyCycleOpenLoopRampPeriod(Constants.ELEVATOR_TIME_TO_MAX_SPEED);
        leftConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs()
            .withDutyCycleClosedLoopRampPeriod(Constants.ELEVATOR_TIME_TO_MAX_SPEED);

        // Configure PID and motion control parameters
        leftConfig.Slot0.kP = Constants.ELEVATOR_PID_P;
        leftConfig.Slot0.kI = Constants.ELEVATOR_PID_I;
        leftConfig.Slot0.kD = Constants.ELEVATOR_PID_D;
        leftConfig.Slot0.kS = Constants.ELEVATOR_PID_S;
        leftConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        leftConfig.Slot0.kG = Constants.ELEVATOR_PID_G;

        // Set speed limits
        leftConfig.MotorOutput.PeakForwardDutyCycle = Constants.ELEVATOR_MAX_SPEED;
        leftConfig.MotorOutput.PeakReverseDutyCycle = -Constants.ELEVATOR_MAX_SPEED;

        // Apply configuration to left motor
        this.leftMotor.getConfigurator().apply(leftConfig);
        
        // Configure right motor to follow left motor
        this.rightMotor.setControl(new Follower(this.leftMotor.getDeviceID(), false));
        
        // Set both motors to brake mode for better position holding
        this.leftMotor.setNeutralMode(NeutralModeValue.Brake);
        this.rightMotor.setNeutralMode(NeutralModeValue.Brake);

        // Zero the elevator position
        this.leftMotor.setPosition(0);
    }
    
    /**
     * Configures the arm motor with proper settings
     */
    private void configureArmMotor() {
        // Configure software limits for safety
        SoftwareLimitSwitchConfigs armLimits = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(Constants.ARM_MAX_ROTATIONS)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(Constants.ARM_MIN_ROTATIONS);

        // Configure motor output settings
        MotorOutputConfigs armOutput = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive);

        // Create current limit configuration (using defaults)
        CurrentLimitsConfigs armCurrentLimits = new CurrentLimitsConfigs();

        // Configure feedback settings (gear ratio)
        FeedbackConfigs armFeedback = new FeedbackConfigs()
            .withSensorToMechanismRatio(Constants.ARM_GEAR_RATIO);

        // Build the complete configuration
        TalonFXConfiguration armConfig = new TalonFXConfiguration()
            .withMotorOutput(armOutput)
            .withSoftwareLimitSwitch(armLimits)
            .withCurrentLimits(armCurrentLimits)
            .withFeedback(armFeedback);

        // Configure PID and motion control parameters
        armConfig.Slot0.kP = Constants.ARM_PID_P;
        armConfig.Slot0.kI = Constants.ARM_PID_I;
        armConfig.Slot0.kD = Constants.ARM_PID_D;
        armConfig.Slot0.kS = Constants.ARM_PID_S;
        armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        armConfig.Slot0.kG = Constants.ARM_PID_G;

        // Set speed limits
        armConfig.MotorOutput.PeakForwardDutyCycle = Constants.ARM_MAX_SPEED;
        armConfig.MotorOutput.PeakReverseDutyCycle = -Constants.ARM_MAX_SPEED;

        // Configure acceleration ramps for smoother motion
        armConfig.OpenLoopRamps = new OpenLoopRampsConfigs()
            .withDutyCycleOpenLoopRampPeriod(Constants.ARM_TIME_TO_MAX_SPEED);
        armConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs()
            .withDutyCycleClosedLoopRampPeriod(Constants.ARM_TIME_TO_MAX_SPEED);

        // Apply configuration to arm motor
        this.armMotor.getConfigurator().apply(armConfig);
        
        // Initialize arm position to stowed position
        this.armMotor.setPosition(Constants.ARM_STOW);
        this.armMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    
    //-------------------------------------------------------------------------
    // Basic Motor Control Methods
    //-------------------------------------------------------------------------
    
    /**
     * Stops the elevator motors
     */
    public void stopElevator() {
        this.leftMotor.stopMotor();
    }

    /**
     * Stops the arm motor
     */
    public void stopArm() {
        this.armMotor.stopMotor();
    }

    /**
     * Sets the raw speed of the elevator motors
     * @param speed Speed from -1.0 to 1.0
     */
    public void setRawElevatorSpeed(double speed) {
        this.leftMotor.setControl(new DutyCycleOut(speed));
    }

    /**
     * Sets the elevator position using closed-loop control
     * @param pos Position in rotations
     */
    public void setRawElevatorPos(double pos) {
        this.leftMotor.setControl(new PositionDutyCycle(pos));
    }

    /**
     * Sets the raw speed of the arm motor
     * @param speed Speed from -1.0 to 1.0
     */
    public void setRawArmSpeed(double speed) {
        this.armMotor.setControl(new DutyCycleOut(speed));
    }

    /**
     * Sets the arm position using closed-loop control
     * @param pos Position in rotations
     */
    public void setRawArmPos(double pos) {
        this.armMotor.setControl(new PositionDutyCycle(pos));
    }

    //-------------------------------------------------------------------------
    // Sendable and Dashboard Methods
    //-------------------------------------------------------------------------
    
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
        
        // Elevator information
        builder.addDoubleProperty(
            "Elevator Position (Rotations)", 
            () -> this.leftMotor.getPosition().getValue().in(Rotations), 
            null);
        builder.addDoubleProperty(
            "Elevator Velocity", 
            () -> this.leftMotor.getVelocity().getValueAsDouble(), 
            null);
        builder.addDoubleProperty(
            "Elevator Voltage", 
            () -> this.leftMotor.getMotorVoltage().getValueAsDouble(), 
            null);
            
        // Arm information
        builder.addDoubleProperty(
            "Arm Position", 
            () -> this.armMotor.getPosition().getValueAsDouble(), 
            null);
        builder.addDoubleProperty(
            "Arm Velocity", 
            () -> this.armMotor.getVelocity().getValueAsDouble(), 
            null);
        builder.addDoubleProperty(
            "Arm Voltage", 
            () -> this.armMotor.getMotorVoltage().getValueAsDouble(), 
            null);
        
        // Command tracking
        builder.addStringProperty(
            "Current Command",
            () -> this.currentCommandName,
            null);
        builder.addBooleanProperty(
            "Has Command",
            () -> this.getCurrentCommand() != null,
            null);
    }

    @Override
    public void periodic() {
        super.periodic();
        // Telemetry is handled by the initSendable method
    }

    //-------------------------------------------------------------------------
    // Basic Control Commands
    //-------------------------------------------------------------------------
    
    /**
     * Creates a command to stop the elevator motors
     */
    public Command stopElevatorCommand() {
        return this.runOnce(this::stopElevator)
            .withName("StopElevator");
    }

    /**
     * Creates a command to stop the arm motor
     */
    public Command stopArmCommand() {
        return this.runOnce(this::stopArm)
            .withName("StopArm");
    }

    //-------------------------------------------------------------------------
    // Manual Control Commands
    //-------------------------------------------------------------------------
    
    /**
     * Creates a command to set the elevator speed manually
     * @param speed Speed from -1.0 to 1.0
     */
    public Command setElevatorManualSpeed(double speed) {
        return this.runOnce(() -> setRawElevatorSpeed(speed))
            .handleInterrupt(this::stopElevator)
            .withName("ElevatorManual(" + speed + ")");
    }

    /**
     * Creates a command to set the arm speed manually
     * @param speed Speed from -1.0 to 1.0
     */
    public Command setArmManualSpeed(double speed) {
        return this.runOnce(() -> setRawArmSpeed(speed))
            .handleInterrupt(this::stopArm)
            .withName("ArmManual(" + speed + ")");
    }

    /**
     * Convenience command to move the elevator up at a preset speed
     */
    public Command elevatorUp() {
        return setElevatorManualSpeed(Constants.ELEVATOR_MANUAL_SPEED)
            .withName("ElevatorUp");
    }

    /**
     * Convenience command to move the elevator down at a preset speed
     */
    public Command elevatorDown() {
        return setElevatorManualSpeed(Constants.ELEVATOR_MANUAL_DOWN_SPEED)
            .withName("ElevatorDown");
    }

    /**
     * Convenience command to move the arm up at a preset speed
     */
    public Command armUp() {
        return setArmManualSpeed(Constants.ARM_MANUAL_UP_SPEED)
            .withName("ArmUp");
    }
    
    /**
     * Convenience command to move the arm down at a preset speed
     */
    public Command armDown() {
        return setArmManualSpeed(Constants.ARM_MANUAL_DOWN_SPEED)
            .withName("ArmDown");
    }

    //-------------------------------------------------------------------------
    // Position Control Commands
    //-------------------------------------------------------------------------
    
    /**
     * Creates a command to move the elevator to a specific position
     * @param position Target position in rotations
     * @param positionName Name of the position for display purposes
     */
    public Command setElevatorPosition(double position, String positionName) {
        return this.runOnce(() -> setRawElevatorPos(position))
            .handleInterrupt(this::stopElevator)
            .withName("ElevatorTo" + positionName);
    }

    /**
     * Creates a command to move the arm to a specific position
     * @param position Target position in rotations
     * @param positionName Name of the position for display purposes
     */
    public Command setArmPosition(double position, String positionName) {
        return this.runOnce(() -> setRawArmPos(position))
            .handleInterrupt(this::stopArm)
            .withName("ArmTo" + positionName);
    }

    //-------------------------------------------------------------------------
    // Preset Position Commands - Elevator
    //-------------------------------------------------------------------------
    
    /**
     * Creates a command to move the elevator to the L1 position
     */
    public Command setElevatorPositionL1() {
        return setElevatorPosition(Constants.ELEVATOR_L1, "L1");
    }

    /**
     * Creates a command to move the elevator to the L2 position
     */
    public Command setElevatorPositionL2() {
        return setElevatorPosition(Constants.ELEVATOR_L2, "L2");
    }

    /**
     * Creates a command to move the elevator to the L3 position
     */
    public Command setElevatorPositionL3() {
        return setElevatorPosition(Constants.ELEVATOR_L3, "L3");
    }

    /**
     * Creates a command to move the elevator to the L4 position
     */
    public Command setElevatorPositionL4() {
        return setElevatorPosition(Constants.ELEVATOR_L4, "L4");
    }

    /**
     * Creates a command to move the elevator to the stowed position
     */
    public Command setElevatorPositionStow() {
        return setElevatorPosition(Constants.ELEVATOR_STOW, "Stow");
    }

    /**
     * Creates a command to move the elevator to the transition position
     */
    public Command setElevatorPositionTransition() {
        return setElevatorPosition(Constants.ELEVATOR_TRANSITION, "Transition");
    }

    /**
     * Creates a command to move the elevator to the pickup position
     */
    public Command setElevatorPositionPickup() {
        return setElevatorPosition(Constants.ELEVATOR_PICKUP, "Pickup");
    }

    //-------------------------------------------------------------------------
    // Preset Position Commands - Arm
    //-------------------------------------------------------------------------
    
    /**
     * Creates a command to move the arm to the stowed position
     */
    public Command setArmPositionStow() {
        return setArmPosition(Constants.ARM_STOW, "Stow");
    }

    /**
     * Creates a command to move the arm to the outward position
     */
    public Command setArmPositionOUT() {
        return setArmPosition(Constants.ARM_OUT, "Out");
    }

    /**
     * Creates a command to move the arm to the low scoring position
     */
    public Command setArmPositionSCORE_LOW() {
        return setArmPosition(Constants.ARM_SCORE_LOW, "ScoreLow");
    }

    /**
     * Creates a command to move the arm to the high scoring position
     */
    public Command setArmPositionSCORE_HIGH() {
        return setArmPosition(Constants.ARM_SCORE_HIGH, "ScoreHigh");
    }

    /**
     * Creates a command to move the arm to the down position
     */
    public Command setArmPositionDown() {
        return setArmPosition(Constants.ARM_DOWN, "Down");
    }

    //-------------------------------------------------------------------------
    // Wait Condition Commands
    //-------------------------------------------------------------------------
    
    /**
     * Creates a command that waits for the arm to be above a certain angle
     * @param angle The target angle to exceed
     */
    public Command waitForArmAbove(double angle) {
        return this.run(() -> {}) // Empty run so command continues to execute
            .until(() -> this.armMotor.getPosition().getValueAsDouble() >= angle - Constants.ARM_TOLERANCE)
            .withName("WaitArmAbove(" + angle + ")");
    }

    /**
     * Creates a command that waits for the arm to be below a certain angle
     * @param angle The target angle to be below
     */
    public Command waitForArmBelow(double angle) {
        return this.run(() -> {})
            .until(() -> this.armMotor.getPosition().getValueAsDouble() <= angle + Constants.ARM_TOLERANCE)
            .withName("WaitArmBelow(" + angle + ")");
    }

    /**
     * Creates a command that waits for the elevator to be above a certain position
     * @param position The target position to exceed
     */
    public Command waitForElevatorAbove(double position) {
        return this.run(() -> {})
            .until(() -> this.leftMotor.getPosition().getValueAsDouble() >= position - Constants.ELEVATOR_TOLERANCE)
            .withName("WaitElevatorAbove(" + position + ")");
    }

    /**
     * Creates a command that waits for the elevator to be below a certain position
     * @param position The target position to be below
     */
    public Command waitForElevatorBelow(double position) {
        return this.run(() -> {})
            .until(() -> this.leftMotor.getPosition().getValueAsDouble() <= position + Constants.ELEVATOR_TOLERANCE)
            .withName("WaitElevatorBelow(" + position + ")");
    }

    /**
     * Creates a command that waits for the elevator to reach a target position
     * within the defined tolerance
     * @param position The target position
     */
    public Command waitForElevatorAtPosition(double position) {
        return this.run(() -> {})
            .until(() -> Math.abs(this.leftMotor.getPosition().getValueAsDouble() - position) <= Constants.ELEVATOR_TOLERANCE)
            .withName("WaitElevatorAt(" + position + ")");
    }
    
    /**
     * Creates a command that waits for the arm to reach a target position
     * within the defined tolerance
     * @param position The target position
     */
    public Command waitForArmAtPosition(double position) {
        return this.run(() -> {})
            .until(() -> Math.abs(this.armMotor.getPosition().getValueAsDouble() - position) <= Constants.ARM_TOLERANCE)
            .withName("WaitArmAt(" + position + ")");
    }

    //-------------------------------------------------------------------------
    // Combined Movement Commands
    //-------------------------------------------------------------------------
    
    /**
     * Creates a command that moves the elevator to a position and waits for completion
     * @param position The target position
     * @param positionName Name of the position for display purposes
     */
    public Command moveElevatorToPositionAndWait(double position, String positionName) {
        return Commands.sequence(
            setElevatorPosition(position, positionName),
            waitForElevatorAtPosition(position)
        ).withName("ElevatorToAndWait" + positionName);
    }
    
    /**
     * Creates a command that moves the arm to a position and waits for completion
     * @param position The target position
     * @param positionName Name of the position for display purposes
     */
    public Command moveArmToPositionAndWait(double position, String positionName) {
        return Commands.sequence(
            setArmPosition(position, positionName),
            waitForArmAtPosition(position)
        ).withName("ArmToAndWait" + positionName);
    }
    
    //-------------------------------------------------------------------------
    // Complex Sequence Commands
    //-------------------------------------------------------------------------
    
    /**
     * Pickup sequence command
     * Coordinates elevator and arm movements to pick up game pieces safely
     * 
     * The sequence:
     * 1. Move arm out
     * 2. Wait for arm to clear elevator
     * 3. Move elevator to transition position
     * 4. Wait for elevator to reach transition
     * 5. Move arm down for pickup
     * 6. Wait for arm to reach down position
     * 7. Move elevator to pickup position
     * 8. Wait for elevator to reach pickup
     * 9. Move elevator back to transition
     * 10. Wait for elevator to reach transition
     * 11. Move arm out
     * 12. Wait for arm to clear elevator
     * 13. Move elevator to stowed position
     * 14. Move arm to stowed position
     */
    public Command pickupSequence() {
        return Commands.sequence(
            setArmPositionOUT(),
            waitForArmBelow(Constants.ARM_UPPER_ELEVATOR_CLEARANCE),
            setElevatorPositionTransition(),
            waitForElevatorAbove(Constants.ELEVATOR_TRANSITION),
            setArmPositionDown(),
            waitForArmBelow(Constants.ARM_DOWN),
            setElevatorPositionPickup(),
            waitForElevatorBelow(Constants.ELEVATOR_PICKUP),
            setElevatorPositionTransition(),
            waitForElevatorAbove(Constants.ELEVATOR_TRANSITION),
            setArmPositionOUT(),
            waitForArmAbove(Constants.ARM_LOWER_ELEVATOR_CLEARANCE),
            setElevatorPositionStow(),
            setArmPositionStow()
        ).withName("PickupSequence");
    }

    /**
     * Level 1 scoring sequence
     * Moves the arm and elevator to score on the L1 position
     */
    public Command L1Sequence() {
        return Commands.sequence(
            setArmPositionSCORE_LOW(),
            waitForArmBelow(Constants.ARM_UPPER_ELEVATOR_CLEARANCE),
            setElevatorPositionL1()
        ).withName("L1Sequence");
    }

    /**
     * Level 2 scoring sequence
     * Moves the arm and elevator to score on the L2 position
     */
    public Command L2Sequence() {
        return Commands.sequence(
            setArmPositionSCORE_LOW(),
            waitForArmBelow(Constants.ARM_UPPER_ELEVATOR_CLEARANCE),
            setElevatorPositionL2()
        ).withName("L2Sequence");
    }
    
    /**
     * Level 3 scoring sequence
     * Moves the arm and elevator to score on the L3 position
     */
    public Command L3Sequence() {
        return Commands.sequence(
            setArmPositionSCORE_LOW(),
            waitForArmBelow(Constants.ARM_UPPER_ELEVATOR_CLEARANCE),
            setElevatorPositionL3()
        ).withName("L3Sequence");
    }
    
    /**
     * Level 4 scoring sequence
     * Moves the arm and elevator to score on the L4 position
     */
    public Command L4Sequence() {
        return Commands.sequence(
            setArmPositionSCORE_HIGH(),
            waitForArmBelow(Constants.ARM_UPPER_ELEVATOR_CLEARANCE),
            setElevatorPositionL4()
        ).withName("L4Sequence");
    }
    
    /**
     * Stow sequence command
     * Moves the elevator and arm to their stowed positions safely
     */
    public Command stowSequence() {
        return Commands.sequence(
            setElevatorPositionStow(),
            waitForElevatorAtPosition(Constants.ELEVATOR_STOW),
            setArmPositionStow()
        ).withName("StowSequence");
    }
}
