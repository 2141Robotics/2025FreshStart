package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
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
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.mineinjava.quail.util.geometry.Vec2d;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.math.Constants;

// 1. Initialize talons
// 2. Configure talons
// 3. Up and down commands

public class ElevatorArm extends SubsystemBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final TalonFX armMotor;

public void init(){
    System.out.println("Initializing elevator!");

    SoftwareLimitSwitchConfigs elevatorMotorSoftwareLimitSwitchConfig = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(Constants.ELEVATOR_MAX_ROTATIONS)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(Constants.ELEVATOR_MIN_ROTATIONS);

    MotorOutputConfigs leftMotorConfig = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive);

    CurrentLimitsConfigs leftTaloncCurrentLimitsConfigs = new CurrentLimitsConfigs();

    TalonFXConfiguration leftTalonConfig = new TalonFXConfiguration()
        .withMotorOutput(leftMotorConfig)
        .withSoftwareLimitSwitch(elevatorMotorSoftwareLimitSwitchConfig)
        .withCurrentLimits(leftTaloncCurrentLimitsConfigs);

    leftTalonConfig.OpenLoopRamps = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(Constants.ELEVATOR_TIME_TO_MAX_SPEED);
    leftTalonConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(Constants.ELEVATOR_TIME_TO_MAX_SPEED);


    leftTalonConfig.Slot0.kP = 0.2;
    leftTalonConfig.Slot0.kI = 0.0;
    leftTalonConfig.Slot0.kD = 0.01;
    leftTalonConfig.Slot0.kS = 0.0;
    leftTalonConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    leftTalonConfig.Slot0.kG = 0.02;

    leftTalonConfig.MotorOutput.PeakForwardDutyCycle = Constants.ELEVATOR_MAX_SPEED;
    leftTalonConfig.MotorOutput.PeakReverseDutyCycle = -Constants.ELEVATOR_MAX_SPEED;

    this.leftMotor.getConfigurator().apply(leftTalonConfig);
    this.rightMotor.setControl(new Follower(this.leftMotor.getDeviceID(), false));
    this.leftMotor.setNeutralMode(NeutralModeValue.Brake);
    this.rightMotor.setNeutralMode(NeutralModeValue.Brake);


    // TODO: Right follow left

    // 0 the positions
    this.leftMotor.setPosition(0);


    SoftwareLimitSwitchConfigs armMotorSoftwareLimitSwitchConfig = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(Constants.ARM_MAX_ROTATIONS)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(Constants.ARM_MIN_ROTATIONS);

    MotorOutputConfigs armMotorOutputConfigs = new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive);

    CurrentLimitsConfigs armCurrentLimitsConfigs = new CurrentLimitsConfigs();

    FeedbackConfigs armFeedbackConfigs = new FeedbackConfigs()
        .withSensorToMechanismRatio(Constants.ARM_GEAR_RATIO);

    TalonFXConfiguration armMotorConfig = new TalonFXConfiguration()
        .withMotorOutput(armMotorOutputConfigs)
        .withSoftwareLimitSwitch(armMotorSoftwareLimitSwitchConfig)
        .withCurrentLimits(armCurrentLimitsConfigs)
        .withFeedback(armFeedbackConfigs);

    armMotorConfig.Slot0.kP = 3.2;
    armMotorConfig.Slot0.kI = 0.05;
    armMotorConfig.Slot0.kD = 0.20;
    armMotorConfig.Slot0.kS = 0.0;
    armMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    armMotorConfig.Slot0.kG = 0.02;

    armMotorConfig.MotorOutput.PeakForwardDutyCycle = Constants.ARM_MAX_SPEED;
    armMotorConfig.MotorOutput.PeakReverseDutyCycle = -Constants.ARM_MAX_SPEED;

    armMotorConfig.OpenLoopRamps = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(Constants.ARM_TIME_TO_MAX_SPEED);
    armMotorConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(Constants.ARM_TIME_TO_MAX_SPEED);


    this.armMotor.getConfigurator().apply(armMotorConfig);
    this.armMotor.setPosition(Constants.ARM_STOW);
    this.armMotor.setNeutralMode(NeutralModeValue.Brake);
}

    public ElevatorArm(int leftMotorID, int rightMotorID, int armMotorID) {
        this.leftMotor = new TalonFX(leftMotorID);
        this.rightMotor = new TalonFX(rightMotorID);
        this.armMotor = new TalonFX(armMotorID);
    }

    public void stopElevator() {
        this.leftMotor.stopMotor();
    }

    public void stopArm(){
        this.armMotor.stopMotor();
    }

    public void setRawElevatorSpeed(double speed) {
        this.leftMotor.setControl(new DutyCycleOut(speed));
    }

    public void setRawElevatorPos(double pos) {
        this.leftMotor.setControl(new PositionDutyCycle(pos));
    }

    public void setRawArmSpeed(double speed){
        this.armMotor.setControl(new DutyCycleOut(speed));
    }

    public void setRawArmPos(double pos) {
        this.armMotor.setControl(new PositionDutyCycle(pos));
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return super.getName();
    }

    @Override
    public String getSubsystem() {
        // TODO Auto-generated method stub
        return super.getSubsystem();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        // To the dashboard, push current position, speed
        //builder.addBooleanProperty("extended", () -> m_hatchSolenoid.get() == this.speed, null);
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        Angle elevatorPositions = this.leftMotor.getPosition().getValue();
        SmartDashboard.putNumber("Elevator Position", elevatorPositions.in(Rotations));
        SmartDashboard.putNumber("Arm Position: ", armMotor.getPosition().getValueAsDouble());

    }

    public Command stopElevatorCommand(){
        //TODO:REMOVE ME PLZ
        return this.runOnce(() -> {
            this.stopElevator();
        });
    }

    public Command stopArmCommand(){
        return this.runOnce(() -> {
            this.stopArm();
        });
    }

    public Command elevatorUp(){
        return this.runOnce(() -> {
            this.setRawElevatorSpeed(Constants.ELEVATOR_MANUAL_SPEED);
        }).handleInterrupt(() -> {
            this.stopElevator();
        });

    }

    public Command elevatorDown(){
        return this.runOnce(() -> {
            this.setRawElevatorSpeed(-0.1d);;
        }).handleInterrupt(() -> {
            this.stopElevator();
        });
    }

    public Command armUp(){
        return this.runOnce(() -> {
            this.setRawArmSpeed(0.1d);;
        }).handleInterrupt(() -> {
            this.stopArm();
        });
    }
    public Command armDown(){
        return this.runOnce(() -> {
            this.setRawArmSpeed(-0.1d);;
        }).handleInterrupt(() -> {
            this.stopArm();
        });
    }


    public Command setElevatorPositionL1(){
        return this.runOnce(() -> {
            this.setRawElevatorPos(Constants.ELEVATOR_L1);;
        }).handleInterrupt(() -> {
            this.stopElevator();
        });
    }

    public Command setElevatorPositionL2(){
        return this.runOnce(() -> {
            this.setRawElevatorPos(Constants.ELEVATOR_L2);;
        }).handleInterrupt(() -> {
            this.stopElevator();
        });
    }

    public Command setElevatorPositionL3(){
        return this.runOnce(() -> {
            this.setRawElevatorPos(Constants.ELEVATOR_L3);;
        }).handleInterrupt(() -> {
            this.stopElevator();
        });
    }

    public Command setElevatorPositionL4(){
        return this.runOnce(() -> {
            this.setRawElevatorPos(Constants.ELEVATOR_L4);;
        }).handleInterrupt(() -> {
            this.stopElevator();
        });
    }

    public Command setElevatorPositionStow(){
        return this.runOnce(() -> {
            this.setRawElevatorPos(Constants.ELEVATOR_STOW);;
        }).handleInterrupt(() -> {
            this.stopElevator();
        });
    }

    public Command setElevatorPositionTransition(){
        return this.runOnce(() -> {
            this.setRawElevatorPos(Constants.ELEVATOR_TRANSITION);
        }).handleInterrupt(() -> {
            this.stopElevator();
        });
    }

    public Command setElevatorPositionPickup(){
        return this.runOnce(() -> {
            this.setRawElevatorPos(Constants.ELEVATOR_PICKUP);
        }).handleInterrupt(() -> {
            this.stopElevator();
        });
    }

    public Command setArmPositionStow(){
        return this.runOnce(() -> {
            this.setRawArmPos(Constants.ARM_STOW);
        }).handleInterrupt(() -> {
            this.stopArm();
        });
    }

    public Command setArmPositionOUT(){
        return this.runOnce(() -> {
            this.setRawArmPos(Constants.ARM_OUT);
        }).handleInterrupt(() -> {
            this.stopArm();
        });
    }

    public Command setArmPositionSCORE_LOW(){
        return this.runOnce(() -> {
            this.setRawArmPos(Constants.ARM_SCORE_LOW);
        }).handleInterrupt(() -> {
            this.stopArm();
        });
    }

    public Command setArmPositionSCORE_HIGH(){
        return this.runOnce(() -> {
            this.setRawArmPos(Constants.ARM_SCORE_HIGH);
        }).handleInterrupt(() -> {
            this.stopArm();
        });
    }

    public Command setArmPositionDown(){
        return this.runOnce(() -> {
            this.setRawArmPos(Constants.ARM_DOWN);
        }).handleInterrupt(() -> {
            this.stopArm();
        });  
    }

    public Command WaitForArmAbove(double angle){
        return this.run(()->{})
            .until(()->{
                return this.armMotor.getPosition().getValueAsDouble() >= angle - Constants.ARM_TOLERANCE;
            });
    }

    public Command WaitForArmBelow(double angle){
        return this.run(()->{})
            .until(()->{
                return this.armMotor.getPosition().getValueAsDouble() <= angle+Constants.ARM_TOLERANCE;
            });
    }

    public Command WaitForElevatorAbove(double angle){
        return this.run(()->{})
        .until(()->{
            return this.leftMotor.getPosition().getValueAsDouble() >= angle-Constants.ELEVATOR_TOLERANCE;
        });
    }

    public Command WaitForElevatorBelow(double angle){
        return this.run(()->{})
        .until(()->{
            return this.leftMotor.getPosition().getValueAsDouble() <= angle+Constants.ELEVATOR_TOLERANCE;
        });
    }

    public Command pickupSequence(){
        return Commands.sequence(
            this.setArmPositionOUT(),
            this.WaitForArmBelow(Constants.ARM_UPPER_ELEVATOR_CLEARANCE),
            this.setElevatorPositionTransition(),
            this.WaitForElevatorAbove(Constants.ELEVATOR_TRANSITION),
            this.setArmPositionDown(),
            this.WaitForArmBelow(Constants.ARM_DOWN),
            this.setElevatorPositionPickup(),
            this.WaitForElevatorBelow(Constants.ELEVATOR_PICKUP),
            this.setElevatorPositionTransition(),
            this.WaitForElevatorAbove(Constants.ELEVATOR_TRANSITION),
            this.setArmPositionOUT(),
            this.WaitForArmAbove(Constants.ARM_LOWER_ELEVATOR_CLEARANCE),
            this.setElevatorPositionStow(),
            this.setArmPositionStow()
        );
    }

    public Command L1Sequence() {
        return Commands.sequence(
            this.setArmPositionSCORE_LOW(),
            this.WaitForArmBelow(Constants.ARM_UPPER_ELEVATOR_CLEARANCE),
            this.setElevatorPositionL1()
        );
    }

    public Command L2Sequence() {
        return Commands.sequence(
            this.setArmPositionSCORE_LOW(),
            this.WaitForArmBelow(Constants.ARM_UPPER_ELEVATOR_CLEARANCE),
            this.setElevatorPositionL2()
        );
    }
    public Command L3Sequence() {
        return Commands.sequence(
            this.setArmPositionSCORE_LOW(),
            this.WaitForArmBelow(Constants.ARM_UPPER_ELEVATOR_CLEARANCE),
            this.setElevatorPositionL3()
        );
    }
    public Command L4Sequence() {
        return Commands.sequence(
            this.setArmPositionSCORE_HIGH(),
            this.WaitForArmBelow(Constants.ARM_UPPER_ELEVATOR_CLEARANCE),
            this.setElevatorPositionL4()
        );
    }
}
