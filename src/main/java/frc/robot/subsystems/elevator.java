package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.mineinjava.quail.util.geometry.Vec2d;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.math.Constants;

// 1. Initialize talons
// 2. Configure talons
// 3. Up and down commands

public class elevator extends SubsystemBase {
    private final TalonFX leftMotor;

    private final TalonFX rightMotor;

public void init(){
    System.out.println("Initializing elevator!");

    SoftwareLimitSwitchConfigs elevatorMotorSoftwareLimitSwitchConfig = new SoftwareLimitSwitchConfigs().      withForwardSoftLimitEnable(true).
    withForwardSoftLimitThreshold(Constants.ELEVATOR_MAX_ROTS)
    .withReverseSoftLimitEnable(true)
    .withReverseSoftLimitThreshold(Constants.ELEVATOR_MIN_ROTS);

    MotorOutputConfigs leftMotorConfig = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);
    TalonFXConfiguration leftDriveTalonConfig = new TalonFXConfiguration().withMotorOutput(leftMotorConfig).withSoftwareLimitSwitch(elevatorMotorSoftwareLimitSwitchConfig);

    MotorOutputConfigs rightMotorConfig = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);
    TalonFXConfiguration rightDriveTalonConfig = new TalonFXConfiguration().withMotorOutput(rightMotorConfig).withSoftwareLimitSwitch(elevatorMotorSoftwareLimitSwitchConfig);;

    this.leftMotor.getConfigurator().apply(leftDriveTalonConfig);
    this.rightMotor.getConfigurator().apply(rightDriveTalonConfig);

    // TODO: Right follow left

    // 0 the positions
    this.leftMotor.setPosition(0);
    this.rightMotor.setPosition(0);
}

    public elevator(int leftMotorID, int rightMotorID) {
        this.leftMotor = new TalonFX(leftMotorID);
        this.rightMotor = new TalonFX(rightMotorID);
    }

    public void stop() {
        this.leftMotor.stopMotor();
        this.rightMotor.stopMotor();
    }

    public void setRawSpeed(double speed) {
        this.leftMotor.setControl(new DutyCycleOut(speed));
        this.rightMotor.setControl(new DutyCycleOut(speed));
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
        Angle pos = this.leftMotor.getPosition().getValue();
        double posAsRotations = pos.in(Rotations);
        SmartDashboard.putNumber("elevator position", posAsRotations);

    }

}
