package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.math.Constants;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

public class LEDSubsystem extends SubsystemBase {
  private static final int kPort = 4;
  private static final int kLength = 288;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  AddressableLEDBufferView left;
  private AddressableLEDBufferView right;
  private AddressableLEDBufferView topleft;
  private AddressableLEDBufferView topright;
  private static final LinearVelocity LEDPATTERN_VELOCITY = MetersPerSecond.of(1);
  private static final Distance LEDPATTERN_DISTANCE = Meters.of(1 / 120);

  private static final Dimensionless brightness = Percent.of(50);

  Random random = new Random();

  Integer length;
  List<Integer> heat;

  LEDPattern m_red = LEDPattern.solid(Color.kRed).atBrightness(brightness);
  LEDPattern m_orange = LEDPattern.solid(Color.kOrange).atBrightness(brightness);
  LEDPattern m_blue = LEDPattern.solid(Color.kBlue).atBrightness(brightness);
  LEDPattern m_yellow = LEDPattern.solid(Color.kYellow).atBrightness(brightness);
  LEDPattern m_green = LEDPattern.solid(Color.kGreen).atBrightness(brightness);
  LEDPattern m_off = LEDPattern.solid(Color.kWhite).atBrightness(Percent.of(5));

  LEDPattern m_fire = LEDPattern.solid(Color.kDarkRed);

  LEDPattern m_up =
      LEDPattern.gradient(GradientType.kDiscontinuous, Color.kPurple, Color.kDarkBlue);
  LEDPattern m_down =
      LEDPattern.gradient(GradientType.kDiscontinuous, Color.kDarkBlue, Color.kPurple);
  LEDPattern ready_scroll = m_up.scrollAtAbsoluteSpeed(LEDPATTERN_VELOCITY, LEDPATTERN_DISTANCE);

  public LEDPattern oldPattern;
  public LEDPattern currentPattern = m_yellow;

  private int cyclesWhileBlinking = 0;
  private boolean blinking;

  private ElevatorArm ele;

  public LEDSubsystem(ElevatorArm elevator) {
    this.ele = elevator;
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();
    // 288 led total
    this.left = m_buffer.createView(0, 105);
    this.topleft = m_buffer.createView(106, 143);
    this.topright = m_buffer.createView(144, 181).reversed();
    this.right = m_buffer.createView(182, 287).reversed();
    runPattern(m_yellow);
    updatePattern();
    this.length = left.getLength();
    heat = new ArrayList<>(Collections.nCopies(length, 0));
  }

  @Override
  public void periodic() {
    switch (this.ele.getCoralState()) {
      case NO_CORAL:
        this.currentPattern = m_yellow;
        break;

      case CORAL_HOPPER:
        this.currentPattern = m_blue;
        break;

      case CORAL_ARM:
        this.currentPattern = m_green;
        break;

      default:
        break;
    }

    switch (this.ele.getElevatorState()) {
      case BLOCKED:
        // this.currentPattern = m_red;
        break;

      case MOVING_UP:
        this.currentPattern = m_up;
        break;

      case MOVING_DOWN:
        this.currentPattern = m_down;
        break;

      default:
        break;
    }

    updatePattern();
    m_led.setData(m_buffer);
  }

  public void runFire() {
    this.fire(left, 50, 60);
    this.fire(right, 50, 60);
  }

  public void updatePattern() {
    if (this.currentPattern == m_fire) {
    } else if (this.currentPattern != this.oldPattern || blinking) {
      this.runPattern(currentPattern);
    }
  }

  public void resetAnimation() {
    this.runPattern(currentPattern);
  }

  public void fire(AddressableLEDBufferView bufferView, int flameHight, int sparks) {

    for (int i = 0; i < length; i++) {
      int cooldown = random.nextInt(((flameHight * 10) / length) + 2);
      if (cooldown > heat.get(i)) {
        heat.set(i, 0);
      } else {
        heat.set(i, heat.get(i) - cooldown);
      }
    }

    for (int k = length - 1; k >= 2; k--) {
      heat.set(k, (heat.get(k - 1) + 2 * heat.get(k - 1)) / 3);
    }

    if (random.nextInt(255) < sparks) {
      Integer y = random.nextInt(7);
      heat.set(y, random.nextInt(160, 255));
    }

    for (int j = 0; j < length; j++) {
      int temperature = heat.get(j);

      if (temperature > 220) {
        bufferView.setRGB(j, 220, 220, temperature);
      } else if (temperature > 40) {
        bufferView.setRGB(j, 220, temperature, 0);
      } else {
        bufferView.setRGB(j, temperature, 0, 0);
      }
    }
  }

  public void runPattern(LEDPattern pattern) {

    if (blinking) {
      pattern =
          pattern.blink(
              Time.ofBaseUnits(Constants.blinkOnLength, Seconds),
              Time.ofBaseUnits(Constants.blinkOffLength, Seconds));
      cyclesWhileBlinking++;
      if (cyclesWhileBlinking > Constants.blinkLength) {
        blinking = false;
        cyclesWhileBlinking = 0;
      }
    }

    pattern.applyTo(this.left);
    pattern.applyTo(this.right);

    pattern.applyTo(this.topleft);
    pattern.applyTo(this.topright);
    this.oldPattern = this.currentPattern;
  }

  public Command blinkLEDs() {
    return this.runOnce(() -> this.blink());
  }

  public void blink() {
    System.out.println("Blinking LEDS");
    blinking = true;
  }
}
