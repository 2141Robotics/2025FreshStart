package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.math.Constants;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

public class LEDs extends SubsystemBase {

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  private AddressableLEDBufferView left;
  private AddressableLEDBufferView right;
  private AddressableLEDBufferView topleft;
  private AddressableLEDBufferView topright;
  private AddressableLEDBufferView top;
  private AddressableLEDBufferView whole;
  private ArrayList<AddressableLEDBufferView> segmentsUnifiedTop;
  private ArrayList<AddressableLEDBufferView> segmentsSplitTop;

  private boolean breathing = false;

  Random random = new Random();

  Integer length;
  List<Integer> heat;

  public LEDPattern oldPattern;
  public LEDPattern currentPattern = Constants.PATTERN_YELLOW;

  private int cyclesWhileBlinking = 0;
  private boolean blinking;
  private int policeBlinkCycles = 0;
  private boolean policeInverted = false;

  private int[] dots;
  private boolean[] dotsDecreasing;

  public LEDs() {
    m_led = new AddressableLED(Constants.LED_PORT);
    m_buffer = new AddressableLEDBuffer(Constants.LED_COUNT);
    m_led.setLength(Constants.LED_COUNT);
    m_led.start();
    // 288 led total
    this.left = m_buffer.createView(0, 104);
    this.topleft = m_buffer.createView(105, 142);
    this.topright = m_buffer.createView(143, 180).reversed();
    this.right = m_buffer.createView(181, 286).reversed();

    this.top = m_buffer.createView(105, 180);
    
    this.whole = m_buffer.createView(0, 286);

    segmentsUnifiedTop = new ArrayList<>();
    segmentsUnifiedTop.add(left);
    segmentsUnifiedTop.add(top);
    segmentsUnifiedTop.add(right);

    segmentsSplitTop = new ArrayList<>();
    segmentsSplitTop.add(left);
    segmentsSplitTop.add(topleft);
    segmentsSplitTop.add(topright);
    segmentsSplitTop.add(right);

    dots = new int[] {0, 0, 0};
    dotsDecreasing = new boolean[] {false, false, true};

    runPattern(Constants.PATTERN_YELLOW);
    updatePattern();
    this.length = left.getLength();
    heat = new ArrayList<>(Collections.nCopies(length, 0));

    policeBlinkCycles = 0;
  }

  @Override
  public void periodic() {

    if (this.currentPattern == Constants.PATTERN_FIRE) {
      runFire();
    } else if (this.currentPattern == Constants.PATTERN_POLICE) {
      runPolice();
    } else if (this.currentPattern == Constants.PATTERN_DOTS) {
      runDots();
    } else {
      updatePattern();
    }
    m_led.setData(m_buffer);
  }

  public void runDots() {
    for (int i = 0; i < segmentsUnifiedTop.size(); i++) {
      AddressableLEDBufferView segment = segmentsUnifiedTop.get(i);
      int length = segment.getLength();
      if (dots[i] == 0 || dots[i] == length - 1) {
        dotsDecreasing[i] = !dotsDecreasing[i];
      }
      if (dotsDecreasing[i]) {
        dots[i]--;
      } else {
        dots[i]++;
      }
      Constants.PATTERN_OFF.applyTo(segment);
      segment.setRGB(dots[i], 255, 255, 255);
      for (int j = 0; j < Constants.DOTS_TRAIL_LENGTH; j++) {
        int brightness = 255 - (j * (255 / Constants.DOTS_TRAIL_LENGTH));
        if (dotsDecreasing[i] && dots[i] + j < length) {
          segment.setRGB(dots[i] + j, brightness, brightness, brightness);
        }
        if (!dotsDecreasing[i] && dots[i] - j > 0) {
          brightness = 255 - (j * (255 / Constants.DOTS_TRAIL_LENGTH));
          segment.setRGB(dots[i] - j, brightness, brightness, brightness);
        }
      }
    }
  }

  public void runPolice() {
    if (policeBlinkCycles > Constants.POLICE_BLINK_SPEED) {
      policeBlinkCycles = 0;
      for (AddressableLEDBufferView segment : segmentsUnifiedTop) {
        int length = segment.getLength();
        AddressableLEDBufferView segment1 = m_buffer.createView(0, length / 2);
        AddressableLEDBufferView segment2 = m_buffer.createView((length / 2), length - 1);
        if (policeInverted) {
          Constants.PATTERN_POLICE_RED.applyTo(segment1);
          Constants.PATTERN_POLICE_BLUE.applyTo(segment2);
        } else {
          Constants.PATTERN_POLICE_BLUE.applyTo(segment1);
          Constants.PATTERN_POLICE_RED.applyTo(segment2);
        }
        policeInverted = !policeInverted;
      }
    } else {
      policeBlinkCycles++;
    }
  }

  public void runFire() {
    this.fire(left, 50, 60);
    this.fire(right, 50, 60);
  }

  public void updatePattern() {
    if (this.currentPattern != this.oldPattern || blinking) {
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
    if(breathing){
      pattern = pattern.breathe(Time.ofBaseUnits(1, Seconds));
    }
    if (blinking) {
      pattern =
          pattern.blink(
              Time.ofBaseUnits(Constants.BLINK_ON_LENGTH, Seconds),
              Time.ofBaseUnits(Constants.BLINK_OFF_LENGTH, Seconds));
      cyclesWhileBlinking++;
      if (cyclesWhileBlinking > Constants.BLINK_CYCLES) {
        blinking = false;
        cyclesWhileBlinking = 0;
      }
    }

    if(pattern == Constants.PATTERN_RAINBOW_SCROLLING){
      pattern.applyTo(this.whole);
    }else{

      pattern.applyTo(this.left);
      pattern.applyTo(this.right);

      pattern.applyTo(this.topleft);
      pattern.applyTo(this.topright);
    }
    this.oldPattern = this.currentPattern;
  }

  public Command blinkLEDs() {
    return this.runOnce(() -> this.blink());
  }

  public Command setPatternCommand(LEDPattern pattern) {
    return this.runOnce(() -> this.setPattern(pattern));
  }

  public Command setBreatheCommand(boolean b) {
    return this.runOnce(() -> this.setBreathe(b));
  }

  private void setPattern(LEDPattern pattern) {
    this.currentPattern = pattern;
    this.runPattern(pattern);
  }

  private void setBreathe(boolean b) {
    this.breathing = b;
  }


  public void blink() {
    System.out.println("Blinking LEDS");
    blinking = true;
  }
}
