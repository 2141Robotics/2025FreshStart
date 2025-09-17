package frc.robot.subsystems.Patterns;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import frc.robot.math.Constants;
import java.util.ArrayList;

public class Dots {

  private static double[] dots = new double[] {(-1), (-1), (-1)};
  private static boolean[] dotsDecreasing = new boolean[] {false, false, true};

  public static void runDots(ArrayList<AddressableLEDBufferView> segmentsUnifiedTop) {
    for (int i = 0; i < segmentsUnifiedTop.size(); i++) {

      if (dots[i] == -1) {
        dots[i] = (1.0 * segmentsUnifiedTop.get(i).getLength()) / (Constants.DOT_FREQUENCY_CYCLES);
      }
      AddressableLEDBufferView segment = segmentsUnifiedTop.get(i);
      int length = segment.getLength();
      if (dotsDecreasing[i]) {
        dots[i] -= length / Constants.DOT_FREQUENCY_CYCLES;
      } else {
        dots[i] += length / Constants.DOT_FREQUENCY_CYCLES;
      }
      if (dots[i] <= 0 || dots[i] >= length - 1) {
        dotsDecreasing[i] = !dotsDecreasing[i];
        if (dots[i] <= 0) {
          dots[i] = 0;
        } else {
          dots[i] = length - 1;
        }
      }
      Constants.PATTERN_DOTS_BACKGROUND.applyTo(segment);
      segment.setRGB((int) dots[i], 255, 255, 255);
      for (int j = 0; j < Constants.DOTS_TRAIL_LENGTH; j++) {
        int brightness = 255 - (j * (255 / Constants.DOTS_TRAIL_LENGTH));
        if (dotsDecreasing[i] && dots[i] + j < length) {
          segment.setRGB(
              (int) dots[i] + j,
              (int) Constants.DOT_COLOR.red * brightness,
              (int) Constants.DOT_COLOR.blue * brightness,
              (int) Constants.DOT_COLOR.green * brightness);
        }
        if (!dotsDecreasing[i] && dots[i] - j > 0) {
          brightness = 255 - (j * (255 / Constants.DOTS_TRAIL_LENGTH));
          segment.setRGB(
              (int) dots[i] - j,
              (int) Constants.DOT_COLOR.red * brightness,
              (int) Constants.DOT_COLOR.blue * brightness,
              (int) Constants.DOT_COLOR.green * brightness);
        }
      }
    }
  }
}
