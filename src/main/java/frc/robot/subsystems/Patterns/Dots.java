package frc.robot.subsystems.Patterns;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import frc.robot.math.Constants;

public class Dots {

    private static ArrayList<AddressableLEDBufferView> segmentsUnifiedTop;

    public Dots(ArrayList<AddressableLEDBufferView> segmentsUnifiedTop) {
        Dots.segmentsUnifiedTop = segmentsUnifiedTop;
    }

    private static double[] dots = new double[] {
            (1.0 * segmentsUnifiedTop.get(0).getLength()) / Constants.DOT_FREQUENCY_CYCLES,
            (1.0 * segmentsUnifiedTop.get(1).getLength()) / Constants.DOT_FREQUENCY_CYCLES,
            (1.0 * segmentsUnifiedTop.get(2).getLength()) / Constants.DOT_FREQUENCY_CYCLES };
    private static boolean[] dotsDecreasing = new boolean[] { false, false, true };

    public static void runDots() {
        for (int i = 0; i < segmentsUnifiedTop.size(); i++) {
            AddressableLEDBufferView segment = segmentsUnifiedTop.get(i);
            int length = segment.getLength();
            if (dots[i] <= 0 || dots[i] >= length - 1) {
                dotsDecreasing[i] = !dotsDecreasing[i];
            }
            if (dotsDecreasing[i]) {
                dots[i] -= length / Constants.DOT_FREQUENCY_CYCLES;
            } else {
                dots[i] += length / Constants.DOT_FREQUENCY_CYCLES;
            }
            Constants.PATTERN_DOTS_BACKGROUND.applyTo(segment);
            segment.setRGB((int) dots[i], 255, 255, 255);
            for (int j = 0; j < Constants.DOTS_TRAIL_LENGTH; j++) {
                int brightness = 255 - (j * (255 / Constants.DOTS_TRAIL_LENGTH));
                if (dotsDecreasing[i] && dots[i] + j < length) {
                    segment.setRGB((int) dots[i] + j,
                            (int) Constants.DOT_COLOR.red * brightness,
                            (int) Constants.DOT_COLOR.blue * brightness,
                            (int) Constants.DOT_COLOR.green * brightness);
                }
                if (!dotsDecreasing[i] && dots[i] - j > 0) {
                    brightness = 255 - (j * (255 / Constants.DOTS_TRAIL_LENGTH));
                    segment.setRGB((int) dots[i] - j,
                            (int) Constants.DOT_COLOR.red * brightness,
                            (int) Constants.DOT_COLOR.blue * brightness,
                            (int) Constants.DOT_COLOR.green * brightness);
                }
            }
        }
    }
}
