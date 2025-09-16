package frc.robot.subsystems.Patterns;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import frc.robot.math.Constants;

public class Police {

    private static int policeBlinkCycles = 0;

    public static void runPolice(ArrayList<AddressableLEDBufferView> segmentsUnifiedTop, AddressableLEDBuffer buffer) {
        if (policeBlinkCycles >= Constants.POLICE_PATTERN.length) {
            policeBlinkCycles = 0;
        }

        int startIndex = 0;
        for (AddressableLEDBufferView segment : segmentsUnifiedTop) {
            int length = segment.getLength();
            int offset = Constants.LED_STRIP_OFFSETS[startIndex];
            startIndex++;
            int odd = 0;
            if (length % 2 == 1) {
                odd = 1;
            }
            AddressableLEDBufferView blue1 = buffer.createView(
                    offset, offset + length / 8 + Constants.POLICE_WHITE_LENGTH_DIFFERENCE);
            AddressableLEDBufferView white1 = buffer.createView(
                    Constants.POLICE_WHITE_LENGTH_DIFFERENCE + offset + length / 8,
                    offset - Constants.POLICE_WHITE_LENGTH_DIFFERENCE + length / 4);
            AddressableLEDBufferView blue2 = buffer.createView(
                    offset - Constants.POLICE_WHITE_LENGTH_DIFFERENCE + length / 4,
                    offset + length / 2 + odd);
            AddressableLEDBufferView red1 = buffer.createView(
                    offset + length / 2,
                    offset + ((3 * length) / 4) + Constants.POLICE_WHITE_LENGTH_DIFFERENCE);
            AddressableLEDBufferView white2 = buffer.createView(
                    offset + Constants.POLICE_WHITE_LENGTH_DIFFERENCE + (3 * length) / 4,
                    offset + ((7 * length) / 8) - Constants.POLICE_WHITE_LENGTH_DIFFERENCE);
            AddressableLEDBufferView red2 = buffer.createView(
                    offset - Constants.POLICE_WHITE_LENGTH_DIFFERENCE + (7 * length) / 8,
                    offset + length - 1);

            Constants.PATTERN_POLICE.applyTo(white1);
            Constants.PATTERN_POLICE.applyTo(white2);

            if (Constants.POLICE_PATTERN[policeBlinkCycles][0]) {

                Constants.PATTERN_POLICE_RED.applyTo(red1);
                Constants.PATTERN_POLICE_BLUE.applyTo(blue1);

            } else {

                Constants.PATTERN_OFF.applyTo(red1);
                Constants.PATTERN_OFF.applyTo(blue1);
            }
            if (Constants.POLICE_PATTERN[policeBlinkCycles][1]) {

                Constants.PATTERN_POLICE_RED.applyTo(red2);
                Constants.PATTERN_POLICE_BLUE.applyTo(blue2);

            } else {

                Constants.PATTERN_OFF.applyTo(red2);
                Constants.PATTERN_OFF.applyTo(blue2);
            }
        }
        policeBlinkCycles++;
    }
}
