package frc.robot.subsystems.Patterns;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import java.util.List;
import java.util.Random;

public class Fire {

  private static Random random = new Random();

  public static List<Integer> fire(
      AddressableLEDBufferView bufferView,
      int flameHight,
      int sparks,
      int length,
      List<Integer> heat) {

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
    return heat;
  }
}
