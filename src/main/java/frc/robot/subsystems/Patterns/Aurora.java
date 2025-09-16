package frc.robot.subsystems.Patterns;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;

public class Aurora {

    private static int cycles = 0;

    /**
     * Aurora pattern with smooth, flowing colors
     * Uses all LEDs as one
     * 
     * @param buffer
     * @param cycles
     */
    public static void runAurora(AddressableLEDBufferView buffer) {

        cycles++;

        for (int i = 0; i < buffer.getLength(); i++) { // Use buffer.getLength() instead of Constants.LED_COUNT

            // Position along strip, scaled
            double x = (double) i / buffer.getLength();

            // Wave movement using sine functions
            double wave1 = Math.sin(10 * x - cycles * 0.1);
            double wave2 = Math.sin(7 * x + cycles * 0.06);

            // Combine waves for complexity
            double intensity = (wave1 + wave2) / 2.0;

            // Scale to [0,1] range
            intensity = (intensity + 1) / 2.0;

            // Aurora color mix (green-blue blend)
            int r = 0;
            int g = (int) (200 * intensity);
            int b = (int) (255 * (0.3 + 0.7 * intensity));

            // Dim background so it looks "glowing"
            g = Math.min(255, g + 20);
            b = Math.min(255, b + 20);

            buffer.setRGB(i, r, g, b);
        }
    }
}
