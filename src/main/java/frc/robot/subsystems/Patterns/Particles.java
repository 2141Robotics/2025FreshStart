package frc.robot.subsystems.Patterns;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import frc.robot.math.Constants;
import java.util.ArrayList;
import java.util.Random;

public class Particles {

  private static final Random random = new Random();
  private static final ArrayList<Particle> particles = new ArrayList<>();

  public static void runParticles(ArrayList<AddressableLEDBufferView> segments) {
    ArrayList<Particle> newParticles = new ArrayList<>();
    ArrayList<Particle> particlesToRemove = new ArrayList<>();

    // Render the background
    for (AddressableLEDBufferView segment : segments) {
      for (int i = 0; i < segment.getLength(); i++) {
        segment.setLED(i, Constants.PARTICLE_BACKGROUND_COLOR);
      }
    }

    // Move particles and handle collisions
    for (Particle particle : particles) {
      // Random chance for the particle to disappear
      if (random.nextDouble() < Constants.PARTICLE_DISAPPEAR_CHANCE) {
        particlesToRemove.add(particle);
        continue;
      }

      particle.move();

      // Check for collisions
      for (Particle other : particles) {
        if (particle != other && particle.collidesWith(other)) {
          particle.explode(segments);
          particle.bounce();
          other.bounce();
        }
      }

      // Randomly split
      if (random.nextDouble() < Constants.PARTICLE_SPLIT_CHANCE) {
        newParticles.add(new Particle(particle.segment, -particle.direction));
      }
    }

    // Remove particles marked for removal
    particles.removeAll(particlesToRemove);

    // Add new particles
    if (random.nextDouble() < Constants.PARTICLE_SPAWN_CHANCE 
      && particles.size() < Constants.PARTICLE_MAX_COUNT) {
      int segmentIndex = random.nextInt(segments.size());
      AddressableLEDBufferView segment = segments.get(segmentIndex);
      particles.add(new Particle(segment, random.nextBoolean() ? 1 : -1));
    }

    // Add newly split particles
    particles.addAll(newParticles);

    // Render particles
    for (Particle particle : particles) {
      particle.render();
    }
  }

  private static class Particle {
    private final AddressableLEDBufferView segment;
    private int position;
    private int direction;
    private int r, g, b;

    public Particle(AddressableLEDBufferView segment, int direction) {
      this.r = random.nextInt(0,255);
      this.g = random.nextInt(0,255);
      this.b = random.nextInt(0,255);
      this.segment = segment;
      this.direction = direction;
      this.position = direction > 0 ? 0 : segment.getLength() - 1;
    }

    public void move() {
      position += direction;

      // Bounce if hitting the end of the segment
      if (position < 0 || position >= segment.getLength()) {
        bounce();
      }
    }

    public boolean collidesWith(Particle other) {
      return this.segment == other.segment && this.position == other.position;
    }

    public void explode(ArrayList<AddressableLEDBufferView> segments) {
      for (AddressableLEDBufferView segment : segments) {
        if (position >= 0 && position < segment.getLength()) {
          segment.setLED(position, Constants.PARTICLE_EXPLOSION_COLOR);
        }
      }
    }

    public void bounce() {
      direction = -direction; // Reverse direction
      position = Math.max(0, Math.min(position, segment.getLength() - 1)); // Clamp position
    }

    public void render() {
      if (position >= 0 && position < segment.getLength()) { // Ensure position is valid
        segment.setRGB(position, r,g,b);
      }
    }
  }
}
