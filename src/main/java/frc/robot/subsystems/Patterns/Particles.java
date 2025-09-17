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

    // Move particles and handle collisions
    for (Particle particle : particles) {
      particle.move();

      // Check for collisions
      for (Particle other : particles) {
        if (particle != other && particle.collidesWith(other)) {
          particle.explode(segments);
          other.explode(segments);
          particlesToRemove.add(particle);
          particlesToRemove.add(other);
          break;
        }
      }

      // Remove particles that go out of bounds
      if (!particle.isInBounds()) {
        particlesToRemove.add(particle);
      }

      // Randomly split
      if (random.nextDouble() < Constants.PARTICLE_SPLIT_CHANCE) {
        newParticles.add(new Particle(particle.segment, -particle.direction));
      }
    }

    // Remove particles marked for removal
    particles.removeAll(particlesToRemove);

    // Add new particles
    particles.addAll(newParticles);

    // Randomly spawn new particles
    if (random.nextDouble() < Constants.PARTICLE_SPAWN_CHANCE) {
      int segmentIndex = random.nextInt(segments.size());
      AddressableLEDBufferView segment = segments.get(segmentIndex);
      particles.add(new Particle(segment, random.nextBoolean() ? 1 : -1));
    }

    // Render particles
    for (Particle particle : particles) {
      particle.render();
    }
  }

  private static class Particle {
    private final AddressableLEDBufferView segment;
    private int position;
    private final int direction;

    public Particle(AddressableLEDBufferView segment, int direction) {
      this.segment = segment;
      this.direction = direction;
      this.position = direction > 0 ? 0 : segment.getLength() - 1;
    }

    public void move() {
      position += direction;
    }

    public boolean collidesWith(Particle other) {
      return this.segment == other.segment && this.position == other.position;
    }

    public void explode(ArrayList<AddressableLEDBufferView> segments) {
      for (AddressableLEDBufferView segment : segments) {
        segment.setLED(position, Constants.PARTICLE_EXPLOSION_COLOR);
      }
    }

    public boolean isInBounds() {
      return position >= 0 && position < segment.getLength();
    }

    public void render() {
      segment.setLED(position, Constants.PARTICLE_COLOR);
    }
  }
}
