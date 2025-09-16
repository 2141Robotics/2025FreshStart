package frc.robot.math;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;

public class Constants {

    // ************* ROBOT CONSTRUCTION CONSTANTS ***************/

    // Diameter of colson swerve wheels
    public static final double WHEEL_DIAMETER = 4;

    public static final double STEERING_RATIO = 12.8d;

    public static final double DRIVE_RATIO = 6.75d;

    // How far the shaft of the swerve module is away from the exact center of
    // rotation
    public static final double CENTER_TO_SWERVE_DIST = 10.75d;

    // ************* CONTROLLER SETTINGS ***************/

    // The port of the driver controller
    public static final int DRIVER_PORT = 0;

    // The port of the operator (secondary driver) controller
    public static final int OPERATOR_PORT = 1;

    // The dead zone of the joystick
    // (The zone where we round down to 0 to combat stick drift)
    public static final double JOYSTICK_DEAD_ZONE = 0.1d;

    // ************* MATH CONSTANTS ***************/

    /** 2π */
    public static final double TWO_PI = 2d * Math.PI;

    /** π/2 */
    public static final double PI_OVER_TWO = Math.PI / 2d;

    /** Angle precision for rotation */
    public static final double ANGLE_PRECISION = Math.PI / 32;

    public static final int INCHES_PER_FOOT = 12;

    public static final double METERS_TO_INCHES = 39.3701;

    // ************* MOVEMENT SETTINGS ***************/

    // Speed without the trigger
    public static final double BASE_SPEED = 0.08d;

    // The slowest speed the robot can go
    public static final double MINIMUM_SPEED = 0.05d;

    // The inverse constant of the rotation speed scale
    public static final double ROTATION_SPEED_INVERSE_SCALE = 35;

    // Limits change in the left joystick
    public static final double MOVEMENT_ACCELERATION_LIMIT = 0.1d;

    // Limits change in the right joystick
    public static final double ROTATION_ACCELERATION_LIMIT = 0.1d;

    // Limits change in the speed trigger
    public static final double SPEED_ACCELERATION_LIMIT = 0.1d;

    // Limits change in the overall movement of the robot
    public static final double DRIVE_ACCELERATION_LIMIT = 0.003d;

    // ************* DEVICE CONFIG SETTINGS ***************/
    /** Settings are {kF, kP, kI, kD} */
    // kV kP kI kD
    // Mainly pay attention to kP
    public static final double[] PID_SETTINGS = { 0.0d, 0.5d, 0.0d, 0.00d };

    // The offset of the roboRio gyro
    // Factored in when moving the entire drivetrain
    public static final Angle GYRO_OFFSET = Angle.ofBaseUnits(-PI_OVER_TWO, Radian);

    // ************* DEVICE ID SETTINGS ***************/
    // The IDs of the drive motors
    public static final int[] DRIVE_MOTOR_IDS = new int[] { 1, 3, 5, 7 };

    // The IDs of the drive motors
    public static final int[] STEER_MOTOR_IDS = new int[] { 2, 4, 6, 8 };

    // The CAN IDs of the canCoders
    public static final int[] CANCODER_IDS = new int[] { 21, 22, 23, 24 };

    // Individual canCoder offsets that are determined by the Phoenix Tuner X
    public static final double[] CANCODER_OFFSETS = new double[] { -.83, -.92, -.52, -.27 };

    // ************* LED SETTINGS ***************/

    // Amount of time the LEDs are on when blinking in seconds
    public static final double BLINK_ON_LENGTH = 1;

    // Amount of time the LEDs are off when blinking in seconds
    public static final double BLINK_OFF_LENGTH = 1;

    // Amount of cycles the LEDs blink for
    public static final int BLINK_CYCLES = 5;

    // The port number of the LED strip
    public static final int LED_PORT = 4;

    // The number of LEDs on the strip
    public static final int LED_COUNT = 288;

    // The offsets for the different LED strips
    // Note: The top LED strip is counted as one and is therefore given only one
    // offset at 105
    public static final int[] LED_STRIP_OFFSETS = new int[] { 0, 105, 181 };

    private static final Dimensionless LED_BRIGHTNESS = Percent.of(50);

    private static final Distance LEDPATTERN_DISTANCE = Meters.of(1.0 / 120);

    public static final Time BREATHE_LOOP_TIME = Time.ofBaseUnits(1, Second);

    public static final LEDPattern PATTERN_RED = LEDPattern.solid(Color.kRed).atBrightness(LED_BRIGHTNESS);
    public static final LEDPattern PATTERN_ORANGE = LEDPattern.solid(Color.kOrange).atBrightness(LED_BRIGHTNESS);
    public static final LEDPattern PATTERN_YELLOW = LEDPattern.solid(Color.kYellow).atBrightness(LED_BRIGHTNESS);
    public static final LEDPattern PATTERN_GREEN = LEDPattern.solid(Color.kGreen).atBrightness(LED_BRIGHTNESS);
    public static final LEDPattern PATTERN_BLUE = LEDPattern.solid(Color.kBlue).atBrightness(LED_BRIGHTNESS);
    public static final LEDPattern PATTERN_PURPLE = LEDPattern.solid(Color.kPurple).atBrightness(LED_BRIGHTNESS);
    public static final LEDPattern PATTERN_OFF = LEDPattern.kOff;

    // public static final LEDPattern PATTERN_DLS_GREEN =
    // LEDPattern.solid(new Color("#0F4D2A")).atBrightness(LED_BRIGHTNESS);

    public static final LEDPattern PATTERN_DLS_GREEN = LEDPattern.solid(new Color(15, 77, 42))
            .atBrightness(LED_BRIGHTNESS);

    public static final LEDPattern PATTERN_UP = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kPurple,
            Color.kDarkBlue);
    public static final LEDPattern PATTERN_DOWN = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kDarkBlue,
            Color.kPurple);

    public static final LEDPattern PATTERN_SCROLL = PATTERN_UP.scrollAtAbsoluteSpeed(InchesPerSecond.of(4),
            LEDPATTERN_DISTANCE);

    public static final LEDPattern PATTERN_FIRE = LEDPattern.solid(Color.kDarkRed);

    public static final int POLICE_SIREN_FREQUENCY = 40; // frequency played by motors in Hz
    public static final int POLICE_BLINK_SPEED = 3; // cycles per color change

    // 1 Seg1 On Seg2 Off
    // 2 Seg1 Off Seg2 Off
    // 3 Seg1 On Seg2 Off
    // 4 Seg1 Off Seg2 On
    // 5 Seg1 Off Seg2 Off
    // 6 Seg1 Off Seg2 On
    // 7 Seg1 On Seg2 Off
    // 8 Seg1 On Seg2 Off
    // 9 Seg1 Off Seg2 On
    // 10 Seg1 Off Seg2 On
    // 11 Seg1 On Seg2 Off
    // 12 Seg1 On Seg2 Off
    // 13 Seg1 Off Seg2 On
    // 14 Seg1 Off Seg2 On
    public static final boolean[][] POLICE_PATTERN = {
            { true, false },
            { true, false },
            { false, false },
            { false, false },
            { true, false },
            { true, false },
            { false, true },
            { false, true },
            { false, false },
            { false, false },
            { false, true },
            { false, true },
            { true, false },
            { true, false },
            { true, false },
            { true, false },
            { false, true },
            { false, true },
            { false, true },
            { false, true },
            { true, false },
            { true, false },
            { true, false },
            { true, false },
            { false, true },
            { false, true },
            { false, true },
            { false, true }
    };

    // The Police pattern is broken into eight sections
    // Blue + White + Blue + Blue + Red + Red + White + Red
    // This stat shortens the white segments by that many LEDs
    public static final int POLICE_WHITE_LENGTH_DIFFERENCE = 1;

    public static final LEDPattern PATTERN_POLICE = LEDPattern.solid(Color.kWhite);
    public static final LEDPattern PATTERN_POLICE_RED = LEDPattern.solid(Color.kRed);
    public static final LEDPattern PATTERN_POLICE_BLUE = LEDPattern.solid(Color.kBlue);

    public static final int DOTS_TRAIL_LENGTH = 10;

    // How long in cycles (20 ms intervals) it takes for a dot to travel
    // the length of the segment one way
    public static final double DOT_FREQUENCY_CYCLES = 10;

    // The color of the dots themselves
    public static final Color DOT_COLOR = Color.kWhite;
    // Just an object to refer to, this has no effect on the actual color
    public static final LEDPattern PATTERN_DOTS = LEDPattern.solid(Color.kWhite);
    // The color of the background
    public static final LEDPattern PATTERN_DOTS_BACKGROUND = LEDPattern.solid(Color.kWhite)
            .atBrightness(Percent.of(30));

    public static final LEDPattern PATTERN_RAINBOW_SCROLLING = LEDPattern.rainbow(255, 255).
    scrollAtAbsoluteSpeed(InchesPerSecond.of(1), LEDPATTERN_DISTANCE);

    public static final LEDPattern PATTERN_PARTICLES = LEDPattern.solid(Color.kWhite);

    public static final LEDPattern PATTERN_AURORA = LEDPattern.solid(Color.kWhite);

    // Particle settings
    public static final double PARTICLE_SPAWN_CHANCE = 0.05; // 5% chance per cycle
    public static final double PARTICLE_SPLIT_CHANCE = 0.02; // 2% chance per move
    public static final Color PARTICLE_COLOR = Color.kWhite;
    public static final Color PARTICLE_EXPLOSION_COLOR = Color.kOrange;

    // ************* MUSIC SETTINGS ***************/

    public static final String SONG_NAME = "src/main/deploy/Gold_On_The_Ceiling.chrp";
    // public static final String SONG_NAME =
    // "src/main/deploy/Let_It_All_Work_Out.chrp";
}