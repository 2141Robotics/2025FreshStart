package frc.robot.math;

public class MechanismStates {
        public enum OdometryState {
            LL_HAS_POSE,
            HAS_POSE,
            NEVER_VISION_POSE
        }
        public enum CoralState {
            NO_CORAL,
            CORAL_HOPPER,
            CORAL_ARM
        }
        public enum ElevatorState {
            BLOCKED,
            STOWED,
            MOVING_UP,
            MOVING_DOWN,
            IN_POSITION,
            NONE
        }
}
