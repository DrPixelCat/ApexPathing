package setup;

/** Marker for robot configurations that intentionally opt into the saved setup profile. */
public interface SavedSetup {
    RobotProfile robotProfile();
}
