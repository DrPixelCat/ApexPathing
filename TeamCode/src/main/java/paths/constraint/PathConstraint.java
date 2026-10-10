package paths.constraint;

/**
 * Describes a velocity or acceleration limit at a normalized path position.
 *
 * <p>This interface exposes the position and type of a constraint. Concrete
 * implementations define the limit's value, units, and whether it applies to
 * translational or angular motion.
 *
 * @see TranslationalConstraint
 * @see AngularConstraint
 */
public interface PathConstraint {

    /**
     * Identifies the motion quantity limited by a constraint.
     */
    enum Type {
        /** A limit on translational or angular velocity. */
        VELOCITY,

        /** A limit on translational or angular acceleration. */
        ACCELERATION
    }

    /**
     * Returns the normalized position in the path's constraint coordinate system
     * where this limit takes effect.
     *
     * @return the constraint position, conventionally in {@code [0.0, 1.0]},
     *         from the start to the end of the constraint coordinate system
     */
    double getS();

    /**
     * Sets the normalized position where this limit takes effect.
     *
     * <p>Positions are conventionally in {@code [0.0, 1.0]}. This interface does
     * not require implementations to validate or clamp the supplied value.
     *
     * @param s the position in the path's constraint coordinate system
     * @see #getS()
     */
    void setS(double s);

    /**
     * Returns the motion quantity limited by this constraint.
     *
     * @return the velocity or acceleration constraint type
     */
    Type getType();

    /**
     * Attempts to register a callback for changes to this constraint.
     *
     * <p>Consumers can use notifications to invalidate cached constraint data.
     * An implementation that returns {@code true} must notify the listener when
     * changes affect the constraint. If registration is unsupported, consumers
     * must read the constraint again to observe changes.
     *
     * @implSpec The default implementation returns {@code false} without storing
     *           or invoking the listener, and does not validate the argument.
     * @param listener the callback to register; callers should supply a non-null value
     * @return {@code true} if change notifications are supported and the listener
     *         was registered; {@code false} otherwise
     */
    default boolean addChangeListener(Runnable listener) {
        return false;
    }
}
