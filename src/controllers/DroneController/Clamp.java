/**
 * The Clamp class provides a static method to clamp a value between a specified range.
 */
public class Clamp {
    /**
     * Clamps a value between a specified range.
     *
     * @param value The value to be clamped.
     * @param low   The lower bound of the range.
     * @param high  The upper bound of the range.
     * @return The clamped value.
     */
    public static double clamp(double value, double low, double high) {
        return Math.min(Math.max(value, low), high);
    }
}
