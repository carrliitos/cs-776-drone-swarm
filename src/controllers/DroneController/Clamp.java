/**
 * The Clamp class ensures that a given value stays within a specified range.
 */
public class Clamp {
    private double value;
    private double value_min;
    private double value_max;

    /**
     * Sets the value, minimum, and maximum values for clamping.
     *
     * @param value     The value to be clamped.
     * @param value_min The minimum value allowed.
     * @param value_max The maximum value allowed.
     */
    public void setValue(double value, double value_min, double value_max) {
      this.value = value;
      this.value_min = value_min;
      this.value_max = value_max;
    }

    /**
     * Gets the clamped value within the specified range.
     *
     * @return The clamped value.
     */
    public double getValue() {
      return Math.min(Math.max(value, value_min), value_max);
    }
}
