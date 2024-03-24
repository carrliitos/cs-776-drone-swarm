public class Clamp {
  private double value = 0.0;
  private double value_min = 0.0;
  private double value_max = 0.0;

  public Clamp() { }

  public void setValue(double value, double value_min, double value_max) {
    this.value = value;
    this.value_min = value_min;
    this.value_max = value_max;
  }

  public double getValue() {
    return Math.min(Math.max(value, value_min), value_max);
  }
}