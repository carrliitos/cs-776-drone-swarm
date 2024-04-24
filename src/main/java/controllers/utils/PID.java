public class PID {
  private double kp;  // Proportional gain
  private double ki;  // Integral gain
  private double kd;  // Derivative gain
  private double setpoint;  // Desired setpoint
  private double integral;  // Integral term
  private double prevInput;  // Previous input value
  private double prevTime;  // Previous time value
  private boolean autoMode;  // Automatic mode flag
  private double lastOutput;  // Last computed output

  public PID(double kp, double ki, double kd, double setpoint) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.setpoint = setpoint;
    this.integral = 0;
    this.prevInput = 0;
    this.prevTime = 0;
    this.autoMode = true;
    this.lastOutput = 0;
  }

  public double update(double input, double dt) {
    if (!autoMode) {
      return lastOutput;
    }

    double error = setpoint - input;
    double dInput = input - prevInput;
    double now = System.currentTimeMillis() / 1000.0;  // Current time in seconds
    double deltaTime = now - prevTime;

    if (dt == 0.0) {
      dt = deltaTime != 0 ? deltaTime : 1e-16;
    } else if (dt <= 0) {
      throw new IllegalArgumentException("dt has negative value " + dt + ", must be positive");
    }

    // Compute proportional term
    double proportional = kp * error;

    // Compute integral term
    integral += ki * error * dt;
    integral = clamp(integral, -1, 1);  // Limit integral term to prevent windup

    // Compute derivative term
    double derivative = 0;
    if (dt > 0) {
      derivative = -kd * dInput / dt;
    }

    // Compute output
    double output = proportional + integral + derivative;
    output = clamp(output, -1, 1);  // Limit output to output limits

    // Update state variables
    lastOutput = output;
    prevInput = input;
    prevTime = now;

    return output;
  }

  private double clamp(double value, double low, double high) {
    return value < low ? low : (value > high ? high : value);
  }

  public void setAutoMode(boolean autoMode) {
    this.autoMode = autoMode;
  }

  public void setPoint(double setpoint) {
    this.setpoint = setpoint;
  }
}
