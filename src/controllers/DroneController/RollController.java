public class RollController {
  private final double roll_Kp;
  private final double roll_Ki;
  private final double roll_Kd;

  private double rollIntegral = 0.0;
  private double previousRollError = 0.0;

  public RollController(double[] roll_pid) {
    this.roll_Kp = roll_pid[0];
    this.roll_Ki = roll_pid[1];
    this.roll_Kd = roll_pid[2];
  }

  public double calculateRollPID(double currentRollAngle, double setpoint) {
    // Calculate the error
    double rollError = setpoint - currentRollAngle;

    // Calculate the proportional term
    double P = roll_Kp * rollError;

    // Calculate the integral term with anti-windup
    rollIntegral += rollError;
    if (rollIntegral > 100) {
      rollIntegral = 100;  // Limit the integral term
    } else if (rollIntegral < -100) {
      rollIntegral = -100;  // Limit the integral term
    }
    double I = roll_Ki * rollIntegral;

    // Calculate the derivative term
    double rollDerivative = rollError - previousRollError;
    double D = roll_Kd * rollDerivative;

    // Update the previous error
    previousRollError = rollError;

    // Calculate the PID output
    double rollPID = P + I + D;

    return rollPID;
  }
}
