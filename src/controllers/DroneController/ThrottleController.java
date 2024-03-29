/**
 * A PID controller for throttle control in a drone.
 */
public class ThrottleController {
  private final double throttle_Kp;
  private final double throttle_Ki;
  private final double throttle_Kd;

  private double throttleIntegral = 0.0;
  private double previousThrottleError = 0.0;

  /**
   * Constructs a new ThrottleController with the given PID constants.
   *
   * @param throttle_pid an array containing the PID constants [Kp, Ki, Kd]
   */
  public ThrottleController(double[] throttle_pid) {
    this.throttle_Kp = throttle_pid[0];
    this.throttle_Ki = throttle_pid[1];
    this.throttle_Kd = throttle_pid[2];
  }

  /**
   * Calculates the PID output for throttle control based on the current altitude and the setpoint.
   *
   * @param currentAltitude the current altitude of the drone
   * @param setpoint the desired altitude (setpoint)
   * @return the PID output for throttle control
   */
  public double calculateThrottlePID(double currentAltitude, double setpoint) {
    // Calculate the error
    double throttleError = setpoint - currentAltitude;

    // Calculate the proportional term
    double P = throttle_Kp * throttleError;

    // Calculate the integral term
    throttleIntegral += throttleError;
    double I = throttle_Ki * throttleIntegral;

    // Calculate the derivative term
    double throttleDerivative = throttleError - previousThrottleError;
    double D = throttle_Kd * throttleDerivative;

    // Update the previous error
    previousThrottleError = throttleError;

    // Calculate the PID output
    double throttlePID = P + I + D;

    return throttlePID;
  }
}
