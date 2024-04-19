public class PID {
  private double setPoint;
  private double kP, kI, kD;
  private double minLimit = Double.NaN;
  private double maxLimit = Double.NaN;
  private double previousTime = Double.NaN;
  private double lastError = 0;
  private double integralError = 0;

  public PID(final double kP, final double kI, final double kD, final double setPoint) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.setSetpoint(setPoint);
  }

  public double getOutput(final double currentTime, final double currentValue) {
    final double error = setPoint - currentValue;
    final double dt = (previousTime != Double.NaN) ? (double)(currentTime - previousTime) : 0;
    
    // Compute Integral & Derivative error
    final double derivativeError = (dt != 0) ? ((error - lastError) / dt) : 0;
    integralError += error * dt;
    
    // Save history
    previousTime = currentTime;
    lastError = error;
    
    return checkLimits((kP * error) + (kI * integralError) + (kD * derivativeError));
  }

  public void reset() {
    previousTime = 0;
    lastError = 0;
    integralError = 0;
  }

  private double checkLimits(final double output){
    if (!Double.isNaN(minLimit) && output < minLimit) {
      return minLimit;
    } else if (!Double.isNaN(maxLimit) && output > maxLimit) {
      return maxLimit;
    } else {
      return output;
    }
  }

  public void setOuputLimits(final double minLimit, final double maxLimit) {
    if (minLimit < maxLimit) {
      this.minLimit = minLimit;
      this.maxLimit = maxLimit;
    } else {
      this.minLimit = maxLimit;
      this.maxLimit = minLimit;
    }
  }
  
  public void removeOuputLimits() {
    this.minLimit = Double.NaN;
    this.maxLimit = Double.NaN;
  }
  
  public double getkP() {
    return kP;
  }

  public void setkP(double kP) {
    this.kP = kP;
    reset();
  }

  public double getkI() {
    return kI;
  }

  public void setkI(double kI) {
    this.kI = kI;
    reset();
  }

  public double getkD() {
    return kD;
  }

  public void setkD(double kD) {
    this.kD = kD;
    reset();
  }

  public double getSetPoint() {
    return setPoint;
  }
  
  public void setSetpoint(final double setPoint) {
    reset();
    this.setPoint = setPoint;
  }
}
