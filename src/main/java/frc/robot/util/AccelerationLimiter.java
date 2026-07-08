package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class AccelerationLimiter {
  private static final double MAX_DT_SECONDS = 0.1;
  private static final double RESET_DT_SECONDS = 0.2;

  private final double maxRate;
  private final double curveExponent;

  private double prevValue;
  private double prevTime;
  private boolean firstCall;

  public AccelerationLimiter(double rate, double exponent) {
    this.maxRate = rate;
    this.curveExponent = exponent;
    prevValue = 0.0;
    prevTime = 0.0;
    firstCall = true;
  }

  public double calculate(double input) {
    double now = Timer.getFPGATimestamp();
    double dt = now - prevTime;

    if (firstCall || dt > RESET_DT_SECONDS) {
      prevValue = input;
      prevTime = now;
      firstCall = false;
      return input;
    }

    if (dt <= 0.0) {
      return prevValue;
    }
    if (dt > MAX_DT_SECONDS) {
      dt = MAX_DT_SECONDS;
    }

    double prevMag = Math.abs(prevValue);
    double inputMag = Math.abs(input);

    if (inputMag < prevMag) {
      prevValue = input;
    } else {
      double distanceFromMax = 1.0 - prevMag;
      double rate = maxRate * Math.pow(distanceFromMax, curveExponent);
      double maxChange = rate * dt;
      double remaining = inputMag - prevMag;
      double delta = Math.min(remaining, maxChange);
      double newMag = prevMag + delta;
      prevValue = Math.signum(input) * newMag;
    }

    prevTime = now;
    return prevValue;
  }

  public void reset() {
    firstCall = true;
  }
}
