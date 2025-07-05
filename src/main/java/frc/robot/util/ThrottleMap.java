package frc.robot.util;

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import edu.wpi.first.math.MathUtil;

/**
 * Utility class that performs a throttle mapping using a piecewise linear spline.
 * This mapping adjusts the response curve of an input, typically from an analog control device,
 * to produce a smoother or more customized output behavior.
 *
 * The curve is defined by matching pairs of input and output values, which must be provided
 * in strictly increasing order for the input values. The mapping is applied using linear
 * interpolation between the defined points.
 *
 * Input values outside the defined domain are clamped to the nearest bound.
 */
public class ThrottleMap {
  private final PolynomialSplineFunction throttleCurve;
  private final double[] inputValues;
  private final double[] outputValues;

  /**
   * Constructs a ThrottleMap using the specified input-output mapping curve.
   * The arrays must be of equal length and the input values must be strictly increasing.
   *
   * @param inputValues  Array of input domain values in ascending order.
   * @param outputValues Array of output range values corresponding to each input.
   */
  public ThrottleMap(double[] inputValues, double[] outputValues) {
    this.inputValues = inputValues;
    this.outputValues = outputValues;
    this.throttleCurve = createThrottleCurve();
  }

  /**
   * Creates a piecewise linear spline using the defined input and output arrays.
   * Each segment between two consecutive input values is represented as a first-degree polynomial.
   *
   * @return A {@link PolynomialSplineFunction} representing the throttle curve.
   */
  private PolynomialSplineFunction createThrottleCurve() {
    PolynomialFunction[] polynomials = new PolynomialFunction[inputValues.length - 1];

    for (int i = 0; i < polynomials.length; i++) {
      double a = outputValues[i];
      double b = (outputValues[i + 1] - outputValues[i]) / (inputValues[i + 1] - inputValues[i]);
      polynomials[i] = new PolynomialFunction(new double[] {a, b});
    }

    return new PolynomialSplineFunction(inputValues, polynomials);
  }

  /**
   * Applies the throttle mapping to a non-negative input value.
   * Values outside the domain are clamped to the nearest valid input.
   *
   * @param input The input value to be mapped.
   * @return The mapped output value.
   */
  public double applyThrottle(double input) {
    double clamped = MathUtil.clamp(input, inputValues[0], inputValues[inputValues.length - 1]);
    return throttleCurve.value(clamped);
  }

  /**
   * Applies the throttle mapping to a signed input value.
   * The absolute value is mapped using the throttle curve,
   * and the original sign of the input is preserved in the result.
   *
   * @param input The signed input value.
   * @return The signed, mapped output value.
   */
  public double applyThrottleAbs(double input) {
    double magnitude = Math.abs(input);
    double clamped = MathUtil.clamp(magnitude, inputValues[0], inputValues[inputValues.length - 1]);
    return Math.copySign(throttleCurve.value(clamped), input);
  }
}
