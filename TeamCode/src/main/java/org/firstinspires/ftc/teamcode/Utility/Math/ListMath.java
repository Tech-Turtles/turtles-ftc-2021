package org.firstinspires.ftc.teamcode.Utility.Math;

import org.apache.commons.math3.stat.regression.SimpleRegression;

import java.util.Arrays;
import java.util.List;


/**
 * Copied from Stephen's VectorMath
 */

public class ListMath
{
    // Summation
    static public double sum(List<Double> v)
    {
        double ret = 0.0;
        for (int i = 0; i < v.size(); ++i)
        {
            ret += v.get(i);
        }
        return ret;
    }

    // Get the average value of an entire List
    static public double average(List<Double> v)
    {
        if (v.size() > 0)
        {
            return sum(v) / v.size();
        }
        else
        {
            return 0.0;
        }
    }

    // Get the average over some past number of steps
    static public double average(List<Double> v, int steps)
    {
        int size = v.size();
        if (steps > size) {
            steps = size;
        }
        int start = size - steps;

        double ret = 0.0;
        for (int i = start; i < size; ++i)
        {
            ret += v.get(i);
        }

        //ret /= size - steps;
        ret /= steps;

        return ret;
    }

    // Get the standard deviation of entire List
    static public double standardDeviation(List<Double> v) {
        int size = v.size();
        double mean = average(v);
        double acc = 0.0;
        for (int i = 0; i < size; ++i) {
            acc += Math.pow(v.get(i) - mean,2);
        }
        double variance = acc / size;
        return Math.sqrt(variance);
    }

    // Get the standard deviation of some past number of steps
    static public double standardDeviation(List<Double> v, int steps) {
        int size = v.size();
        if (steps > size) {
            steps = size;
        }
        double mean = average(v,steps);
        double acc = 0.0;
        for (int i = size-steps; i < size; ++i) {
            acc += Math.pow(v.get(i) - mean,2);
        }
        double variance = acc / steps;
        return Math.sqrt(variance);
    }

    // Get the span of the List, i.e. lastValue - firstValue.
    static public double span(List<Double> v) {
        if (v.size() > 0)
        {
            return v.get(v.size()-1) - v.get(0);
        }
        else
        {
            return 0.0;
        }
    }

    public static void addRemove3(List<Double> v, double value) {
        v.add(value);
        while (v.size() > 3)
        {
            v.remove(v.get(0));
        }
    }

    public static void addRemoveN(List<Double> v, double value, double  maxSize) {
        v.add(value);
        while (v.size() > maxSize)
        {
            v.remove(v.get(0));
        }
    }


    // Derivative
    // x is the independent variable, y is dependent. Returns derivative for unequally spaced points. xest is the x value at which to evaluate the derivative.
    // x and y Lists don't need to be the same length if they have three elements or more each
    static public double derivative(List<Double> x, List<Double> y, double xest) {

        int nx = x.size();
        int ny = y.size();
        if (x.size() < 2 || y.size() < 2) {
            return 0.0;
        } else if (nx == 2 || ny == 2) {
            return (y.get(1) - y.get(0)) / (x.get(1) - x.get(0));
        } else {
            double x2 = x.get(nx - 1);
            double x1 = x.get(nx - 2);
            double x0 = x.get(nx - 3);

            double y2 = y.get(ny - 1);
            double y1 = y.get(ny - 2);
            double y0 = y.get(ny - 3);

            double dydx = y0 * (2.0 * xest - x1 - x2) / ((x0 - x1) * (x0 - x2)) + y1 * (2.0 * xest - x0 - x2) / ((x1 - x0) * (x1 - x2)) + y2 * (2.0 * xest - x0 - x1) / ((x2 - x0) * (x2 - x1));

            return dydx;
        }
    }

    // Default derivative computation at the latest x value
    static public double derivative(List<Double> x, List<Double> y) {

        return derivative(x, y, x.get(x.size()-1));
    }

    // x is the independent variable, y is dependent. Returns the integral estimate.
    static public double integral(List<Double> x, List<Double> y) {
        // Uses the composite trapezoidal rule to calculate the integral for unequally spaced data.
        if (x.size() != y.size()) {
            throw new RuntimeException("ERROR: Lists to be integrated are of differing size") ;
        }

        if (x.size() < 2)
            return 0.0;

        int n = x.size();
        int end = n - 1; // must end one step short because we are looking ahead one step in the loop
        double I = 0.0;

        for (int i = 0; i < end; ++i)
            I += (x.get(i + 1) - x.get(i)) * (y.get(i + i) + y.get(i)) / 2.0;

        return I;
    }


    /**
     * Convert List of Doubles to primitive array double[]
     * @param list<Double>
     * @return double[]
     */
    public static double[] toPrimitive(final List<Double> list) {
        final double[] out = new double[list.size()];
        Arrays.setAll(out, list::get);
        return out;
    }

    /**
     * Merge two equal length primative double arrays into a 2d primative double array.
     * @param x double[]
     * @param y double[]
     * @return double[][2]
     */
    public static double[][] merge2D(double[] x, double[] y) {
        if(x.length != y.length) {
            throw new RuntimeException("ERROR: x and y vectors must have same length") ;
        }
        final double[][] out = new double[x.length][2];
        for(int i = 0; i < x.length; ++i) {
            out[i][0] = x[i];
            out[i][1] = y[i];
        }
        return out;
    }

    /**
     * Calculate Linear regression slope, provides SimpleRegression object instance.
     * Test Apache Commons Math3 regression tools
     * @see <a href="https://commons.apache.org/proper/commons-math/javadocs/api-3.3/org/apache/commons/math3/stat/regression/SimpleRegression.html">SimpleRegression JavaDoc</a>
     * @see <a href="https://www.bmj.com/about-bmj/resources-readers/publications/statistics-square-one/11-correlation-and-regression">Regression Reference</a>
     * @param time List of Doubles
     * @param data List of Doubles
     * @return SimpleRegression object
     */
    public static SimpleRegression getLinearRegression(List<Double> time, List<Double> data) {
        SimpleRegression simpleRegression = new SimpleRegression();
        double[][] dataXY = merge2D(toPrimitive(time), toPrimitive(data));
        simpleRegression.addData(dataXY);
        return simpleRegression;
    }

    /**
     * Calculate Linear regression slope, provides SimpleRegression object instance.
     * Ignores all but the last 'numSteps' worth of data.
     * @param time List of Doubles
     * @param data List of Doubles
     * @param numSteps int, Use only the last numSteps in the calculation
     * @return SimpleRegression object
     */
    public static SimpleRegression getLinearRegression(List<Double> time, List<Double> data, int numSteps) {
        if(time.size() != data.size()) {
            throw new RuntimeException("ERROR: time and data vectors must have same length") ;
        }
        int size = time.size();
        if(numSteps > time.size())
            numSteps = time.size();
        int startIndex = time.size() - numSteps;
        SimpleRegression simpleRegression = new SimpleRegression();
        double[][] dataXY = merge2D(
                toPrimitive( time.subList(startIndex,size - 1)),
                toPrimitive( data.subList(startIndex,size - 1)));
        simpleRegression.addData(dataXY);
        return simpleRegression;
    }

}
