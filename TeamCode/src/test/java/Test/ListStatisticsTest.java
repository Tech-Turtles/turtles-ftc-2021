package Test;


import org.apache.commons.math3.stat.regression.SimpleRegression;
import org.firstinspires.ftc.teamcode.Utility.Math.ListMath;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.junit.Before;
import org.junit.Test;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ListStatisticsTest {

    //String IMAGE_READ_PATH = "./TestData/openCV_input/skystone/";
    //String IMAGE_WRITE_PATH = "./TestData/openCV_output/skystone/";


    ArrayList<Double> dataList = new ArrayList();
    ArrayList<Double> timeList = new ArrayList();

    @Before
    public void initialize() {
        int maxSize = 100;
        // Base data
        for (int i = 0; i <= 31; ++i) {
            dataList.add((double) i);
            //dataList.add((double) Math.random());
            if (i == 0)
                timeList.add(0.0);
            else
                timeList.add(timeList.get(timeList.size() - 1) + 0.02);
        }
        // Data noise
        for (int i = 0; i < dataList.size(); ++i) {
            dataList.set(i, dataList.get(i) + 10.0*Math.random());
        }
    }

    @Test
    public void addSequenceToArray() {
        ArrayList<Double> list = new ArrayList<>();
        System.out.println("Add Sequence to Array");
        int maxSize = 100;

        for (int i = 1; i <= 5; ++i) {
            ListMath.addRemoveN(list, i, maxSize);
        }

        System.out.println(list.toString());
        System.out.println("Size: " + list.size());
        System.out.println("Mean: " + ListMath.average(list));
        // should just be size, but -1 offset in there somewhere
        System.out.println("Mean, size: " + ListMath.average(list, list.size()));
        System.out.println("Mean, size -1: " + ListMath.average(list, list.size() - 1));
        System.out.println("Mean, 1: " + ListMath.average(list, 1));
        System.out.println("Mean, 0: " + ListMath.average(list, 0));
        System.out.println("Mean, 10: " + ListMath.average(list, 10));
        System.out.println("StdDev: " + ListMath.standardDeviation(list));
        // should just be size, but -1 offset in there somewhere
        System.out.println("StdDev, size: " + ListMath.standardDeviation(list, list.size()));
        System.out.println("StdDev, size -1: " + ListMath.standardDeviation(list, list.size() - 1));
        System.out.println("StdDev, 1: " + ListMath.standardDeviation(list, 1));
        System.out.println("StdDev, 0: " + ListMath.standardDeviation(list, 0));
        System.out.println("StdDev, 10: " + ListMath.standardDeviation(list, 10));

    }


    /**
     * Test Apache Commons Math3 regression tools
     * @see <a href="https://commons.apache.org/proper/commons-math/javadocs/api-3.3/org/apache/commons/math3/stat/regression/SimpleRegression.html">SimpleRegression JavaDoc</a>
     * @see <a href="https://www.bmj.com/about-bmj/resources-readers/publications/statistics-square-one/11-correlation-and-regression">Regression Reference</a>
     */
    @Test
    public void linearRegression() {
        System.out.println("Regression Test");
        // One liner to get linear regression.
        SimpleRegression simpleRegression = ListMath.getLinearRegression(timeList,dataList,50);
        printRegression(simpleRegression);
    }


    private void printRegression(SimpleRegression simpleRegression) {
        DecimalFormat df = RobotHardware.df;
        System.out.println(
                "Y = " + df.format(simpleRegression.getSlope()) + " X   +   "  +
                 df.format(simpleRegression.getIntercept())  + "   ,  Error: " +
                        df.format(simpleRegression.getMeanSquareError()));
        //System.out.println(df.format(simpleRegression.getSlope()));
        //System.out.println(df.format(simpleRegression.getIntercept()));
        //System.out.println(df.format(simpleRegression.getMeanSquareError()));
    }

}
