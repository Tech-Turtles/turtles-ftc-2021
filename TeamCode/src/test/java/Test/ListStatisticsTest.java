package Test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.opencsv.CSVWriter;
import com.opencsv.bean.HeaderColumnNameMappingStrategy;
import com.opencsv.bean.StatefulBeanToCsv;
import com.opencsv.bean.StatefulBeanToCsvBuilder;
import com.opencsv.bean.comparator.LiteralComparator;
import com.opencsv.exceptions.CsvDataTypeMismatchException;
import com.opencsv.exceptions.CsvRequiredFieldEmptyException;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.TrajectoryRR;
import org.firstinspires.ftc.teamcode.Utility.Math.ListMath;
import org.firstinspires.ftc.teamcode.Utility.Roadrunner.util.TelemetryLog;
import org.junit.Before;
import org.junit.Test;
import org.yaml.snakeyaml.Yaml;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.StringWriter;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class ListStatisticsTest {

    //String IMAGE_READ_PATH = "./TestData/openCV_input/skystone/";
    //String IMAGE_WRITE_PATH = "./TestData/openCV_output/skystone/";

    @Before
    public void initialize() {
    }

    @Test
    public void addSequenceToArray() {
        ArrayList<Double> list = new ArrayList<>();
        System.out.println("Add Sequence to Array");
        int maxSize = 100;

        for(int i = 1; i <= 5 ; ++i) {
            ListMath.addRemoveN(list, i, maxSize);
        }

        System.out.println(list.toString());
        System.out.println("Size: " + list.size());
        System.out.println("Mean: " + ListMath.average(list));
        // should just be size, but -1 offset in there somewhere
        System.out.println("Mean, size: " + ListMath.average(list,list.size()));
        System.out.println("Mean, size -1: " + ListMath.average(list,list.size()-1));
        System.out.println("Mean, 1: " + ListMath.average(list,1));
        System.out.println("Mean, 0: " + ListMath.average(list,0));
        System.out.println("Mean, 10: " + ListMath.average(list,10));
        System.out.println("StdDev: " + ListMath.standardDeviation(list));
        // should just be size, but -1 offset in there somewhere
        System.out.println("StdDev, size: " + ListMath.standardDeviation(list,list.size()));
        System.out.println("StdDev, size -1: " + ListMath.standardDeviation(list,list.size()-1));
        System.out.println("StdDev, 1: " + ListMath.standardDeviation(list,1));
        System.out.println("StdDev, 0: " + ListMath.standardDeviation(list,0));
        System.out.println("StdDev, 10: " + ListMath.standardDeviation(list,10));

    }

}
