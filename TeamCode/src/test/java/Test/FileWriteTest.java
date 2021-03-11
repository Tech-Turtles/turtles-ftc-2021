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

import static com.google.common.truth.Truth.assertThat;

public class FileWriteTest {

    // Load x64 OpenCV Library dll

    String IMAGE_READ_PATH = "./TestData/openCV_input/skystone/";
    String IMAGE_WRITE_PATH = "./TestData/openCV_output/skystone/";

    @Before
    public void initialize() {
    }



    @Test
    public void cornerTest() {
        System.out.println(TrajectoryRR.getNearestCornerPose2d(new Pose2d(-60.0, 82.0, Math.toRadians(-100.0))));
}

    @Test
    public void yamlWriteTest() {

        //        EnumMap<Motors,Integer> motorEncoders = new EnumMap<Motors, Integer>(Motors.class){{
//            put(Motors.FRONT_RIGHT,150);
//            put(Motors.FRONT_LEFT,100);
//            put(Motors.BACK_RIGHT,200);
//            put(Motors.BACK_LEFT,100);
//        }};

        TelemetryLog testLog = new TelemetryLog(1.5, new Pose2d(10, 17, 0.0),
                new EnumMap<Motors, Integer>(Motors.class){{
                    put(Motors.FRONT_RIGHT,150);
                    put(Motors.FRONT_LEFT,100);
                    put(Motors.BACK_RIGHT,200);
                    put(Motors.BACK_LEFT,100);
                }});

        Map<String, Object> data = new LinkedHashMap<String, Object>();
        data.put("name", "Silenthand Olleander");
        data.put("race", "Human");
        data.put("traits", new String[] { "ONE_HAND", "ONE_EYE" });

        Yaml yaml = new Yaml();
        StringWriter writer = new StringWriter();
//        yaml.dump(data, writer);
        yaml.dump(testLog, writer);
        String expectedYaml = "name: Silenthand Olleander\nrace: Human\ntraits: [ONE_HAND, ONE_EYE]\n";

        String yamlString = writer.toString();
        System.out.println(yamlString);
        //assertEquals(expectedYaml, yamlString);

        String writePath = "./TestData/openCV_output/skystone/" + "test.yaml";
        File file = new File(writePath);
        File directory = new File(file.getParent());
        if(!directory.exists()) {
            directory.mkdir();
        }

        try {
            FileWriter fw = new FileWriter(file);
            fw.write(yamlString);
            fw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }



    }


    @Test
    public void csvWriteTest() {
        TelemetryLog testLog = new TelemetryLog(1.5, new Pose2d(10, 17, 0.0),
                new EnumMap<Motors, Integer>(Motors.class){{
                    put(Motors.FRONT_RIGHT,150);
                    put(Motors.FRONT_LEFT,100);
                    put(Motors.BACK_RIGHT,200);
                    put(Motors.BACK_LEFT,100);
                }});

        List<TelemetryLog> telemetryLogList = new ArrayList();
        telemetryLogList.add(testLog);
        telemetryLogList.add(new TelemetryLog(11.5, new Pose2d(11, 18, Math.toRadians(90.0)),
                new EnumMap<Motors, Integer>(Motors.class){{
                    put(Motors.FRONT_RIGHT,1500);
                    put(Motors.FRONT_LEFT,1000);
                    put(Motors.BACK_RIGHT,2000);
                    put(Motors.BACK_LEFT,1000);
                }}));

        // Path and file creation
        String writePath = "./TestData/openCV_output/skystone/" + "test.csv";
        File file = new File(writePath);
        File directory = new File(file.getParent());
        if(!directory.exists()) {
            directory.mkdir();
        }

        // Create FileWriter from File object
        try {
            FileWriter fw = new FileWriter(file);
            HeaderColumnNameMappingStrategy<TelemetryLog> strategy = new HeaderColumnNameMappingStrategy<>();
            strategy.setType(TelemetryLog.class);
            strategy.setColumnOrderOnWrite(new LiteralComparator<>(TelemetryLog.fieldOrder));

            StatefulBeanToCsv sbc = new StatefulBeanToCsvBuilder(fw)
                    .withSeparator(CSVWriter.DEFAULT_SEPARATOR)
                    .withMappingStrategy(strategy)
                    .build();
            sbc.write(telemetryLogList);
            fw.close();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (CsvRequiredFieldEmptyException e) {
            e.printStackTrace();
        } catch (CsvDataTypeMismatchException e) {
            e.printStackTrace();
        }

    }


}
