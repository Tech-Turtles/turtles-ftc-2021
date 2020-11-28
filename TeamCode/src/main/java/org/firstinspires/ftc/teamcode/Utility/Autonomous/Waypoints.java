package org.firstinspires.ftc.teamcode.Utility.Autonomous;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Reader;
import java.io.StringWriter;
import java.io.Writer;

public class Waypoints {
    JSONObject waypoints;
    RobotHardware opmode;

    public Waypoints(RobotHardware opmode) {
        this.opmode = opmode;
    }

    public void test() {
        try {
            waypoints = new JSONObject(getFileContents(R.raw.waypoints));
        } catch(JSONException ignore) {}
    }

    public String getFileContents(int file) {
        InputStream is = opmode.hardwareMap.appContext.getResources().openRawResource(file);
        Writer writer = new StringWriter();
        char[] buffer = new char[1024];
        try {
            Reader reader = new BufferedReader(new InputStreamReader(is, "UTF-8"));
            int n;
            while ((n = reader.read(buffer)) != -1) {
                writer.write(buffer, 0, n);
            }
            is.close();
        } catch(IOException ignore) {}

        return writer.toString();
    }
}
