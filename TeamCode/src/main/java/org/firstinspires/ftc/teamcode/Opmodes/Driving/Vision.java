package org.firstinspires.ftc.teamcode.Opmodes.Driving;

public class Vision extends Manual {

    @Override
    public void init() {
        super.init();
        new Thread(() -> loadVision(false)).start();
        telemetry.addLine("Vision OpMode Initialized");
    }

    @Override
    public void init_loop(){
        super.init_loop();
        if (ringDetector == null)
            telemetry.addData("Vision:", "LOADING...");
        else
            telemetry.addData("Vision:", "INITIALIZED");
    }

    @Override
    public void loop() {
        super.loop();

        if (ringDetector != null)
            telemetry.addData("Ring Amount: ", ringDetector.getHeight().name());
    }
}
