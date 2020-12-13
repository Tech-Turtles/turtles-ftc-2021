package org.firstinspires.ftc.teamcode.Opmodes.Driving;

public class DiagnosticVision extends Diagnostic {

    @Override
    public void init() {
        super.init();
        new Thread(() -> loadVision(true)).start();
        telemetry.addLine("Diagnostic Vision OpMode Initialized");
    }

    @Override
    public void init_loop(){
        super.init_loop();
        if (ringDetector == null)
            telemetry.addData("Vision: ", "LOADING...");
        else
            telemetry.addData("Vision: ", "INITIALIZED");
    }

    @Override
    public void loop() {
        super.loop();

        if (ringDetector != null)
            telemetry.addData("Ring Amount: ", ringDetector.getHeight().name());
    }
}
