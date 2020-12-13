package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class TeleopManager {

    @TeleOp(name="Manual", group="A")
    class ManualTeleop extends Manual {

        @Override
        public void init() {
            super.init();
            clearHubCache();
        }

        @Override
        public void init_loop() {
            super.init_loop();
            clearHubCache();
        }

        @Override
        public void start() {
            super.start();
            clearHubCache();
        }

        @Override
        public void loop() {
            super.loop();
            clearHubCache();
        }

        @Override
        public void stop() {
            super.stop();
            clearHubCache();
        }
    }

    @TeleOp(name="Debug", group="B")
    class DebugTeleop extends Debug {

        @Override
        public void init() {
            super.init();
            clearHubCache();
        }

        @Override
        public void init_loop() {
            super.init_loop();
            clearHubCache();
        }

        @Override
        public void start() {
            super.start();
            clearHubCache();
        }

        @Override
        public void loop() {
            super.loop();
            clearHubCache();
        }

        @Override
        public void stop() {
            super.stop();
            clearHubCache();
        }
    }

    @TeleOp(name="Diagnostic", group="C")
    class DiagnosticTeleop extends Diagnostic {

        @Override
        public void init() {
            super.init();
            clearHubCache();
        }

        @Override
        public void init_loop() {
            super.init_loop();
            clearHubCache();
        }

        @Override
        public void start() {
            super.start();
            clearHubCache();
        }

        @Override
        public void loop() {
            super.loop();
            clearHubCache();
        }

        @Override
        public void stop() {
            super.stop();
            clearHubCache();
        }
    }

    @TeleOp(name="Vision", group="B")
    class VisionTeleop extends Vision {

        @Override
        public void init() {
            super.init();
            clearHubCache();
        }

        @Override
        public void init_loop() {
            super.init_loop();
            clearHubCache();
        }

        @Override
        public void start() {
            super.start();
            clearHubCache();
        }

        @Override
        public void loop() {
            super.loop();
            clearHubCache();
        }

        @Override
        public void stop() {
            super.stop();
            clearHubCache();
        }
    }

    @TeleOp(name="Diagnostic Vision", group="C")
    class DiagnosticVisionTeleop extends DiagnosticVision {

        @Override
        public void init() {
            super.init();
            clearHubCache();
        }

        @Override
        public void init_loop() {
            super.init_loop();
            clearHubCache();
        }

        @Override
        public void start() {
            super.start();
            clearHubCache();
        }

        @Override
        public void loop() {
            super.loop();
            clearHubCache();
        }

        @Override
        public void stop() {
            super.stop();
            clearHubCache();
        }
    }

    @TeleOp(name="Tank Drive", group="E")
    class TankTeleop extends Tank {

        @Override
        public void init() {
            super.init();
            clearHubCache();
        }

        @Override
        public void init_loop() {
            super.init_loop();
            clearHubCache();
        }

        @Override
        public void start() {
            super.start();
            clearHubCache();
        }

        @Override
        public void loop() {
            super.loop();
            clearHubCache();
        }

        @Override
        public void stop() {
            super.stop();
            clearHubCache();
        }
    }
}
