package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.openftc.revextensions2.*;

import java.util.HashMap;

import org.firstinspires.ftc.teamcode.HardwareTypes.*;
import org.firstinspires.ftc.teamcode.Menu.InteractiveInitialization;

import static org.firstinspires.ftc.teamcode.Utility.Configuration.*;

/**
 * @author Christian
 * Revamped RobotHardware, better readability, new methods, more organized, easier to build upon.
 */
public class RobotHardware extends OpMode {

    private static final HashMap<Motors, ExpansionHubMotor> motors = new HashMap<>();
    private static final HashMap<Servos, Servo> servos = new HashMap<>();

    private ExpansionHubEx expansionHub1, expansionHub2;
    private RevBulkData bulkDataHub1, bulkDataHub2;

    public InteractiveInitialization initMenu;

    public final ElapsedTimer timer = new ElapsedTimer();

    public class MotorUtility {

        private ExpansionHubMotor m;

        private ExpansionHubMotor getMotor(Motors motor) {
            m = motors.get(motor);
            if(m == null)
                telemetry.addData("Motor Missing", motor.name());
            return m;
        }

        public double getPower(Motors motor) {
            m = getMotor(motor);
            if (m != null)
                return m.getPower();
            return 0;
        }

        public void setPower(Motors motor, double power) {
            m = getMotor(motor);
            if (m != null)
                m.setPower(power);
        }

        public int getEncoderValue(Motors motor) {
            m = getMotor(motor);
            if(m == null) return -1;
            RevBulkData bulkData = null;
            for (Motors motorName : Motors.values()) {
                switch(motorName.getExpansionHub()) {
                    case HUB1:
                        bulkData = bulkDataHub1;
                        break;
                    case HUB2:
                        bulkData = bulkDataHub2;
                }
                if(bulkData == null) continue;
                if(motor.name().equals(motorName.name()))
                    return bulkData.getMotorCurrentPosition(m);
            }
            telemetry.addData("Not using bulk reads", motor.name());
            return m.getCurrentPosition();
        }

        public void stopAllMotors() {
            for(Motors motor : Motors.values())
                setPower(motor, 0f);
        }

        public void stopResetAllMotors() {
            motors.forEach((k, v) -> v.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        }

        public void stopResetMotor(Motors motor) {
            m = getMotor(motor);
            if(m != null)
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void setTypeMotorsRunmode(MotorTypes type, DcMotor.RunMode runMode) {
            for(Motors motor : Motors.values()) {
                if(!motor.getType().equals(type)) continue;
                m = getMotor(motor);
                if(m == null) continue;
                m.setMode(runMode);
            }
        }

        public void setMotorsRunMode(DcMotor.RunMode runMode, Motors... motors) {
            for(Motors motor : motors) {
                m = getMotor(motor);
                if(m == null) continue;
                m.setMode(runMode);
            }
        }

        public void setTypeMotorsZeroPowerBehavior(MotorTypes type, DcMotor.ZeroPowerBehavior behavior) {
            for(Motors motor : Motors.values()) {
                if(!motor.getType().equals(type)) continue;
                m = getMotor(motor);
                if(m == null) continue;
                m.setZeroPowerBehavior(behavior);
            }
        }

        public void setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior, Motors... motors) {
            for(Motors motor : motors) {
                m = getMotor(motor);
                if(m == null) continue;
                m.setZeroPowerBehavior(behavior);
            }
        }

        private void initializeMotors() {
            stopResetAllMotors();
            motors.forEach((k,v) -> {
                v.setMode(k.getRunMode());
                v.setZeroPowerBehavior(k.getZeroPowerBehavior());
                v.setDirection(k.getDirection());
            });
            for (Motors m : motors.keySet()) {

            }
        }
    }

    public class ServoUtility {
        Servo s;

        private Servo getServo(Servos servo) {
            s = servos.get(servo);
            if(s == null) {
                telemetry.addData("Servo Missing", servo.name());
            }
            return s;
        }

        public void setAngle(Servos servo, double pos) {
            s = getServo(servo);
            if (s != null)
                s.setPosition(pos);
        }

        public double getAngle(Servos servo) {
            s = getServo(servo);
            if (s != null)
                return s.getPosition();
            return -1;
        }

        public boolean moveServoAtRate(Servos servo, double targetPos, double rate) {
            boolean isMovementDone = false;
            double distanceThreshold = 0.05;
            rate = Range.clip(rate,0,10);
            targetPos = Range.clip(targetPos,0,1);
            double currentPosition = getAngle(servo);
            double distance = targetPos - currentPosition;
            double direction = targetPos > currentPosition ? 1 : -1;
            double nextPosition;
            if ( Math.abs(distance) > distanceThreshold ) {
                nextPosition = rate * direction * timer.getLastPeriodSec() + currentPosition;
            } else {
                nextPosition = targetPos;
                isMovementDone = true;
            }
            nextPosition = Range.clip(nextPosition,0,1);
            setAngle(servo, nextPosition);

            return isMovementDone;
        }
    }

    @Override
    public void init() {

        // Need to test whether or not these vars can be changed post-init
        msStuckDetectInit = msStuckInit;
        msStuckDetectInitLoop = msStuckInitLoop;
        msStuckDetectStart = msStuckStart;
        msStuckDetectLoop = msStuckLoop;
        msStuckDetectStop = msStuckStop;

        try {
            expansionHub1 = hardwareMap.get(ExpansionHubEx.class, ExpansionHubs.HUB1.name());
            expansionHub2 = hardwareMap.get(ExpansionHubEx.class, ExpansionHubs.HUB2.name());
        } catch (IllegalArgumentException e) {
            telemetry.addData("Failed to find Expansion Hub", e.getMessage());
        }

        for(Motors m : Motors.values()) {
            try {
                motors.put(m, hardwareMap.get(ExpansionHubMotor.class, m.getConfigName()));
            } catch (IllegalArgumentException ignore) {
                telemetry.addData("Motor Missing", m.name());
            }
        }
        new MotorUtility().initializeMotors();

        for(Servos s : Servos.values()) {
            try {
                Servo servo = hardwareMap.get(Servo.class, s.getConfigName());
                servos.put(s, servo);
                servo.setDirection(s.getDirection());
            } catch (IllegalArgumentException ignore) {
                telemetry.addData("Servo Missing", s.name());
            }
        }

        initMenu = new InteractiveInitialization(this);
    }

    @Override
    public void init_loop() {
        super.init_loop();
        initMenu.loop();
    }

    @Override
    public void start() {
        super.start();
        timer.reset();
    }

    @Override
    public void loop() {
        timer.updatePeriodTime();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
