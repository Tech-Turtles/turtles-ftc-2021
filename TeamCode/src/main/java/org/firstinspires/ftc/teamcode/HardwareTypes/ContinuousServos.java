package org.firstinspires.ftc.teamcode.HardwareTypes;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public enum ContinuousServos {

    WOBBLE_LEFT("wobble_left", DcMotorSimple.Direction.FORWARD, ServoTypes.LINKED),
    WOBBLE_right("wobble_right", DcMotorSimple.Direction.REVERSE, ServoTypes.LINKED);

    private final String configName;
    private final DcMotorSimple.Direction direction;
    private final ServoTypes type;

    ContinuousServos(String configName, DcMotorSimple.Direction direction, ServoTypes type) {
        this.configName = configName;
        this.direction = direction;
        this.type = type;
    }

    public String getConfigName() {
        return configName;
    }

    public DcMotorSimple.Direction getDirection() {
        return direction;
    }

    public ServoTypes getType() {
        return type;
    }
}
