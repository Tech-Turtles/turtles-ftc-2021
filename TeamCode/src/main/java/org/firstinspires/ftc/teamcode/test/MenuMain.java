package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utility.Controller;
import org.firstinspires.ftc.teamcode.Utility.Math.ElapsedTimer;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.test.MenuMode.*;

import java.util.ArrayList;

public class MenuMain {
    private Controller controller;
    private final Telemetry telemetry;
    private final RobotHardware robotHardware;
    private Gamepad gamepad;
    private MenuMode mode;
    private final ElapsedTimer timer = new ElapsedTimer();
    private final StringBuilder builder = new StringBuilder();

    private ArrayList<MenuCategory> categories = new ArrayList<>();
    private final double deadzone = 0.49f;
    private boolean locked;
    private int cursor;

    MenuMain(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
        this.gamepad = robotHardware.gamepad1 == null ? robotHardware.gamepad2 : robotHardware.gamepad1;
        this.telemetry = this.robotHardware.telemetry;
        this.controller = new Controller(gamepad);
        this.mode = MenuMode.CATEGORY;
        this.locked = false;
        this.cursor = 0;
    }

    public <T> MenuCategory<T> addCategory(String name, MenuOption<T>... args) {
        MenuCategory<T> object = new MenuCategory<>(name, args);
        categories.add(object);
        return object;
    }

    public void start() {

    }

    public void loop() {
        displayTelemetry();
        telemetry.update();
        updateInputs();
    }

    public void next() {
        cursor++;

    }

    public void previous() {
        cursor--;
    }

    public void select() {

    }

    public void nextOption() {

    }

    public void previousOption() {

    }

    public void lock() {

    }

    public void displayTelemetry() {

    }

    public void updateInputs() {
        if(controller == null) {
            return;
        }

        if(locked) {
            return;
        }

        switch (mode) {
            case CATEGORY:
                if(controller.left_stick_y < -deadzone) previous();
                else if(controller.left_stick_y > deadzone) next();
                else if(controller.rightStickButtonOnce()) select();
                else if(controller.right_stick_x < -deadzone) previousOption();
                else if(controller.right_stick_x > deadzone) nextOption();
            case OPTION:

                break;
        }

    }

    public int size() {
        return categories.size();
    }
}
