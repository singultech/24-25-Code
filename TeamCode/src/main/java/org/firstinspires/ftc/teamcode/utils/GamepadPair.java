package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadPair {
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final Gamepad previousGamepad1;
    private final Gamepad previousGamepad2;

    public GamepadPair(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.previousGamepad1 = new Gamepad();
        this.previousGamepad2 = new Gamepad();
    }

    public void copyStates() {
        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }

    public boolean isPressedOnce(int gamepadNum, String button) {
        Gamepad currentGamepad = gamepadNum == 1 ? gamepad1 : gamepad2;
        Gamepad previousGamepad = gamepadNum == 1 ? previousGamepad1 : previousGamepad2;
        switch (button) {
            case "a":
            case "cross":
                return currentGamepad.a && !previousGamepad.a;
            case "b":
            case "circle":
                return currentGamepad.b && !previousGamepad.b;
            case "x":
            case "square":
                return currentGamepad.x && !previousGamepad.x;
            case "y":
            case "triangle":
                return currentGamepad.y && !previousGamepad.y;
            case "dpad_up":
                return currentGamepad.dpad_up && !previousGamepad.dpad_up;
            case "dpad_down":
                return currentGamepad.dpad_down && !previousGamepad.dpad_down;
            case "dpad_left":
                return currentGamepad.dpad_left && !previousGamepad.dpad_left;
            case "dpad_right":
                return currentGamepad.dpad_right && !previousGamepad.dpad_right;
            case "left_bumper":
                return currentGamepad.left_bumper && !previousGamepad.left_bumper;
            case "right_bumper":
                return currentGamepad.right_bumper && !previousGamepad.right_bumper;
            case "left_stick_button":
                return currentGamepad.left_stick_button && !previousGamepad.left_stick_button;
            case "right_stick_button":
                return currentGamepad.right_stick_button && !previousGamepad.right_stick_button;
            default:
                throw new IllegalArgumentException("Unknown button: " + button);
        }
    }

    public boolean isHeld(int gamepadNum, String button) {
        Gamepad currentGamepad = gamepadNum == 1 ? gamepad1 : gamepad2;
        switch (button) {
            case "a":
            case "cross":
                return currentGamepad.a;
            case "b":
            case "circle":
                return currentGamepad.b;
            case "x":
            case "square":
                return currentGamepad.x;
            case "y":
            case "triangle":
                return currentGamepad.y;
            case "dpad_up":
                return currentGamepad.dpad_up;
            case "dpad_down":
                return currentGamepad.dpad_down;
            case "dpad_left":
                return currentGamepad.dpad_left;
            case "dpad_right":
                return currentGamepad.dpad_right;
            case "left_bumper":
                return currentGamepad.left_bumper;
            case "right_bumper":
                return currentGamepad.right_bumper;
            case "left_stick_button":
                return currentGamepad.left_stick_button;
            case "right_stick_button":
                return currentGamepad.right_stick_button;
            default:
                throw new IllegalArgumentException("Unknown button: " + button);
        }
    }

    public float joystickValue(int gamepadNum, String joystick, String direction) {
        Gamepad currentGamepad = gamepadNum == 1 ? gamepad1 : gamepad2;
        switch (joystick) {
            case "left":
                if ("x".equals(direction)) return currentGamepad.left_stick_x;
                if ("y".equals(direction)) return currentGamepad.left_stick_y;
                break;
            case "right":
                if ("x".equals(direction)) return currentGamepad.right_stick_x;
                if ("y".equals(direction)) return currentGamepad.right_stick_y;
                break;
        }
        throw new IllegalArgumentException("Unknown joystick/direction combination: " + joystick + "/" + direction);
    }
}
