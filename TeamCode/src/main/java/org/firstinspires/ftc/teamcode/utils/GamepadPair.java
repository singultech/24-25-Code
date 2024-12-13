package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadPair {
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final Gamepad previousGamepad1;
    private final Gamepad previousGamepad2;
    private boolean secondControllerEnabled = true;

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

    public boolean isPressed(int gamepadNum, String button) {

        if (gamepadNum == 2 && !secondControllerEnabled) return false;
        if (gamepadNum == 1 || gamepadNum == 2){
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
        if (!secondControllerEnabled){
            Gamepad currentGamepad = gamepad1;
            Gamepad previousGamepad = previousGamepad1;
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
        switch (button) {
            case "a":
            case "cross":
                return (gamepad1.a && !previousGamepad1.a) || (gamepad2.a && !previousGamepad2.a);
            case "b":
            case "circle":
                return (gamepad1.b && !previousGamepad1.b) || (gamepad2.b && !previousGamepad2.b);
            case "x":
            case "square":
                return (gamepad1.x && !previousGamepad1.x) || (gamepad2.x && !previousGamepad2.x);
            case "y":
            case "triangle":
                return (gamepad1.y && !previousGamepad1.y) || (gamepad2.y && !previousGamepad2.y);
            case "dpad_up":
                return (gamepad1.dpad_up && !previousGamepad1.dpad_up) || (gamepad2.dpad_up && !previousGamepad2.dpad_up);
            case "dpad_down":
                return (gamepad1.dpad_down && !previousGamepad1.dpad_down) || (gamepad2.dpad_down && !previousGamepad2.dpad_down);
            case "dpad_left":
                return (gamepad1.dpad_left && !previousGamepad1.dpad_left) || (gamepad2.dpad_left && !previousGamepad2.dpad_left);
            case "dpad_right":
                return (gamepad1.dpad_right && !previousGamepad1.dpad_right) || (gamepad2.dpad_right && !previousGamepad2.dpad_right);
            case "left_bumper":
                return (gamepad1.left_bumper && !previousGamepad1.left_bumper) || (gamepad2.left_bumper && !previousGamepad2.left_bumper);
            case "right_bumper":
                return (gamepad1.right_bumper && !previousGamepad1.right_bumper) || (gamepad2.right_bumper && !previousGamepad2.right_bumper);
            case "left_stick_button":
                return (gamepad1.left_stick_button && !previousGamepad1.left_stick_button) || (gamepad2.left_stick_button && !previousGamepad2.left_stick_button);
            case "right_stick_button":
                return (gamepad1.right_stick_button && !previousGamepad1.right_stick_button) || (gamepad2.right_stick_button && !previousGamepad2.right_stick_button);
            default:
                throw new IllegalArgumentException("Unknown button: " + button);
        }
    }

    public boolean isHeld(int gamepadNum, String button) {
        Gamepad currentGamepad = gamepadNum == 1 ? gamepad1 : gamepad2;
        if (gamepadNum == 2 && !secondControllerEnabled) return false;
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
        if (gamepadNum == 2 && !secondControllerEnabled) return 0;
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

    public void setSecondControllerState(boolean state){
        secondControllerEnabled = state;
    }

    public boolean getSecondControllerState(){
        return secondControllerEnabled;
    }
}
