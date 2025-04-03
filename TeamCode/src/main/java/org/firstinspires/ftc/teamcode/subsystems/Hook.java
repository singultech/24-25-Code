package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

public class Hook {
    private final Servo hookServo;
    private HookPosition hookPosition;

    public enum HookPosition {
        UP(0.5),
        DOWN(0.0),
        STANDBY(0.1);

        private final double position;

        HookPosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public Hook(Servo hookServo) {
        this(hookServo, HookPosition.STANDBY);
    }

    public Hook(Servo hookServo, HookPosition position) {
        this.hookServo = hookServo;
        setPosition(position);
    }

    public void setPosition(HookPosition position) {
        hookPosition = position;
        hookServo.setPosition(position.getPosition());
    }

    public HookPosition getPosition() {
        return hookPosition;
    }

    @NonNull
    public String toString(){
        return hookPosition.toString();
    }
    @TeleOp(name = "Hook Test", group = "Dev")
    public static class HookTest extends LinearOpMode {

        @Override
        public void runOpMode() throws InterruptedException {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            Servo rightServo = hardwareMap.get(Servo.class, "rightHook");
            rightServo.setDirection(Servo.Direction.REVERSE);
            Hook leftHook = new Hook(hardwareMap.get(Servo.class, "leftHook"));
            Hook rightHook = new Hook(rightServo);
            GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
            waitForStart();
            while (opModeIsActive()) {
                if (gamepads.isPressed("dpad_up")) {
                    leftHook.setPosition(Hook.HookPosition.UP);
                    rightHook.setPosition(Hook.HookPosition.UP);
                }
                if (gamepads.isPressed("dpad_down")) {
                    leftHook.setPosition(Hook.HookPosition.DOWN);
                    rightHook.setPosition(Hook.HookPosition.DOWN);
                }
                if (gamepads.isPressed("dpad_right") || gamepads.isPressed("dpad_left")) {
                    leftHook.setPosition(Hook.HookPosition.STANDBY);
                    rightHook.setPosition(Hook.HookPosition.STANDBY);
                }
                telemetry.addLine("DPAD UP/DOWN to move hook up or down");
                telemetry.addLine("DPAD RIGHT/LEFT to put the hooks in standby position");
                telemetry.addLine("Left Hook " + leftHook);
                telemetry.addLine("Right Hook " + rightHook);
                telemetry.update();
            }

        }
    }
}
