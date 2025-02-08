package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.GamepadPair;

@TeleOp(name = "Servo Controller", group = "Dev")
public class ManualServoControl extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        Servo servo = hardwareMap.servo.get("frontFlip");
        double targetPosition = 0;


        waitForStart();

        while (opModeIsActive()) {

            if (gamepads.isPressed(-1, "dpad_up")) {
                targetPosition += 0.05;
            }
            if (gamepads.isPressed(-1, "dpad_down")) {
                targetPosition -= 0.05;
            }
            if (gamepads.isPressed(-1, "dpad_right")){
                targetPosition = 1;
            }
            if (gamepads.isPressed(-1, "dpad_left")){
                targetPosition = 0;
            }
            servo.setPosition(targetPosition);
            gamepads.copyStates();
            telemetry.addLine("Vertical D-pad control to increment servo.");
            telemetry.addLine("Horizontal D-pad to set to 0 and max.");
            telemetry.addLine("Target Position " + targetPosition);
            telemetry.addLine("Current Position " + servo.getPosition());
            telemetry.update();
        }
    }
}

