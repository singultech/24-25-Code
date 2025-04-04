package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;

@TeleOp(name = "ServoTest")
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        Servo servo = hardwareMap.get(Servo.class, "frontArm");

        waitForStart();
        while (!isStopRequested()) {
            if(gamepads.isPressed("cross")){
                servo.setPosition(0.5);
            }
            if(gamepads.isPressed("triangle")){
                servo.setPosition(0.0);
            }

            telemetry.update();
        }
    }
}