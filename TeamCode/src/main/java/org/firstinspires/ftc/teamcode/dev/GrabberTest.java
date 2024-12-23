/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.utils.FrontGrabber;
import org.firstinspires.ftc.teamcode.utils.GamepadPair;

@TeleOp(name = "Front Grabber Test", group = "Dev")
public class GrabberTest extends LinearOpMode {
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Servo grabberServo = hardwareMap.servo.get("frontGrabberServo");
        TouchSensor limitSwitch = hardwareMap.touchSensor.get("frontGrabberSwitch");
        FrontGrabber grabber = new FrontGrabber(0.72, 1, hardwareMap);
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        long curTime;
        long lastOpened = 0;

        waitForStart();

        while (opModeIsActive()) {
            curTime = System.currentTimeMillis() / 1000L;
            gamepads.copyStates();

            if (gamepads.isPressed(-1, "cross")) {
                if (grabber.isClosed()) {grabber.open(); lastOpened = curTime;}
                else grabber.close();
            }
            if (grabber.getSwitchState() && curTime - lastOpened > 3){
                grabber.close();
                gamepads.rumble(-1, 250);

            }

            telemetry.addLine("Press X to open or close the grabber");
            telemetry.addLine(grabber.isClosed() ? "Grabber closed" : "Grabber opened");
            telemetry.addLine(grabber.getSwitchState() ? "Switch pressed" : "Switch not pressed");
            telemetry.update();
        }
    }
}

