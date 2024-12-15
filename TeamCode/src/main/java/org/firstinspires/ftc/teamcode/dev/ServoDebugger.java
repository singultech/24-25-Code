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

import org.firstinspires.ftc.teamcode.utils.SlidePair;

@TeleOp(name = "Servo Test", group = "Dev")
public class ServoDebugger extends LinearOpMode {
    Servo rightFlip;
    Servo leftFlip;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Servo servo = hardwareMap.servo.get("testservo");
        double targetPosition = 0;


        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                targetPosition += 0.05;
            }
            if (gamepad1.dpad_down) {
                targetPosition -= 0.05;
            }
            if (gamepad1.square){
                targetPosition = 1;
            }
            if (gamepad1.triangle){
                targetPosition = 0;
            }
            servo.setPosition(targetPosition);
            telemetry.addLine("Target Position " + targetPosition);
            telemetry.addLine("Current Position " + servo.getPosition());
            telemetry.update();
        }
    }
}


