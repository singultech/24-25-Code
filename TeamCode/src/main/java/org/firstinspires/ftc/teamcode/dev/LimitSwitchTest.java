/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package org.firstinspires.ftc.teamcode.dev;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Limit Switch Test", group = "Dev")
public class LimitSwitchTest extends LinearOpMode {
    TouchSensor limitSwitch;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limitSwitch = hardwareMap.touchSensor.get("frontGrabberLimit");


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Switch State: " + limitSwitch.isPressed());
            telemetry.update();
        }
    }
}


