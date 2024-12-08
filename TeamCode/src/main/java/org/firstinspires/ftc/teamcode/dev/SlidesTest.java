/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package org.firstinspires.ftc.teamcode.dev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.SlidePair;

@TeleOp(name = "Slides Test", group = "Dev")
public class SlidesTest extends LinearOpMode {
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    @Override
    public void runOpMode() {

        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        SlidePair slides = new SlidePair(leftSlide, rightSlide, 2000, 0.25);


        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.square) {
                slides.resetPosition();
            }
            if (gamepad1.triangle){
                if (slides.isActive()){
                    slides.setPower(0);
                } else{
                    slides.setPower(0.25);
                }
            }
            if (gamepad1.dpad_up){
                slides.changeTargetPosition(1);
            }
            if (gamepad1.dpad_down){
                slides.changeTargetPosition(-1);
            }
            telemetry.addLine("Use the D-pad to control the slides.");
            telemetry.addLine("Press ▣ to reset the slides position to 0.");
            telemetry.addLine("Press ▲ to toggle the holding motors");
            telemetry.addLine("Current Left Position " + slides.getLeftPosition());
            telemetry.addLine("Current Right Position " + slides.getRightPosition());
            telemetry.addLine("Target Position " + slides.getTargetPosition());
            telemetry.update();
        }
    }
}


