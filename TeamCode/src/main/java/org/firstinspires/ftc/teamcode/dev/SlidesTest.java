/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package org.firstinspires.ftc.teamcode.dev;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Slides Test", group = "Dev")
public class SlidesTest extends LinearOpMode {
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    int currentPosition;
    boolean isActive;
    final int maxH = 10000;
    @Override
    public void runOpMode() {

        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        setSlidePositions(1);
        isActive = true;
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.square) {
                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (gamepad1.triangle){
                if (isActive){
                    setSlidePositions(0);
                    isActive = false;
                } else{
                    setSlidePositions(1);
                    isActive = true;
                }
            }
            if (gamepad1.dpad_up && currentPosition<maxH){
                currentPosition += 1;
            }
            if (gamepad1.dpad_down && currentPosition>=0){
                currentPosition -= 1;
            }
            setSlidePositions(0);
            telemetry.addLine("Use the D-pad to control the slides.");
            telemetry.addLine("Press ▣ to reset the slides position to 0.");
            telemetry.addLine("Press ▲ to toggle the holding motors");
            telemetry.addLine("Current Left Position " + leftSlide.getCurrentPosition());
            telemetry.addLine("Current Right Position " + rightSlide.getCurrentPosition());
            telemetry.addLine("Target Position " + currentPosition);
            telemetry.update();
        }
    }
    public void setSlidePositions(int pos){
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
    }
}


