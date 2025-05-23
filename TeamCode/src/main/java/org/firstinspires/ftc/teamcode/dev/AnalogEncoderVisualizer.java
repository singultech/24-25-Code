package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

//@Disabled
@TeleOp(name = "Encoder Visualizer", group = "Dev")
public class AnalogEncoderVisualizer extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        AnalogInput leftDiffyEncoder = hardwareMap.get(AnalogInput.class, "leftDiffyEncoder");
        AnalogInput rightDiffyEncoder = hardwareMap.get(AnalogInput.class, "rightDiffyEncoder");
        AnalogInput rightArmEncoder = hardwareMap.get(AnalogInput.class, "rightArmEncoder");
        AnalogInput rightSlideEncoder = hardwareMap.get(AnalogInput.class, "rightHorizSlideEncoder");
        CRServo rightFlip = hardwareMap.crservo.get("rightHorizSlide");
        waitForStart();

        while (opModeIsActive()) {
            rightFlip.setPower(0.1);

            telemetry.addData("LeftDiffy:", leftDiffyEncoder.getVoltage());
            telemetry.addData("RightDiffy:", rightDiffyEncoder.getVoltage());
            telemetry.addData("RightArm:", rightArmEncoder.getVoltage());
            telemetry.addData("RightSlide:", rightSlideEncoder.getVoltage());
            telemetry.update();
        }
    }
}
