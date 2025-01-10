package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.GamepadPair;
import org.firstinspires.ftc.teamcode.utils.HorizSlidePair;

@TeleOp(name = "Horiz Slides Test", group = "Dev")
public class HorizontalSlidesTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HorizSlidePair slides = new HorizSlidePair(hardwareMap);
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);

        waitForStart();

        while (opModeIsActive()) {

            if(gamepads.getTrigger(1, "right_trigger")>0.1){slides.setPower(gamepads.getTrigger(1, "right_trigger"));}
            else if(gamepads.getTrigger(1, "left_trigger")>0.1){slides.setPower(-gamepads.getTrigger(1, "left_trigger"));}
            else slides.setPower(0);
            slides.update();

            telemetry.addLine("Use the triggers to control the slides");
            telemetry.addLine("Current Position " + slides.getCurrentPosition());
            telemetry.addLine("Full Rotation " + slides.getTotalRotation());
            telemetry.update();
        }
    }
}

