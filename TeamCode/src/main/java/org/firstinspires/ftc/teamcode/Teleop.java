package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.utils.GamepadPair;
import org.firstinspires.ftc.teamcode.utils.SlidePair;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        DcMotorEx rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SlidePair vertSlides = new SlidePair(leftSlide, rightSlide, 4100, 1);

        int[] vertSlidePresets = {0, 1000, 2000, 3000};
        int vertSlidePreset = 0;
        /*
        while (!isStarted()){
            if (gamepads.isHeld(-1, "triangle")) drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
            if (gamepads.isHeld(-1, "circle")) drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
            if (gamepads.isHeld(-1, "square")) drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
            if (gamepads.isHeld(-1, "cross")) drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
        }*/

        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepads.joystickValue(1, "left", "y"),
                            -gamepads.joystickValue(1, "left", "x")
                    ),
                    -gamepads.joystickValue(1, "right", "x")
            ));

            if (gamepads.isPressed(-1, "dpad_up") && vertSlidePreset+1 < vertSlidePresets.length) vertSlidePreset++;
            if (gamepads.isPressed(-1, "dpad_down") && vertSlidePreset>0) vertSlidePreset--;



            vertSlides.setTargetPosition(vertSlidePresets[vertSlidePreset]);
            drive.updatePoseEstimate();
            gamepads.copyStates();

            telemetry.addData("x: ", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("Slide preset", vertSlidePreset);
            telemetry.update();

        }
    }
}