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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.Arm;
import org.firstinspires.ftc.teamcode.utils.FrontGrabber;
import org.firstinspires.ftc.teamcode.utils.GamepadPair;
import org.firstinspires.ftc.teamcode.utils.SlidePair;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        DcMotorEx rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        long curTime;
        long lastFrontOpened = 0;

        Servo grabberServo = hardwareMap.servo.get("frontGrabberServo");
        TouchSensor limitSwitch = hardwareMap.touchSensor.get("frontGrabberSwitch");
        Servo rServo = hardwareMap.servo.get("rightFlip");
        FrontGrabber grabber = new FrontGrabber(0.73, 1, grabberServo, limitSwitch);
        SlidePair vertSlides = new SlidePair(leftSlide, rightSlide, 4100, 1);

        Arm arm = new Arm(0.55, 0, rServo);

        int[] vertSlidePresets = {0, 300, 894, 2478, 2800, 3500, 4100};
        int vertSlidePreset = 0;
        grabber.close();
        /*
        while (!isStarted()){
            if (gamepads.isHeld(-1, "triangle")) drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
            if (gamepads.isHeld(-1, "circle")) drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
            if (gamepads.isHeld(-1, "square")) drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
            if (gamepads.isHeld(-1, "cross")) drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
        }*/

        waitForStart();

        while (opModeIsActive()) {
            gamepads.copyStates();
            curTime = System.currentTimeMillis() / 1000L;
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -(gamepads.joystickValue(1, "left", "y")-gamepads.getTrigger(1, "right_trigger")),
                            -(gamepads.joystickValue(1, "left", "x")-gamepads.getTrigger(1, "right_trigger"))
                    ),
                    -(gamepads.joystickValue(1, "right", "x")-gamepads.getTrigger(1, "right_trigger"))
            ));

            // Slide preset control
            if (gamepads.isPressed(-1, "dpad_up") && vertSlidePreset+1 < vertSlidePresets.length) {vertSlidePreset++; vertSlides.setTargetPosition(vertSlidePresets[vertSlidePreset]);}
            if (gamepads.isPressed(-1, "dpad_down") && vertSlidePreset>0) {vertSlidePreset--; vertSlides.setTargetPosition(vertSlidePresets[vertSlidePreset]);}

            if (gamepads.isPressed(-1, "right_bumper")){
                new Thread(() -> {
                    try {
                        int targetPos = vertSlides.getTargetPosition()-300;
                        vertSlides.setTargetPosition(targetPos);

                        while (Math.abs(vertSlides.getLeftPosition() - targetPos) > 20) {
                            Thread.sleep(20);
                        }

                        Thread.sleep(500);

                        grabber.open();

                    } catch (InterruptedException e) {
                        // Handle interruption
                        Thread.currentThread().interrupt();
                    }
                }).start();
            }

            if (gamepads.isPressed(-1, "dpad_right")) {
                arm.up();
            }
            if (gamepads.isPressed(-1, "dpad_left")) {
                arm.down();
            }

            if (gamepads.isPressed(-1, "circle")) {
                if (grabber.isClosed()) {grabber.open(); lastFrontOpened = curTime;}
                else grabber.close();
            }
            if (grabber.getSwitchState() && curTime - lastFrontOpened > 2){
                grabber.close();
            }


            drive.updatePoseEstimate();


            telemetry.addData("x: ", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("Slide preset", vertSlidePreset);
            telemetry.update();

        }
    }
}