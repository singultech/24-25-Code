package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.BackArm;
import org.firstinspires.ftc.teamcode.utils.BackGrabber;
import org.firstinspires.ftc.teamcode.utils.DiffyGrabber;
import org.firstinspires.ftc.teamcode.utils.FrontArm;
import org.firstinspires.ftc.teamcode.utils.FrontGrabber;
import org.firstinspires.ftc.teamcode.utils.GamepadPair;
import org.firstinspires.ftc.teamcode.utils.HorizSlidePair;
import org.firstinspires.ftc.teamcode.utils.RumbleEffects;
import org.firstinspires.ftc.teamcode.utils.VertSlidePair;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(90)));
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        String driveStyle = "robot-centric";

        long curTime;
        long lastFrontOpened = 0;
        long lastBackOpened = 0;

        boolean isHorizExtended = false;

        FrontGrabber frontGrabber = new FrontGrabber(0.73, 1, hardwareMap);
        BackGrabber backGrabber = new BackGrabber(0.73, 1, hardwareMap);

        VertSlidePair vertSlides = new VertSlidePair(4100, 1, hardwareMap);
        HorizSlidePair horizSlides = new HorizSlidePair(hardwareMap);
        BackArm backArm = new BackArm(0.55, 0, hardwareMap);
        FrontArm frontArm = new FrontArm(1, 0.35, hardwareMap);
        DiffyGrabber diffy = new DiffyGrabber(hardwareMap);
        int[] vertSlidePresets = {0, 300, 894, 2100,                         2478, 2800, 3500, 4100};
        frontArm.forward();
        /*
        Zero
        Slightly lifted for pickup from ground
        Grab off wall
        slightly above top bar
        slightly above top basket?
        */
        int vertSlidePreset = 0;

        /*
        while (!isStarted()){
            if (gamepads.isHeld(-1, "triangle")) drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
            if (gamepads.isHeld(-1, "circle")) drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
            if (gamepads.isHeld(-1, "square")) drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
            if (gamepads.isHeld(-1, "cross")) drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
        }*/
        Vector2d driveVector;

        waitForStart();

        while (opModeIsActive()) {
            gamepads.copyStates();
            horizSlides.update();
            curTime = System.currentTimeMillis();

            driveVector = new Vector2d(
                    -(gamepads.joystickValue(1, "left", "y")-gamepads.getTrigger(1, "right_trigger")),
                    -(gamepads.joystickValue(1, "left", "x")-gamepads.getTrigger(1, "right_trigger"))
            );
            if (driveStyle.equals("field-centric")) {
                float gpx = -gamepad1.left_stick_y;
                float gpy = -gamepad1.left_stick_x;
                float head = (float) -drive.pose.heading.toDouble();
                float nx = (float) ((gpx * Math.cos(head)) - (gpy * Math.sin(head)));
                float ny = (float) ((gpx * Math.sin(head)) + (gpy * Math.cos(head)));
                driveVector = new Vector2d(
                        nx,
                        ny
                );
            }

            drive.setDrivePowers(new PoseVelocity2d(
                    driveVector,
                    -(gamepads.joystickValue(1, "right", "x")-gamepads.getTrigger(1, "right_trigger"))
            ));

            // Slide preset control
            if (gamepads.isPressed(1, "dpad_up") && vertSlidePreset+1 < vertSlidePresets.length) {vertSlidePreset++; vertSlides.setTargetPosition(vertSlidePresets[vertSlidePreset]);}
            if (gamepads.isPressed(1, "dpad_down") && vertSlidePreset>0) {vertSlidePreset--; vertSlides.setTargetPosition(vertSlidePresets[vertSlidePreset]);}

            if (gamepads.isHeld(-1, "dpad_right")) {
                backArm.setPower(1);
            } else if (gamepads.isHeld(-1, "dpad_left")) backArm.setPower(-1);
            else backArm.setPower(0);


            if (gamepads.isPressed(1, "square")){
                if (!isHorizExtended){
                    isHorizExtended = true;
                    horizSlides.setTargetRotation(400);
                } else {
                    isHorizExtended = false;
                    horizSlides.setTargetRotation(0);
                }
            }

            if (gamepads.isPressed(1, "circle")) {
                if (frontGrabber.isClosed()) {frontGrabber.open(); lastFrontOpened = curTime;}
                else frontGrabber.close();
                if (backGrabber.isClosed()) {backGrabber.open(); lastBackOpened = curTime;}
                else backGrabber.close();
            }/*
            if (frontGrabber.getSwitchState() && curTime - lastFrontOpened > 2000){
                new Thread(() -> {
                    try {
                        frontGrabber.close();
                        gamepads.blipRumble(-1, 1);

                        Thread.sleep(300);

                        if (!frontGrabber.getSwitchState()) {frontGrabber.open(); gamepads.rumble(1, RumbleEffects.alternating);}

                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }).start();
            }*/
            if (backGrabber.getSwitchState() && curTime - lastBackOpened > 2000){
                new Thread(() -> {
                    try {
                        backGrabber.close();
                        gamepads.blipRumble(1, 1);

                        Thread.sleep(300);

                        if (!backGrabber.getSwitchState()) {backGrabber.open(); gamepads.rumble(1, RumbleEffects.alternating);}

                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }).start();
            }


            drive.updatePoseEstimate();


            telemetry.addData("x: ", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("Slide preset", vertSlidePreset);
            telemetry.addData("Drive style", driveStyle);
            telemetry.update();

        }
    }
}