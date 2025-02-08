package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.BackArm;
import org.firstinspires.ftc.teamcode.utils.Diffy;
import org.firstinspires.ftc.teamcode.utils.FrontArm;
import org.firstinspires.ftc.teamcode.utils.GamepadPair;
import org.firstinspires.ftc.teamcode.utils.Grabber;
import org.firstinspires.ftc.teamcode.utils.HorizSlidePair;
import org.firstinspires.ftc.teamcode.utils.RumbleEffects;
import org.firstinspires.ftc.teamcode.utils.VertSlidePair;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        String driveStyle = "robot-centric";

        long curTime;
        long lastFrontOpened = 0;
        long lastBackOpened = 0;



        Grabber frontGrabber = new Grabber(0.73, 1, hardwareMap.servo.get("frontGrabberServo"), hardwareMap.touchSensor.get("frontGrabberSwitch"));
        Grabber backGrabber = new Grabber(0.73, 1, hardwareMap.servo.get("backGrabberServo"), hardwareMap.touchSensor.get("backGrabberSwitch"));

        VertSlidePair vertSlides = new VertSlidePair(4100, 1, hardwareMap);
        HorizSlidePair horizSlides = new HorizSlidePair(hardwareMap, true);
        BackArm backArm = new BackArm(0.55, 0, hardwareMap, true);
        FrontArm frontArm = new FrontArm(1, 0.35, hardwareMap);
        Diffy diffy = new Diffy(hardwareMap, true);

        int grabOffWall = 894;
        int aboveTopBar = 3550;
        int hangHeight = 2050;
        int[] vertSlidePresets = {0, grabOffWall, aboveTopBar, hangHeight};
        frontArm.forward();
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
            gamepads.copyStates();
            horizSlides.update();
            curTime = System.currentTimeMillis();

            double driveScaleFactor = 1-gamepads.getTrigger(1, "right_trigger");
            Vector2d driveVector = new Vector2d(
                    -(gamepads.joystickValue(1, "left", "y")*driveScaleFactor),
                    -(gamepads.joystickValue(1, "left", "x")*driveScaleFactor)
            );
            if (driveStyle.equals("field-centric")) {
                float gpx = -gamepad1.left_stick_y;
                float gpy = -gamepad1.left_stick_x;
                float head = (float) -drive.pose.heading.toDouble();
                float nx = (float) (((gpx * Math.cos(head)) - (gpy * Math.sin(head))) * driveScaleFactor);
                float ny = (float) (((gpx * Math.sin(head)) + (gpy * Math.cos(head))) * driveScaleFactor);
                driveVector = new Vector2d(
                        nx,
                        ny
                );
            }

            drive.setDrivePowers(new PoseVelocity2d(
                    driveVector,
                    -(gamepads.joystickValue(1, "right", "x")*driveScaleFactor)
            ));

            // Slide preset control
            if (gamepads.isPressed(1, "dpad_up") && vertSlidePreset+1 < vertSlidePresets.length) {vertSlidePreset++; vertSlides.setTargetPosition(vertSlidePresets[vertSlidePreset]);}
            if (gamepads.isPressed(1, "dpad_down") && vertSlidePreset>0) {vertSlidePreset--; vertSlides.setTargetPosition(vertSlidePresets[vertSlidePreset]);}

            // Back Arm Control
            if (gamepads.isHeld(-1, "dpad_right")) {
                backArm.setPower(1);
            } else if (gamepads.isHeld(-1, "dpad_left")) backArm.setPower(-1);
            else backArm.setPower(0);

            // Switch Drive Style
            if (gamepads.isPressed(1, "left_stick_button")){
                if (driveStyle.equals("field-centric")) driveStyle = "robot-centric";
                else driveStyle = "field-centric";
            }

            // Horiz Slide Control
            if (gamepads.isPressed(1, "square")){
                if (horizSlides.getTargetRotation()==0){
                    horizSlides.setTargetRotation(400);
                } else {
                    horizSlides.setTargetRotation(0);
                }
            }
            // Toggle both grabbers
            if (gamepads.isPressed(1, "circle")) {
                if (frontGrabber.isClosed()) {
                    frontGrabber.open();
                    backGrabber.open();
                    lastFrontOpened = curTime;
                    lastBackOpened = curTime;
                }
                else {
                    frontGrabber.close();
                    backGrabber.close();
                }
            }

            if (frontGrabber.getSwitchState() && curTime - lastFrontOpened > 2000 && !frontGrabber.isClosed()){
                new Thread(() -> {
                    try {
                        frontGrabber.close();
                        gamepads.blipRumble(-1, 1);

                        Thread.sleep(750);

                        if (!frontGrabber.getSwitchState()) {frontGrabber.open(); gamepads.rumble(1, RumbleEffects.alternating);}

                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }).start();
            }
            if (backGrabber.getSwitchState() && curTime - lastBackOpened > 2000 && !backGrabber.isClosed()){
                new Thread(() -> {
                    try {
                        backGrabber.close();
                        gamepads.blipRumble(1, 1);

                        Thread.sleep(750);

                        if (!backGrabber.getSwitchState()) {backGrabber.open(); gamepads.rumble(1, RumbleEffects.alternating);}

                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }).start();
            }

            // Lower and release to bar
            if (vertSlidePresets[vertSlidePreset] == aboveTopBar && gamepads.isPressed(1, "right_bumper")){

                new Thread(() -> {
                    vertSlides.changeTargetPosition(-400);
                    while (Math.abs(vertSlides.getCurrentPosition() - vertSlides.getTargetPosition()) > 15) {
                        try {
                            Thread.sleep(10);
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                        }
                    }
                    frontGrabber.open();
                    gamepads.blipRumble(1, 1);
                }).start();
            }
            // Pick up from wall
            if (vertSlidePresets[vertSlidePreset] == grabOffWall && gamepads.isPressed(1, "right_bumper")){
                frontGrabber.close();
                vertSlides.changeTargetPosition(300);
                gamepads.blipRumble(1,  1);
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