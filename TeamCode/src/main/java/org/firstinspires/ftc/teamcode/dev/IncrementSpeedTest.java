package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AutoSubsystems;
import org.firstinspires.ftc.teamcode.subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Huskylens;
import org.firstinspires.ftc.teamcode.subsystems.VertSlidePair;

@TeleOp(name = "Speed Increment Test", group = "Testing")
public class IncrementSpeedTest extends LinearOpMode {
    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, initialPose);
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        double strafeSpeed = 0.0;
        double forwardSpeed = 0.0;
        waitForStart();
        while (!isStopRequested()) {
            gamepads.copyStates();
            if(gamepads.isPressed(-1, "dpad_right")){
                strafeSpeed+=0.02;
            }
            if(gamepads.isPressed(-1, "dpad_left")){
                strafeSpeed-=0.02;
            }
            if(gamepads.isPressed(-1, "dpad_up")){
                forwardSpeed+=0.02;
            }
            if(gamepads.isPressed(-1, "dpad_down")){
                forwardSpeed-=0.02;
            }
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(forwardSpeed, strafeSpeed),
                    0
            ));
            telemetry.addData("Forward/Back Speed", forwardSpeed);
            telemetry.addData("Sideways Speed", strafeSpeed);
            telemetry.update();
        }
    }
}