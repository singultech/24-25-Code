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
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AutoSubsystems;
import org.firstinspires.ftc.teamcode.subsystems.CRAxon;
import org.firstinspires.ftc.teamcode.subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Huskylens;
import org.firstinspires.ftc.teamcode.subsystems.VertSlidePair;

@TeleOp(name = "Cont. Rotation Axon Test", group = "Testing")
public class CRAxonTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CRServo crservo = hardwareMap.crservo.get("leftDiffy");
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "leftDiffyEncoder");
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        CRAxon servo = new CRAxon(crservo, encoder);
        waitForStart();
        while (!isStopRequested()) {
            gamepads.copyStates();
            servo.update();

            if(gamepads.isPressed(-1, "dpad_up")){
                servo.changeTargetRotation(15);
            }
            if(gamepads.isPressed(-1, "dpad_down")){
                servo.changeTargetRotation(-15);
            }
            if(gamepads.isPressed(-1, "cross")){
                servo.setTargetRotation(0);
            }

            telemetry.addLine(servo.log());
            telemetry.update();
        }
    }
}