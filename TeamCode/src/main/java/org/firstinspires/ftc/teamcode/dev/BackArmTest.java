package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.BackArm;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;

@TeleOp(name = "Back Arm Test", group = "Dev")
public class BackArmTest extends LinearOpMode {
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        BackArm arm = new BackArm(hardwareMap, false);
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            gamepads.copyStates();
            arm.update();

            if (gamepads.isHeld(-1, "cross")) {
                arm.setPower(1);
            } else if (gamepads.isHeld(-1, "circle")) arm.setPower(-1);
            else arm.setPower(0);

            telemetry.addData("Current Angle", arm.getAngle());
            telemetry.addData("TotalRotation", arm.getPosition());
            telemetry.update();
        }
    }
}

