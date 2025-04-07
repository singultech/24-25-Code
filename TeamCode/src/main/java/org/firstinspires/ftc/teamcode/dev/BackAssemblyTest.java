package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.BackArm;
import org.firstinspires.ftc.teamcode.subsystems.Diffy;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;

@TeleOp(name = "BackAssemblyTest")
public class BackAssemblyTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        BackArm backArm = new BackArm(hardwareMap);
        Diffy diffy = new Diffy(hardwareMap);
        
        waitForStart();
        while (!isStopRequested()) {
            gamepads.copyStates();
            diffy.update();
            backArm.update();

            if(gamepads.isPressed("dpad_right")) backArm.changeTargetRotation(15);
            if(gamepads.isPressed("dpad_left")) backArm.changeTargetRotation(-15);
            if(gamepads.isPressed("circle")) diffy.pitchGrabber(15);
            if(gamepads.isPressed("square")) diffy.pitchGrabber(-15);

            telemetry.addLine(backArm.toString());
            telemetry.addLine(diffy.log());
            telemetry.update();
        }
    }
}