package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class AutoSubsystems {
    private final FrontArm arm;
    private final VertSlidePair slides;
    private final Grabber grabber;

    public AutoSubsystems(FrontArm a, VertSlidePair s, Grabber g) {
        arm = a;
        slides = s;
        grabber = g;
    }

    public Action moveArm(double position){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                arm.setPosition(position);
                return true;
            }
        };
    }
}
