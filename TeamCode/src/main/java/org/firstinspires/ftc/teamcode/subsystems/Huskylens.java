package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Huskylens {

    private final HuskyLens huskyLens;

    public Huskylens(HardwareMap hmap){
        huskyLens = hmap.get(HuskyLens.class, "huskylens");
        selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    public boolean knock(){
        return huskyLens.knock();
    }

    public void selectAlgorithm(HuskyLens.Algorithm algorithm){
        huskyLens.selectAlgorithm(algorithm);
    }

    public HuskyLens.Block[] getBlocks(){
        return huskyLens.blocks();
    }

}