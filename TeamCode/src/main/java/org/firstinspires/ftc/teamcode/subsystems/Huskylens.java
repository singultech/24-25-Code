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

    public HuskyLens.Block getBiggestBlock() {
        HuskyLens.Block biggest = huskyLens.blocks()[0];
        for (HuskyLens.Block block : huskyLens.blocks()){
            if((block.height*block.width)>=(biggest.height*biggest.width)) biggest = block;
        }
        return biggest;
    }

}