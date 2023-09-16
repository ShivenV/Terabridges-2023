package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrivetrain extends Drivetrain4 {

    public MecanumDrivetrain(HardwareMap hardwareMap, String lf, String rf, String lb, String rb) {
        super(hardwareMap, lf, rf, lb, rb);
    }


    public void setPowersMecanum(double x, double y, double r){
        this.setPowers(
                y - x - r,
                y + x + r,
                y + x - r,
                y - x + r
        );
    }

    public void moveRelative(double x, double y){

    }
}
