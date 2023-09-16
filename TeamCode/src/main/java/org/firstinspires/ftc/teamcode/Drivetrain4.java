package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

class Drivetrain4{
    private DcMotor lf;
    private DcMotor rf;
    private DcMotor lb;
    private DcMotor rb;
    private double robotRadius = 1;
    private double wheelRadius = 1;
    private double ticksPerRotation = 1;


    public Drivetrain4(HardwareMap hardwareMap, String lf, String rf, String lb, String rb){
        this.lf = hardwareMap.get(DcMotor.class, lf);
        this.rf = hardwareMap.get(DcMotor.class, rf);
        this.lb = hardwareMap.get(DcMotor.class, lb);
        this.rb = hardwareMap.get(DcMotor.class, rb);
    }

    public void setRobotRadius(double robotRadius){
        this.robotRadius = robotRadius;
    }
    public void setWheelRadius(double wheelRadius){
        this.wheelRadius = wheelRadius;
    }
    public void setTicksPerRotation(double ticksPerRotation){
        this.ticksPerRotation = ticksPerRotation;
    }

    public DcMotor getLf(){
        return this.lf;
    }
    public DcMotor getRf(){
        return rf;
    }
    public DcMotor getLb(){
        return lb;
    }
    public DcMotor getRb(){
        return rb;
    }

    public void setDirections(DcMotorSimple.Direction lf, DcMotorSimple.Direction rf, DcMotorSimple.Direction lb, DcMotorSimple.Direction rb){
        this.lf.setDirection(lf);
        this.rf.setDirection(rf);
        this.lb.setDirection(lb);
        this.rb.setDirection(rb);
    }
    public void setPowers(double lf, double rf, double lb, double rb){
        this.lf.setPower(lf);
        this.rf.setPower(rf);
        this.lb.setPower(lb);
        this.rb.setPower(rb);
    }
    public void setPowers(double pow){
        this.setPowers(
                pow,
                pow,
                pow,
                pow
        );
    }
    public void moveForward(double dist){

        int startLf = lf.getCurrentPosition();
        int startRf = rf.getCurrentPosition();
        int startLb = lb.getCurrentPosition();
        int startRb = rb.getCurrentPosition();

//        while(
//                Math.abs(startLf - lf.getCurrentPosition())/ticksPerRotation*wheelRadius < errorMargin &&
//                Math.abs(startRf - rf.getCurrentPosition())/ticksPerRotation*wheelRadius < errorMargin &&
//                Math.abs(startLb - lb.getCurrentPosition())/ticksPerRotation*wheelRadius < errorMargin &&
//                Math.abs(startRb - rb.getCurrentPosition())/ticksPerRotation*wheelRadius < errorMargin
//        ){
//
//        }

    }
}
