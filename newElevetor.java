package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


@Disabled
public class newElevetor implements Runnable{
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    static private boolean change;
    static private boolean isUp = false;
    static private boolean close = false;

    public newElevetor()
    {}

    public void run(){
        try {
            changeVar();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void elevetorManger(DcMotor spin, DcMotor leftMotor, DcMotor rightMotor, Servo leftServo, Servo rightServo, Gamepad gamepad1) throws InterruptedException {
        if(gamepad1.x) {
            isUp = true;
            spin.setPower(0.75);
            Thread.sleep(1000);
            spin.setPower(0);
        }
        else if(gamepad1.b){
            isUp = false;
            spin.setPower(-1);
            Thread.sleep(250);
            spin.setPower(0.1);
            Thread.sleep(50);
            spin.setPower(0);
        }

        if(gamepad1.a){
            close = true;
            closeServo(leftServo, rightServo);
        }
        else if(gamepad1.y){
            close = false;
            openServo(leftServo, rightServo);
        }

        if(gamepad1.dpad_up) {
            rightMotor.setPower(-1);
            leftMotor.setPower(1);
            /*close = false;
            Thread t = new Thread();
            t.start();
            if(close)
            {
                rightMotor.setPower(0.75);
                leftMotor.setPower(0.5);
            }
            else
            {
                rightMotor.setPower(0.5);
                leftMotor.setPower(0.75);
            }
            close = true;*/
        }
        else if(gamepad1.dpad_down) {
            rightMotor.setPower(-1);
            leftMotor.setPower(1);
            /*close = false;
            Thread t = new Thread();
            t.start();
            if(close)
            {
                rightMotor.setPower(-0.75);
                leftMotor.setPower(-0.5);
            }
            else
            {
                rightMotor.setPower(-0.5);
                leftMotor.setPower(-0.75);
            }
            close = true;*/
        }
        else{
            rightMotor.setPower(0);
            leftMotor.setPower(0);
        }
    }

    private void closeServo(Servo left, Servo right){
        left.setPosition(MAX_POS);
        right.setPosition(MIN_POS);
    }

    private void openServo(Servo left, Servo right){
        left.setPosition(MIN_POS);
        right.setPosition(MAX_POS);
    }

    private void changeVar() throws InterruptedException {
        while (true) {
            Thread.sleep(100);
            change = !change;
            if(close)
                break;
        }
    }
}
