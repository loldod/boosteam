
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static android.os.SystemClock.sleep;

@Disabled
public class elevetor_class {
    public elevetor_class() {
    }


    // (Servo elevetorServoLeft, Servo elevetorServoRight, DcMotor spinL, DcMotor spinR, DcMotor elevtorMotor, double power, int time)
    public void open(Servo elevetorServoLeft, Servo elevetorServoRight) {
        elevetorServoLeft.setPosition(1);
        elevetorServoRight.setPosition(0);
    }

    public void close(Servo elevetorServoLeft, Servo elevetorServoRight) {
        elevetorServoLeft.setPosition(0);
        elevetorServoRight.setPosition(1);
    }
    public void In( DcMotor spinL, DcMotor spinR, double power)
    {
        spinL.setPower(power);
        spinR.setPower(-power);
    }
    public void Out(DcMotor spinL, DcMotor spinR, double power)
    {
        spinL.setPower(-power);
        spinR.setPower(power);
    }
    public void up(DcMotor elevtorMotor, double power, int time)
    {
        elevtorMotor.setPower(power);
        sleep(time);
    }
    public void down(DcMotor elevtorMotor)
    {
        elevtorMotor.setPower(-0.8);
        sleep(200);
        elevtorMotor.setPower(0.3);
        sleep(200);
    }
    public void stopAll(DcMotor spinL, DcMotor spinR,DcMotor elevtorMotor)
    {
        elevtorMotor.setPower(0);
        spinL.setPower(0);
        spinR.setPower(0);
    }


}