package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import static android.os.SystemClock.sleep;

public class auto_class {

    public auto_class() {}

    private void autoDriveRight(DcMotor leftBack, DcMotor leftFront, DcMotor rightBack, DcMotor rightFront, double power, int time)
    {
        leftBack.setPower(-power);
        leftFront.setPower(power);
        rightBack.setPower(-power);
        rightFront.setPower(power);
        sleep(time);
        stopDrive(leftBack, leftFront, rightBack, rightFront);
    }

    private void autoDriveLeft(DcMotor leftBack, DcMotor leftFront, DcMotor rightBack, DcMotor rightFront, double power, int time)
    {
        autoDriveRight(leftBack, leftFront, rightBack, rightFront, -power, time);
    }

    public void stopDrive(DcMotor leftBack, DcMotor leftFront, DcMotor rightBack, DcMotor rightFront) {
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void autoDriveForward(DcMotor leftBack, DcMotor leftFront, DcMotor rightBack, DcMotor rightFront, double power, int time)
    {
        leftBack.setPower(-power);
        leftFront.setPower(-power);
        rightBack.setPower(power);
        rightFront.setPower(power);
        sleep(time);
        stopDrive(leftBack, leftFront, rightBack, rightFront);
    }

    public void autoDriveBackward(DcMotor leftBack, DcMotor leftFront, DcMotor rightBack, DcMotor rightFront, double power, int time) {
        autoDriveForward(leftBack, leftFront, rightBack, rightFront, -power, time);
    }

    public void autoTurnRight(DcMotor leftBack, DcMotor leftFront, DcMotor rightBack, DcMotor rightFront, double power, int time)
    {
        leftBack.setPower(-power);
        leftFront.setPower(-power);
        rightBack.setPower(-power);
        rightFront.setPower(-power);
        sleep(time);
        stopDrive(leftBack, leftFront, rightBack, rightFront);
    }

    public void autoTurnLeft(DcMotor leftBack, DcMotor leftFront, DcMotor rightBack, DcMotor rightFront, double power, int time) {
        autoTurnRight(leftBack, leftFront, rightBack, rightFront, power*-1, time);
    }

    public void autoTurnRight90Deg(DcMotor leftBack, DcMotor leftFront, DcMotor rightBack, DcMotor rightFront) {
        autoTurnRight(leftBack, leftFront, rightBack, rightFront, 1, 1000);
    }

    public void driveRight(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack, double power, int time)
    {
        leftBack.setPower(power);
        leftFront.setPower(-power);
        rightBack.setPower(power);
        rightFront.setPower(-power);
        sleep(time);
        stopDrive(leftBack, leftFront, rightBack, rightFront);
    }

    public void driveLeft(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack, double power, int time)
    {
        driveRight(leftFront, leftBack, rightFront, rightBack, -power, time);
    }

    public void autoTurnLeft90Deg(DcMotor leftBack, DcMotor leftFront, DcMotor rightBack, DcMotor rightFront) {
        autoTurnLeft(leftBack, leftFront, rightBack, rightFront, 1, 1000);
    }

    public void turn180Deg(DcMotor leftBack, DcMotor leftFront, DcMotor rightBack, DcMotor rightFront)
    {
        for (int i=0; i<2; i++)
        {
            autoTurnRight90Deg(leftBack, leftFront, rightBack, rightFront);
        }
    }

    public void turn360Deg(DcMotor leftBack, DcMotor leftFront, DcMotor rightBack, DcMotor rightFront)
    {
        for (int i=0; i<2; i++)
        {
            turn180Deg(leftBack, leftFront, rightBack, rightFront);
        }
    }

}
