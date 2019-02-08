package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import static android.os.SystemClock.sleep;

public class Two_Motor_Class {

    private void autoDriveRight(DcMotor leftBack, DcMotor rightBack, double power, int time)
    {
        leftBack.setPower(-power);
        rightBack.setPower(-power);
        sleep(time);
        stopDrive(leftBack, rightBack);
    }

    private void autoDriveLeft(DcMotor leftBack, DcMotor rightBack, double power, int time)
    {
        autoDriveRight(leftBack, rightBack, -power, time);
    }

    public void stopDrive(DcMotor leftBack, DcMotor rightBack) {
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void TurnOnMotor(DcMotor motor, double power, int time) {
        motor.setPower(power);
        sleep(time);
        motor.setPower(0);
    }

    public void autoDriveForward(DcMotor leftBack, DcMotor rightBack, double power, int time)
    {
        leftBack.setPower(-power);
        rightBack.setPower(power);
        sleep(time);
        stopDrive(leftBack, rightBack);
    }

    public void autoDriveBackward(DcMotor leftBack, DcMotor rightBack, double power, int time) {
        autoDriveForward(leftBack, rightBack, -power, time);
    }

    public void autoTurnRight(DcMotor leftBack, DcMotor rightBack, double power, int time)
    {
        leftBack.setPower(-power);
        rightBack.setPower(-power);
        sleep(time);
        stopDrive(leftBack, rightBack);
    }

    public void autoTurnLeft(DcMotor leftBack, DcMotor rightBack, double power, int time) {
        autoTurnRight(leftBack, rightBack, power*-1, time);
    }

    public void autoTurnRight90Deg(DcMotor leftBack, DcMotor rightBack) {
        autoTurnRight(leftBack, rightBack, 1, 1000);
    }

    public void driveRight(DcMotor leftBack, DcMotor rightBack, double power, int time)
    {
        leftBack.setPower(power);
        rightBack.setPower(power);
        sleep(time);
        stopDrive(leftBack, rightBack);
    }

    public void driveLeft(DcMotor leftBack, DcMotor rightBack, double power, int time)
    {
        driveRight(leftBack, rightBack, -power, time);
    }

    public void autoTurnLeft90Deg(DcMotor leftBack, DcMotor rightBack) {
        autoTurnLeft(leftBack, rightBack, 1, 1000);
    }

    public void turn180Deg(DcMotor leftBack, DcMotor rightBack)
    {
        for (int i=0; i<2; i++)
        {
            autoTurnRight90Deg(leftBack, rightBack);
        }
    }

    public void turn360Deg(DcMotor leftBack, DcMotor rightBack)
    {
        for (int i=0; i<2; i++)
        {
            turn180Deg(leftBack, rightBack);
        }
    }

}

