package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class elevetor {
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    public elevetor()
    {}

    public void checkDirection(DcMotor eleMotor, Gamepad gamepad1)
    {
        if(gamepad1.right_stick_y > 0) { up(eleMotor, gamepad1); }

        else if(gamepad1.right_stick_y < 0) { down(eleMotor, gamepad1); }

        else { eleMotor.setPower(0); }
    }

    public void getAndLeave (Servo upLeft, Servo downLeft,Servo upRight, Servo downRight, Gamepad gamepad1)
    {
        if(gamepad1.a) {
            downLeft.setPosition(MIN_POS);
            downRight.setPosition(MAX_POS);
        }
        else if(gamepad1.y)
        {
            upLeft.setPosition(MIN_POS);
            upRight.setPosition(MAX_POS);
        }
        if(gamepad1.b)
        {
            downLeft.setPosition(MAX_POS/2);
            downRight.setPosition(MAX_POS/2);

            upLeft.setPosition(MAX_POS/2);
            upRight.setPosition(MAX_POS/2);
        }
    }



    public void up(DcMotor eleMotor, Gamepad gamepad1)
    {
        eleMotor.setPower(gamepad1.right_stick_y);
    }

    public void down(DcMotor eleMotor, Gamepad gamepad1)
    {
        eleMotor.setPower(gamepad1.right_stick_y);
    }

}
