package org.firstinspires.ftc.teamcode;

/**
 * Created by Boosteam on 01/01/2018.
 */
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class relic_class {
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    static boolean up = true;
    public relic_class()
    {}

    public void relicManager(DcMotor relicMotor1, DcMotor relicMotor2, DcMotor hand, Servo catcher, Gamepad gamepad2)
    {
        checkDirection(relicMotor1, relicMotor2, gamepad2);
        updown(hand, gamepad2);
        catchrelic(catcher, gamepad2);
    }

     private void checkDirection(DcMotor relicMotor1, DcMotor relicMotor2, Gamepad gamepad2)
    {
        if(gamepad2.left_stick_x > 0) {
            relicMotor1.setPower(0.5);
            relicMotor2.setPower(0.5);
        }
        else if(gamepad2.left_stick_x < 0) {
            relicMotor1.setPower(gamepad2.left_stick_x);
            relicMotor2.setPower(gamepad2.left_stick_x);
        }
        else {
            relicMotor1.setPower(0);
            relicMotor2.setPower(0);
        }
    }

    private void updown(DcMotor hand, Gamepad gamepad1)
    {
        if(gamepad1.left_stick_y > 0) {
            if(gamepad1.x)
            {
                hand.setPower(gamepad1.left_stick_y/4);
            }
            else {
                hand.setPower(gamepad1.left_stick_y / 2);
            }
        }
        else if(gamepad1.left_stick_y < 0) {
            if(gamepad1.x)
            {
                hand.setPower(gamepad1.left_stick_y/4);
            }
            else {
                hand.setPower(gamepad1.left_stick_y / 2);
            }
        }
        else { hand.setPower(0);}
    }
    private void catchrelic(Servo relicCatch, Gamepad gamepad1)
    {
        if(gamepad1.right_bumper)
        {
            relicCatch.setPosition(MIN_POS);
        }
        else if (gamepad1.left_bumper)
        {
            relicCatch.setPosition(MAX_POS);
        }
    }
}
