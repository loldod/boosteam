package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="learn", group="Linear Opmode")
@Disabled
public class learn extends LinearOpMode {
    private DcMotor right;
    private DcMotor left;

    @Override
    public void runOpMode() {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.right_trigger > 0)
            {
                left.setPower(gamepad1.right_trigger);
                right.setPower(-gamepad1.right_trigger);
            }
            else if(gamepad1.left_trigger > 0)
            {
                left.setPower(-gamepad1.left_trigger);
                right.setPower(gamepad1.left_trigger);
            }
            else
            {
                left.setPower(0);
                right.setPower(0);
            }
        }

    }
}
