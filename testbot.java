


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="testbot", group="Linear Opmode")
//@Disabled
public class testbot extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor Pump = null;
    private Servo stabilizerLeft = null;
    private Servo stabilizerRight = null;
    private DcMotor newElevetor = null;
    private DcMotor relicMotor = null;
    private Servo upDown = null;
    private Servo catchrelic = null;

    auto_class autoDrive = new auto_class();
    slowMod slowDrive = new slowMod();

    @Override
    public void runOpMode() {

        drive_class drive = new drive_class();
        slowMod slowDrive = new slowMod();


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftFront = hardwareMap.dcMotor.get("Left_Front");
        leftBack = hardwareMap.dcMotor.get("Left_Back");
        rightBack = hardwareMap.dcMotor.get("Right_Back");
        rightFront = hardwareMap.dcMotor.get("Right_Front");
        Pump = hardwareMap.dcMotor.get("Pump");
        stabilizerLeft = hardwareMap.servo.get("stabilizerLeft");
        stabilizerRight = hardwareMap.servo.get("stabilizerRight");
        newElevetor = hardwareMap.dcMotor.get("newElevetor");
        relicMotor = hardwareMap.dcMotor.get("relicMotor");
        catchrelic = hardwareMap.servo.get("catcherrelic");
        upDown = hardwareMap.servo.get("upDown");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        boolean driveSlow = true;
        // run until the end of the match (driver presses STOP)

        //gamepad1 - relic, elevetor
        //gamepad2 - drive, slow drive

        while (opModeIsActive()) {



            if (gamepad2.left_stick_y > 0)
            {
                Pump.setPower(1);

            }

            else if (gamepad2.left_stick_y < 0)
            {
                Pump.setPower(-1);

            }
            else {
                Pump.setPower(0);
            }
            if (gamepad2.right_bumper)
            {
                stabilizerLeft.setPosition(0);
                stabilizerRight.setPosition(1);
            }

            else if (gamepad2.left_bumper)
            {
                stabilizerLeft.setPosition(1);
                stabilizerRight.setPosition(0);
            }

            if (gamepad2.y)
            {
                newElevetor.setPower(-1);
                sleep(400);
            }

            else if (gamepad2.a)
            {
                newElevetor.setPower(1);
                sleep(500);
                newElevetor.setPower(-0.15);
                sleep(400);
            }
            else {

                newElevetor.setPower(0);
            }
            if (gamepad2.dpad_left)
            {
                catchrelic.setPosition(1);
            }
            if (gamepad2.dpad_right)
            {
                catchrelic.setPosition(0);
            }
            if (gamepad2.dpad_up)
            {
                upDown.setPosition(0);
            }
            if (gamepad2.dpad_down)
            {
                upDown.setPosition(1);
            }
            if (gamepad2.right_stick_y > 0)
            {
                relicMotor.setPower(0.2);
            }
            if (gamepad2.right_stick_y < 0)
            {
                relicMotor.setPower(-0.2);
            }
            else
            {
                relicMotor.setPower(0);
            }





            if(driveSlow)
            {
                drive.drive_manager(leftFront, leftBack, rightFront, rightBack, gamepad1);
            }
            else
            {
                slowDrive.drive_manager(leftFront, leftBack, rightFront, rightBack, gamepad1);
            }
            if(gamepad1.start)
            {
                driveSlow = !driveSlow;
            }
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
