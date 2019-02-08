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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@TeleOp(name="gamepad", group="Linear Opmode")
//@Disabled
public class gamepad extends LinearOpMode {

    ColorSensor sensorColor;
    private DcMotor leftFront = null; //drive
    private DcMotor leftBack = null; //drive
    private DcMotor rightFront = null; //drive
    private DcMotor rightBack = null; //drive

    private Servo elevetorServoLeft = null;
    private Servo elevetorServoRight = null;

    private Servo updown = null;
    private Servo balls = null;

    private DcMotor spinL;
    private DcMotor spinR;


    private DcMotor elevtorMotor = null;
    DistanceSensor sensorDistance;

    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position

    @Override
    public void runOpMode() {

        boolean driveSlow = false;
        drive_class drive = new drive_class();
        slowMod slowDrive = new slowMod();
        newElevetor elevetor_func = new newElevetor();
        relic_class relic = new relic_class();

        leftFront = hardwareMap.dcMotor.get("Left_front");
        leftBack = hardwareMap.dcMotor.get("Left_back");
        rightBack = hardwareMap.dcMotor.get("Right_back");
        rightFront = hardwareMap.dcMotor.get("Right_front");

        elevetorServoLeft = hardwareMap.servo.get("elevetorServoLeft");
        elevetorServoRight = hardwareMap.servo.get("elevetorServoRight");

        spinL = hardwareMap.dcMotor.get("spinL");
        spinR = hardwareMap.dcMotor.get("spinR");

        updown = hardwareMap.servo.get("updown");
        balls = hardwareMap.servo.get("balls");

        elevtorMotor = hardwareMap.dcMotor.get("elevetorMotor");
        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        waitForStart();

        // run until the end of the match (driver presses STOP)

        //gamepad1 - relic, elevetor
        //gamepad2 - drive, slow drive

        while (opModeIsActive()) {

            if (gamepad1.x)
            {
                spinL.setPower(-1);
                spinR.setPower(1);
            }
            else if(gamepad1.y){
                spinL.setPower(1);
                spinR.setPower(-1);
            }
            else if (gamepad1.b){
                spinL.setPower(0);
                spinR.setPower(0);
            }
            if(gamepad1.a || gamepad2.a) {
                balls.setPosition(0.5);
                sleep(150);
                updown.setPosition(1);
                sleep(400);
                balls.setPosition(1);
            }


            if (gamepad1.dpad_up)
            {
                elevtorMotor.setPower(0.9);
                sleep(1700);
            }
            else if (gamepad1.dpad_down)
            {
                elevtorMotor.setPower(-0.8);
                sleep(200);
                elevtorMotor.setPower(0.3);
                sleep(200);
            }
            else {
                elevtorMotor.setPower(0);
            }


            if (gamepad1.left_bumper)
            {
                elevetorServoLeft.setPosition(0.5);
                //elevetorServoRight.setPosition(0.2);
            }
            else if(gamepad1.right_bumper)
            {
                elevetorServoLeft.setPosition(0);
                elevetorServoRight.setPosition(1);

            }

            //relic.relicManager(relicMotor1, relicMotor2, hand, relicCatcher, gamepad1);
            if (!driveSlow) {
                drive.drive_manager(leftFront, leftBack, rightFront, rightBack, gamepad2);
            } else {
                slowDrive.drive_manager(leftFront, leftBack, rightFront, rightBack, gamepad2);
            }
            if (gamepad2.start) {
                driveSlow = !driveSlow;
            }

            telemetry.addData("Status", "Run Time: " );
            telemetry.update();
        }
    }
}
