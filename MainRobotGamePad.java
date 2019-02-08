/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.tree.JCTree;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name="MainRobotGamePad", group="Linear Opmode")
//@Disabled
public class MainRobotGamePad extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor Pumper;
    private DcMotor LeftBack;
    private DcMotor RightBack;
    private DcMotor OpenPump;
    private DcMotor BringUpPumpRight;
    private DcMotor BringUpPumpLeft;

    DigitalChannel startDigitalTouch;  // Hardware Device Object
    DigitalChannel endDigitalTouch;  // Hardware Device Object

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        LeftBack = hardwareMap.dcMotor.get("left_motor"); // port: 0
        RightBack = hardwareMap.dcMotor.get("right_motor"); // port: 1
        Pumper = hardwareMap.dcMotor.get("pumper"); // port: 2
        OpenPump = hardwareMap.dcMotor.get("op"); // port: 3
        BringUpPumpRight = hardwareMap.dcMotor.get("bup_right"); // port: 0
        BringUpPumpLeft = hardwareMap.dcMotor.get("bup_left"); // port: 1

        startDigitalTouch = hardwareMap.get(DigitalChannel.class, "start_touch");
        endDigitalTouch = hardwareMap.get(DigitalChannel.class, "end_touch");

        startDigitalTouch.setMode(DigitalChannel.Mode.INPUT);
        endDigitalTouch.setMode(DigitalChannel.Mode.INPUT);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive())
        {
            if (gamepad1.right_trigger > 0) //forward
            {
                RightBack.setPower(gamepad1.right_trigger);
                LeftBack.setPower(-gamepad1.right_trigger);
            }

            else if (gamepad1.left_trigger > 0) //backward
            {
                RightBack.setPower(-gamepad1.left_trigger);
                LeftBack.setPower(gamepad1.left_trigger);
            }

            else if (gamepad1.left_stick_x > 0) //turn right
            {
                RightBack.setPower(-1);
                LeftBack.setPower(-1);
            }

            else if (gamepad1.left_stick_x < 0) //turn left
            {
                RightBack.setPower(1);
                LeftBack.setPower(1);
            }

            else if (gamepad1.dpad_right) //drive right
            {
                RightBack.setPower(0);
                LeftBack.setPower(-1);
            }

            else if (gamepad1.dpad_left) // turn left
            {
                RightBack.setPower(1);
                LeftBack.setPower(0);
            }

            else {
                RightBack.setPower(0);
                LeftBack.setPower(0);
            }

            if (startDigitalTouch.getState() == true){
                if (gamepad2.right_trigger > 0) OpenPump.setPower(-0.8);
                else OpenPump.setPower(0);
            }

            if (endDigitalTouch.getState() == true){
                if (gamepad2.left_trigger > 0) OpenPump.setPower(0.8);
                else OpenPump.setPower(0);
            }

            if (gamepad2.x) Pumper.setPower(0.8);
            else if (gamepad2.a ) Pumper.setPower(-0.8);
            else Pumper.setPower(0);

            if (gamepad2.dpad_down) {
                BringUpPumpRight.setPower(-1);
                BringUpPumpLeft.setPower(1);
                /*sleep(2000);
                BringUpPumpRight.setPower(0);
                BringUpPumpLeft.setPower(0);*/
            }

            else if (gamepad2.dpad_up) {
                BringUpPumpRight.setPower(1);
                BringUpPumpLeft.setPower(-  1);
                /*sleep(2000);
                BringUpPumpRight.setPower(0);
                BringUpPumpLeft.setPower(0);*/
            }

            else
            {
                //Pumper.setPower(0);
                BringUpPumpRight.setPower(0);
                BringUpPumpLeft.setPower(0);
            }



            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

