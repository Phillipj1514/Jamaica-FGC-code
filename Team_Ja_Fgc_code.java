package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;

@TeleOp(name = "Team_Jamaica",group = "Prized_Bot") // The name of program and bot name
public class Team_Ja_Fgc_code extends LinearOpMode {
    // Declaration of the objects to be used
    private ElapsedTime period = new ElapsedTime();
    private DcMotor left_Motor;
    private DcMotor right_Motor;
    private DcMotor color_sensor_motor;
    private DcMotor color_sensor_motor2;
    private ColorSensor color_sensor;
    private Servo color_servo;
    private DcMotor linearSlide_motor1;
    private DcMotor linearSlide_motor2;
    private Servo orangeGate_servo;
//    private DcMotor blueGate;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private Servo blueGateServo;
    //variables
    private boolean elevator_on, manual_mode = false;
    public boolean bGate_open = false, oGate_open = false;
    private int color_red = 0;
    private int color_blue = 0;
    private int color_value_alpha = 0;
    private int color_value_argb = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // this method will run continuously on the processor completely using it up
        // inside here is were all the action goes
        //put the functioning code here for the robot

        //initialization of electronics
        left_Motor = hardwareMap.dcMotor.get("left_drive");
        right_Motor = hardwareMap.dcMotor.get("right_drive");
        color_sensor = hardwareMap.colorSensor.get("color_sensor");
        color_servo = hardwareMap.servo.get("col_servo");
//        blueGate = hardwareMap.dcMotor.get("blue_gate");
        orangeGate_servo = hardwareMap.servo.get("orange_gate");
        color_sensor_motor = hardwareMap.dcMotor.get("col_motor");
        color_sensor_motor2 = hardwareMap.dcMotor.get("col_motor2");
        linearSlide_motor1 = hardwareMap.dcMotor.get("linearslide1");
        linearSlide_motor2 = hardwareMap.dcMotor.get("linearslide2");
        frontRight= hardwareMap.dcMotor.get("frontright");
        frontLeft= hardwareMap.dcMotor.get("frontleft");
        blueGateServo=hardwareMap.servo.get("blueGateServo");
        //motor and sensor configuration
        right_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide_motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        color_servo.setDirection(FORWARD);
        blueGateServo.setDirection(FORWARD);
        orangeGate_servo.setDirection(FORWARD);
        left_Motor.setPower(0);            // initialize motors to zero powers
        right_Motor.setPower(0);
        frontRight.setPower(0.0);
        frontLeft.setPower(0.0);
        color_sensor_motor.setPower(0);
        color_sensor_motor2.setPower(0);
        color_servo.setPosition(0.5);
        orangeGate_servo.setPosition(0);
        blueGateServo.setPosition(0.5);


        //variable initialization
        double left = 0.0;
        double right = 0.0;
        double motion = 0.0;
        int minimum_distance = 40; // the minimum distance of the proximity / color sensor

        // Send message to tablet showing that robot is ready and wait until start is pressed
        telemetry.addData("Say", "READY TO ROLL"); //
        telemetry.update();
        waitForStart();

        //robot functioning code which would run until user press stop
        try {
            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                //auxiliary  variable declaration and initialization
                color_red = color_sensor.red();
                color_blue = color_sensor.blue();
                color_value_alpha = color_sensor.alpha();
                color_value_argb = color_sensor.argb();
                left = -gamepad1.left_stick_y;
                right = -gamepad1.right_stick_y;




                motion = -gamepad2.left_stick_y;

                //controller command function and functionality

                drive_control(left, right); // mobility code for the drive wheels

                ball_Elevator(minimum_distance,1.0); //elevator system for the balls

                sorting_System(minimum_distance,1.0,1.0,1.0); //sorting system

                blueGateOpen(0.2,1000); //function to open the blue ball storage whenever the button is clicked

                blueGateClose(0.2,3000); //function to close the blue ball storage whenever the button is clicked

                orangeGateOpen(0.7); //function to open the orange ball storage whenever the button is clicked

                orangeGateClose(0); //function to close the orange ball storage whenever the button is clicked

                linear_Slide(motion); //linear slide movement function

                // logging information to user that is necessary
                telemetry.addData("Left wheel: ", "%.2f", left);
                telemetry.addData("Right wheel:", "%.2f", right);
                telemetry.addData("color sensor: Red Channel", "%d", color_red);
                telemetry.addData("color sensor: Blue Channel", "%d", color_blue);
                telemetry.addData("color sensor: alpha Channel", "%d", color_value_alpha);
                telemetry.addData("color sensor: argb Channel", "%d", color_value_argb);

                // update the telemetry after each run to ensure your always up to date
                telemetry.update();
                // Pause for metronome tick. 40 mS each cycle = update 25 times
                // a second.
                waitForTick(40);
            }
        } catch (java.lang.InterruptedException exc) {
            return;
        } finally {
            // if there is no error while the try statement stop the motors
            left_Motor.setPower(0);
            right_Motor.setPower(0);
        }
    }

    // Modules for the various robot functions
    private void drive_control(double left, double right){
        // The drive code for the robot
        // This allow it to move at 80% of its speed moving forward and 100% in reverse
        if(left>0.8) {
            left = 0.8;
        }
        if(right>0.8) {
            right = 0.8;
        }
        left_Motor.setPower(left);
        right_Motor.setPower(right);
        frontLeft.setPower(left);
        frontRight.setPower(right);

        // let the robot brake whenever it stop moving
        if(left == 0) {
            left_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }else if (right==0){
            right_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void ball_Elevator(int minimum_distance, double power){
        //elevator system functionalities
        if (color_value_alpha <= minimum_distance) {  // check if balls are in sensing range
            if (gamepad2.b || gamepad1.right_bumper) {
                elevator_on = true; //set  true to show that the elevator is running
                startElevator(power);
            }
        } else {
            //turn the motors off if a ball is being sensed
            color_sensor_motor.setPower(0);
            color_sensor_motor2.setPower(0);
        }

        if (gamepad2.a || gamepad1.left_bumper) { // turn off button for elevator
            elevator_on = false;
            stopElevator();
        }
    }

    private void stopElevator(){
        color_sensor_motor.setPower(0);
        color_sensor_motor2.setPower(0);
    }

    private void startElevator(double power){
        color_sensor_motor.setPower(-power);
        color_sensor_motor2.setPower(-power);
    }

    private void sorting_System(int minimum_distance, double orange_ball_position, double blue_ball_position,double power){
        // The codes for the sorting action
        if (gamepad2.x) { // Turn on manual mode for the sorting system
            manual_mode = true;
            telemetry.addData("say", "Manual mode on");
        }
        if (gamepad2.y) { // turn off manual mode for the sorting sytem
            manual_mode = false;
            telemetry.addData("say", "Manual mode off");
        }
        //manual control for servo
        if (gamepad2.right_stick_x > 0.5) { // Turn the servo to the blue ball storage

            color_servo.setPosition(blue_ball_position);
            sleep(1000);
            color_servo.setPosition(0.5);
            telemetry.addData("say", "Blue ball score");
            sleep(1500);
            if (elevator_on) { // turn the elevator back on if the elevator was on
                startElevator(power);//Anika
            }
        }
        if (gamepad2.right_stick_x < -0.5) { //Turn the servo to the orange ball storage

            color_servo.setPosition(-orange_ball_position);
            sleep(1000);
            color_servo.setPosition(0.5);
            telemetry.addData("say", "Orange ball score");
            sleep(1500);
            if (elevator_on) { // turn the elevator back on if the elevator was on
                startElevator(power);//Anika
            }
        }
        // Mode to collect just blue balls
        if (gamepad1.dpad_up) { // Turn the servo to the blue ball storage and hold
            manual_mode = false;
            color_servo.setPosition(0.7);
            telemetry.addData("say", "Blue ball only mode on");
        }
        if (gamepad1.dpad_down) { // return the servo position to the neutral zone
            manual_mode = true;
            color_servo.setPosition(0.5);
            telemetry.addData("say", "Blue ball only mode off");
        }
        //automatic ball sorting system
        if (color_red > color_blue && color_value_alpha >= minimum_distance && !manual_mode && !oGate_open) {
            //If the orange ball is detected it pushes it to the orange ball storage

            color_servo.setPosition(-orange_ball_position);
            sleep(1000);
            color_servo.setPosition(0.5);
            telemetry.addData("say", "Orange ball detected");
            sleep(1500);
            if (elevator_on) { // turn the elevator back on if the elevator was on
                startElevator(power);//Anika
            }

        } else if (color_red < color_blue && color_value_alpha >= minimum_distance && !manual_mode && !bGate_open) {// gate boolean added by Anika
            //If the blue ball is detected it pushes it to the blue ball storage

            color_servo.setPosition(blue_ball_position);
            sleep(1000);
            color_servo.setPosition(0.5);
            telemetry.addData("say", "Blue ball detected");
            sleep(1500);
            if (elevator_on) { // turn the elevator back on if the elevator was on
                startElevator(power);//Anika
            }
        }

    }

    private void linear_Slide(double motion){
        // Lift system controls
        if(motion != 0){
            linearSlide_motor1.setPower(motion);
            linearSlide_motor2.setPower(motion);
        }else{
            stopLinearSlide();
        }

    }

    private void stopLinearSlide(){
        linearSlide_motor1.setPower(0.0);
        linearSlide_motor2.setPower(0.0);

        // brake mode
//        not needed
//        linearSlide_motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        linearSlide_motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void blueGateOpen(double power, int time){
        if(gamepad2.left_trigger > 0.5){
            bGate_open = true;
//            blueGate.setPower(-power);
            blueGateServo.setPosition(-1.0);
            sleep(time);
//            blueGate.setPower(0);
            stopElevator();
        }
    }

    private void blueGateClose(double power, int time){
        if(gamepad2.left_bumper) {
            bGate_open = false;
            blueGateServo.setPosition(1.0);
//            blueGate.setPower(power);
            sleep(time);
//            blueGate.setPower(0);
        }
    }

    private void orangeGateOpen(double position){
        if(gamepad2.right_trigger > 0.5) {
            oGate_open = true;
            orangeGate_servo.setPosition(position);

            stopElevator();
        }
    }

    private void orangeGateClose(double position){
        if(gamepad2.right_bumper) {
            oGate_open = false;
            orangeGate_servo.setPosition(position);
        }
    }
    private void rampdrive(){
//  SOME PROCESS
    }
    //The Processor ticking time code
    /***
     * waitForTick implements a periodic delay. However, this acts like a metronome
     * with a regular periodic tick. This is used to compensate for varying
     * processing times for each cycle. The function looks at the elapsed cycle time,
     * and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     */
    private void waitForTick(long periodMs) throws java.lang.InterruptedException {
        long remaining = periodMs - (long) period.milliseconds();
// sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            Thread.sleep(remaining);
        }
// Reset the cycle clock for the next pass.
        period.reset();
    }
}


