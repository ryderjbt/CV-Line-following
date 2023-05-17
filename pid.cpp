// Include files for required libraries
#include <stdio.h>
#include <time.h>

#include "opencv_aee.hpp"
#include "main.hpp"     // You can use this file for declaring defined values and functions
#include "pi2c.h"

float prevError = 0;

Pi2c car(0x22); // Configure the I2C interface to the Car as a global variable

void setup(void)
{
    setupCamera(320, 240);  // Enable the camera for OpenCV
}

int main( int argc, char** argv )
{
    setup();    // Call a setup function to prepare IO and devices

    cv::namedWindow("lineView");
    cv::namedWindow("croppedLineView1");
    cv::namedWindow("croppedLineView2");
    cv::namedWindow("croppedLineView3");
    cv::namedWindow("croppedLineView4");
    cv::namedWindow("croppedLineView5");
    cv::namedWindow("croppedLineView6");
    cv::namedWindow("croppedLineView7");
    cv::namedWindow("croppedLineView8");

    while(1)    // Main loop to perform image processing
    {
        Mat frame;
        Mat hsvFrame;
        Mat lineView;
        Mat rotatedFrame;
        int values[8];
        while(frame.empty()){
            frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable
        }

        cv::Point2f centre((319)/2, (239)/2);
        cv::Mat flip = cv::getRotationMatrix2D(centre,180,1);
        cv::warpAffine(frame, rotatedFrame, flip, frame.size());
        //Display the image in the window
        cv::imshow("rotated", rotatedFrame);

        cvtColor(rotatedFrame, hsvFrame, COLOR_BGR2HSV);
        inRange(hsvFrame, Scalar(0,0,0), Scalar(255,255,50), lineView);
        cv::imshow("lineView", lineView);

        Mat cropped1 = lineView(Rect(0, 0, 40, 1));
        values[0] = countNonZero(cropped1);
        cv::imshow("croppedLineView1", cropped1);

        Mat cropped2 = lineView(Rect(40, 0, 40, 1));
        values[1] = countNonZero(cropped2);
        cv::imshow("croppedLineView2", cropped2);

        Mat cropped3 = lineView(Rect(80, 0, 40, 1));
        values[2] = countNonZero(cropped3);
        cv::imshow("croppedLineView3", cropped3);

        Mat cropped4 = lineView(Rect(120, 0, 40, 1));
        values[3] = countNonZero(cropped4);
        cv::imshow("croppedLineView4", cropped4);

        Mat cropped5 = lineView(Rect(160, 0, 40, 1));
        values[4] = countNonZero(cropped5);
        cv::imshow("croppedLineView5", cropped5);

        Mat cropped6 = lineView(Rect(200, 0, 40, 1));
        values[5] = countNonZero(cropped6);
        cv::imshow("croppedLineView6", cropped6);

        Mat cropped7 = lineView(Rect(240, 0, 40, 1));
        values[6] = countNonZero(cropped7);
        cv::imshow("croppedLineView7", cropped7);

        Mat cropped8 = lineView(Rect(280, 0, 40, 1));
        values[7] = countNonZero(cropped8);
        cv::imshow("croppedLineView8", cropped8);

        int i;
        float offset = -139.5;
        float sumOfCounts = 0;
        float sumOfWeights = 0;

        for (i = 0; i <= 7; i++){
            offset += 40;
            values[i] = 40 - values[i];
            sumOfCounts += values[i] * offset;
            sumOfWeights += values[i];
        }

        float weightedAverage = sumOfCounts / sumOfWeights;
        float error = (37 - weightedAverage);

        float integral;
        // Calculate integral and derivative terms for PID controller
        float dt = 1.0 / 30; // assuming camera runs at 30 fps
        integral += error * dt;
        float derivative = (error - prevError) / dt;
        // Calculate output of PID controller
        float Kp = 10, Ki = 0, Kd = 0;
        float u = ((Kp * error) + (Ki * integral) + (Kd * derivative));

        // Send output to ESP32 over I2C
        uint8_t output_byte = (uint8_t)output;
        arduino.i2cRead(car, output_byte);

        // Update previous error
        prevError = error;

        int key = cv::waitKey(1);   // Wait 1ms for a keypress (required to update windows)

        key = (key==255) ? -1 : key;    // Check if the ESC key has been pressed
        if (key == 27)
            break;
    }
}


	closeCV();  // Disable the camera and close any windows

	return 0;




