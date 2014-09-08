/**
   Proportional Gain coefficient – you quadcopter can fly relatively stable without other parameters but this one. This coefficient determines which is more important, human control or the values measured by the gyroscopes. The higher the coefficient, the higher the quadcopter seems more sensitive and reactive to angular change. If it is too low, the quadcopter will appear sluggish and will be harder to keep steady. You might find the multicopter starts to oscillate with a high frequency when P gain is too high.

   Integral Gain coefficient - this coefficient can increase the precision of the angular position. For example when the quadcopter is disturbed and its angle changes 20 degrees , in theory it remembers how much the angle has changed and will return 20 degrees. In practice if you make your quadcopter go forward and the force it to stop, the quadcopter will continue for some time to counteract the action. Without this term, the opposition does not last as long. This term is especially useful with irregular wind, and ground effect (turbulence from motors). However, when the I value gets too high your quadcopter might begin to have slow reaction and a decrease effect of the Proportional gain as consequence, it will also start to oscillate like having high P gain, but with a lower frequency.

   Derivative Gain coefficient - this coefficient allows the quadcopter to reach more quickly the desired attitude. Some people call it the accelerator parameter because it amplifies the user input. It also decrease control action fast when the error is decreasing fast. In practice it will increase the reaction speed and in certain cases an increase the effect of the P gain.  D gain makes the quadcopter more sensitive, which means if your multicopter oscillates, this parameter will make oscillations worse, so be careful.

Aerobatic flight:

    Requires a slightly higher P
    Requires a slightly lower I
    Increase D

Gentle smooth flight:

    requires a slightly lower lower P
    Requires a slightly higher I
    Decrease D

*/

#define ROLL_PID_KP  0.250
#define ROLL_PID_KI  0.950
#define ROLL_PID_KD  0.011
#define ROLL_PID_MIN  -50.0
#define ROLL_PID_MAX  50.0

#define PITCH_PID_KP  0.250
#define PITCH_PID_KI  0.950
#define PITCH_PID_KD  0.011
#define PITCH_PID_MIN  -50.0
#define PITCH_PID_MAX  50.0

#define YAW_PID_KP  0.680
#define YAW_PID_KI  0.500
#define YAW_PID_KD  0.0001
#define YAW_PID_MIN  -50.0
#define YAW_PID_MAX  50.0

#define ANGLEX_KP 5.0
#define ANGLEX_KI 0.02
#define ANGLEX_KD -0.015
#define ANGLEX_MIN -100.0
#define ANGLEX_MAX 100.0

#define ANGLEY_KP 5.0
#define ANGLEY_KI 0.02
#define ANGLEY_KD -0.015
#define ANGLEY_MIN -100.0
#define ANGLEY_MAX 100.0
