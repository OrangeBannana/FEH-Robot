// Implementation of a PDFL controller

// PDFL controller is a alternative to a PID controller that omits the I term
// and achieves similar functionality with a F and L term
// Takes in an arbitrary current system value and target value and returns a arbitrary output
// to try and move system state such that the current value = target value

// kP -> P gain
// Proportional to error in the direction of error
// Determines aggressiveness of correction

// kD -> D gain
// Proportional to the derivitive of the error opposite the direction of error
// Reduces corrective output when current value is quickly approaching target value & dampens oscillations

// kF -> F gain (Systems with constant external forces)
// Constant value with constant direction
// Used to counteract constant external forces such as gravity

// kL -> L gain (Most systems)
// Minimum magnitude of output in the direction of error
// Used to overcome forces such as static friction, improving controller performance near target value

// Tuning Guide:
// 1. Set all gains to 0
// 2. If kF is applicable, increase slowly until system can be manually moved to any state and maintain it
// 3. If kL is applicable, increase slowly until system moves as slowly as possible in all states
// 4. Increase kP slowly until system reaches desired state with acceptable accuracy and speed, if significant oscillation occours damp it manually during this process
// 5. Increase kD slowly until target positions can be achieved with minimal oscillation and acceptable accuracy and speed
// 6. Repeat steps 4 and 5 iteratively in any order to achieve desired behavior. It is not reccomended to adjust kF and kL arbitrary order

// Usage Note for systems with a cyclic/periodic domain (EX: angle control):
// It is reccomended to manually calculate error outside of the controller
// and set target controller value to 0, and then pass error as current value

#include <FEHUtility.h>
#include <FEHIO.h>

#define S_TO_MS 1000.0f

class PDFL {
    public:
    
        PDFL(float p, float i, float d, float f);
        void setPIDF(float p, float i, float d, float f);
        void setTarget(float target);
        void update(float value);

        float output = 0;

    private:
        // Constants
        float
            kP = 0,
            kD = 0,
            kF = 0,
            kL = 0;
        // Track Current Iteration State
        float
            target = 0,
            time = TimeNow() * S_TO_MS,
            error = 0,
            P = 0,
            D = 0;
        // Track Previous Iteration State
        float
            timePrevious = 0,
            errorPrevious = 0;
        // Track Change In State Between Iteration 
        float
            deltaTime = 0,
            deltaError = 0,
            deltaErrorDeltaTime = 0;
};