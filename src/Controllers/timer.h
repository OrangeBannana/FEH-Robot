#include <FEHUtility.h>

struct timer {
    public:

        void start(float length) {
            startTime = TimeNow();
            timerLength = length;
        }
    
        bool isOver() {
            return TimeNow() - startTime >= timerLength;
        }
    
        float getTime() {
            return TimeNow() - startTime;
        }

    private:
        float startTime;
        float timerLength;
};