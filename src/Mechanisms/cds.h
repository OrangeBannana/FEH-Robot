#include <FEHIO.h>

enum CDSColor {
    Red = 0,
    Blue = 1,
    Off = 2
};

class cds {

    public:
        cds(FEHIO::FEHIOPin pin) : CDS(pin) {};

        void update() {
            lastReadValue = CDS.Value();

            float redDist = abs(lastReadValue - redVal);
            float blueDist = abs(lastReadValue - blueVal);
            float offDist = abs(lastReadValue - offVal);

            if (redDist < blueDist && redDist < offDist) {
                lastReadColor = CDSColor::Red;
            } else if (blueDist < redDist && blueDist < offDist) {
                lastReadColor = CDSColor::Blue;
            } else {
                lastReadColor = CDSColor::Off;
            }
        };

        float value() {return lastReadValue;};
        CDSColor Color() {return lastReadColor;};


    private:
    
        AnalogInputPin CDS;

        float lastReadValue;
        CDSColor lastReadColor;

        float redVal = 0.415;
        float offVal = 3.162;
        float blueVal = 0.0;
};