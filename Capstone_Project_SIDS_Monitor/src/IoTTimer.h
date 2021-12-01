#ifndef _IoTTIMER_H_
#define _IoTTIMER_H_

class IoTTimer {

    unsigned int _timerStart, _timerTarget;

    public:
      void startTimer (unsigned int msec) {
        _timerStart = millis();
        _timerTarget = msec;
    }

    bool isTimerReady () {
      return ((millis() - _timerStart) >= _timerTarget);
    }
};

#endif // _IoTTIMER_H_