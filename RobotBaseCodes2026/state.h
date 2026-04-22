#ifndef STATE_H
#define STATE_H

class Tiller;
struct StateResult;

struct TillData {
    float distance; // distance ultrasonic sensor is from wall (cm)
    bool drive_foward;    // if true drive foward, false = backward
    bool from_homing;
};

class State
{
public:
    enum Name
    {
        INITIALISING,
        HOMING,
        TILL,
        TURN,
        STRAFE,
        STOPPED,

        // leave last, gives access to length of states
        NUM_STATES,
    };

    State(Name name, Tiller *tiller) : name_(name), tiller_(tiller) {};
    virtual ~State() {};

    inline Name getState() const { return name_; }

    virtual void begin() = 0;
    virtual void begin(TillData data) {};
    virtual void end() = 0;
    virtual void poll() {};

protected:
    Name name_;
    Tiller *tiller_;
};

#endif // STATE_H
