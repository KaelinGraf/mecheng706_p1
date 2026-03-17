#ifndef STATE_H
#define STATE_H

class Tiller;
struct StateResult;

class State
{
public:
    enum Name
    {
        INITIALISING,
        FIND_CORNER,
        TILL,
        TURN,
        STOPPED,

        // leave last, gives access to length of states
        NUM_STATES,
    };

    State(Name name, Tiller *tiller) : name_(name), tiller_(tiller) {};
    virtual ~State() {};

    inline Name getState() const { return name_; }

    virtual void begin() = 0;
    virtual void end() = 0;
    virtual void poll() {};

protected:
    Name name_;
    Tiller *tiller_;
};

#endif // STATE_H
