#ifndef STATE_H
#define STATE_H

class Fsm;
struct StateResult;

class State {
public:
    enum Name {
        FIND_CORNER,
        TILL,
        TURN,

        // leave last, gives access to length of states
        NUM_STATES,
    };

    State(Name name, Fsm *fsm) : name_(name), fsm_(fsm) {};
    virtual ~State() {};

    inline Name getState() const { return name_; }

    virtual void begin() = 0;
    virtual void end() = 0;
    virtual StateResult* poll() { return nullptr; };

protected:
    Name name_;
    Fsm *fsm_;
};

struct StateResult {
    State::Name next_state;
    void *data;
};

#endif // STATE_H
