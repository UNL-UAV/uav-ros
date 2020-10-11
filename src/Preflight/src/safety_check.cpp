#include "safety_check.hpp"

//Class that holds methods that check certain safety parameters

class safety_check{
    private:    

    state_msg drone_state;

    public:

    safety_check(const state_msg::ConstPtr& msg){
        drone_state = *msg;
    }

    void update_state(const state_msg::ConstPtr& msg){
        drone_state = *msg;
    }

    bool armed(){
        return drone_state.armed;
    }

    bool offboard_mode(){
        if(this.drone_state.state == "OFFBOARD") return true;
        return false;
    }



}