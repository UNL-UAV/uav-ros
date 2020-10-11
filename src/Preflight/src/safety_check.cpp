#include "safety_check.hpp"

//Class that holds methods that check certain safety parameters

    safety_check::safety_check(state_msg msg){
        this->drone_state = msg;
    }

    void safety_check::update_state(state_msg msg){
        this->drone_state = msg;
    }

    bool safety_check::armed(){
        return this->drone_state.armed;
    }

    bool safety_check::offboard_mode(){
        if(this->drone_state.mode == "OFFBOARD") return true;
        return false;
    }

    bool safety_check::connected(){
        return this->drone_state.connected;
    }