#include "safety_check.hpp"

//Class that holds methods that check certain safety parameters

class safety_check{
    private:    


    public:
    
    state_msg drone_state;

    bool armed(const state_msg::ConstPtr& msg){
        return msg.armed
    }

    bool offboard_mode(const state_msg::ConstPtr& msg){
        if(msg.state == "OFFBOARD") return true;
        return false;
    }

    
}