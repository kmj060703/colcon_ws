#ifndef FORWARD_HPP
#define FORWARD_HPP

#include "player.hpp"

namespace master_jo
{

    class Forward : public Player
    {
    public:
        Forward(std::shared_ptr<master_jo::MasterRcko> master);

        void stateInitial();
        void stateReady();
        void stateSet();
        void statePlay();
        void stateFinished();
    // 상태 관리를 위한 boolean 플래그 사용
    bool is_waiting_for_stop_ = false; 
    
    int previous_vision_flag_ = -1;
    int target_vision_flag_ = -1; // 정지 후 수행할 행동을 결정하는 플래그


    };

}

#endif