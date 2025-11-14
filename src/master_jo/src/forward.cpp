#include "../include/master_jo/forward.hpp"
#include <unistd.h>
namespace master_jo
{

  using namespace std;

  Forward::Forward(std::shared_ptr<master_jo::MasterRcko> master)
      : Player(master)

  {
    cout << endl
         << "HELLO!!" << endl
         << endl;
  }

  void Forward::stateInitial()
  {
    move(false);
  }

  void Forward::stateReady()
  {
    move(false);
  }

  void Forward::stateSet()
  {
    move(false);
  }

  void Forward::statePlay()
  {

    int current_flag = master->vision.flag;
    int current_vision_flag = master->vision.flag;

    // "정지 대기" 상태일 경우의 로직
    if (is_waiting_for_stop_ == true)
    {

      walkStart(0,0,0);
      sleep(3);
      walkStop();
      sleep(3);
    }

    if (current_flag == 0)
    {
      walkStart(10, 0, 0);
    }
    else if (current_flag == 1)
    {
      cout << endl
           << "호잇짜 flag 1" << endl
           << endl;
      walkStart(0, -20, 0);
    }
    else if (current_flag == 2)
    {
      cout << endl
           << "짜잇호 flag 2" << endl
           << endl;
      walkStart(0, 20, 0);
    }

      walkStop();

      // 로봇이 완전히 정지했는지 확인
      if (master->ikEnd.ikend == 1)
      {
        cout
            << "Robot stop confirmed. Executing next action." << endl;

        // 정지가 확인되면, 저장해둔 target_vision_flag_에 따라 다음 행동을 설정
        if (target_vision_flag_ == 0)
        {
          cout
              << "Starting forward walk." << endl;

          walkStart(10, 0, 0);
        }
        else if (target_vision_flag_ == 1)
        {
          cout
              << "호잇짜 flag 1" << endl;

          walkStart(0, -20, 0);
        }
        else if (target_vision_flag_ == 2)
        {
          cout
              << "짜잇호 flag 2"
              << endl;
          walkStart(0, 20, 0);
        }
        is_waiting_for_stop_ = false;
      }
    }
    // "정지 대기" 상태가 아닐 경우의 로직 (평상시)
    else
    {
      // 비전 플래그에 변화가 감지되었는지 확인
      if (current_vision_flag != previous_vision_flag_)
      {
        cout
            << "Vision flag changed. Initiating stop." << endl;

        // 1. "정지 대기" 상태로 전환
        is_waiting_for_stop_ = true;

        // 2. 정지 후 수행할 행동(플래그)을 저장
        target_vision_flag_ = current_vision_flag;

        // 3. 첫 번째 정지 명령
        walkStop();
      }

      // --- 모든 경우에 매 주기마다 실행되는 부분 ---

      previous_vision_flag_ = current_vision_flag;
    }
  }
  void Forward::stateFinished()
  {
    move(false);
  }
}