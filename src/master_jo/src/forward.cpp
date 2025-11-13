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

    if (current_flag != previous_vision_flag_)
    {
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

    previous_vision_flag_ = current_flag;
  }

  void Forward::stateFinished()
  {
    move(false);
  }
}