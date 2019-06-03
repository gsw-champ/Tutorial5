#include "Grasp_Action.h"


void Grasp::Grasp_CB(){
        goal_ = grasp_.acceptNewGoal() ->goal;
}




