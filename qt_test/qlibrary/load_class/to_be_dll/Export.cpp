#include "Export.h"
#include <vector>
#include "Interface.h"
#include "VenderAdd.h"

std::vector<Interface *> vecAllResource;

Interface * GetInstance(){
    Interface *manager = new VenderAdd();
    vecAllResource.push_back(manager);
    return manager;
}

void ReleaseInstance(){
    for(int index = vecAllResource.size() -1; index >= 0; index--){
      delete vecAllResource.at(index);
    }
}

