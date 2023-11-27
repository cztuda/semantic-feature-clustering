#pragma once

#include <memory>
#include <istream>

#include "../dlc/dlclass.hpp"
#include "../models/optimalcontrolproblem.h"

namespace ofc::models {

inline ofc::models::OptimalControlProblem* createModel(const std::string& name)
{
    auto dlModelclass = new DLClass<ofc::models::OptimalControlProblem>("lib"+name+".so");
    ofc::models::OptimalControlProblem* problem = dlModelclass->make_obj();
    delete dlModelclass;
    return problem;
}

inline ofc::models::OptimalControlProblem* unserializeModel(std::istream& s)
{
    std::string libname;
    s >> libname;
    auto dlModelclass = new DLClass<ofc::models::OptimalControlProblem>(libname);
    ofc::models::OptimalControlProblem* problem = dlModelclass->make_obj(s);
    delete dlModelclass;
    return problem;
}

inline void serializeModel(std::ostream& s, const ofc::models::OptimalControlProblem* p)
{
    std::string pname = p->getThisLibraryPathname();
    
    // cut name from pathname
    pname = pname.substr(pname.find_last_of("/\\")+1);
    
    s << pname << " ";
    p->serializeOCP(p, s);
}

inline ofc::models::OptimalControlProblem* copyModel(const std::string& name, const ofc::models::OptimalControlProblem& p){
    auto dlModelclass = new DLClass<ofc::models::OptimalControlProblem>("lib"+name+".so");
    ofc::models::OptimalControlProblem* p2 = dlModelclass->make_obj(p);
    delete dlModelclass;
    return p2;
}

}
