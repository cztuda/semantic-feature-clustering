
#include <boost/archive/polymorphic_text_oarchive.hpp>
#include <boost/archive/polymorphic_text_iarchive.hpp>

#include "Function.h"

BOOST_CLASS_EXPORT_IMPLEMENT(ofc::functionLib::Function)


void ofc::functionLib::Function::serialize(const Function* f, std::ostream& s) {
    boost::archive::polymorphic_text_oarchive  ar(s);
    ar << f;
}


bool ofc::functionLib::Function::unserialize( Function** f, std::istream& s) {
    Function* tmp = new Function();
    boost::archive::polymorphic_text_iarchive ia(s);
    ia >> tmp;
    *f = tmp;
    return true;
}
