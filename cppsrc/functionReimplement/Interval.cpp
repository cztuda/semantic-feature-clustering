
#include <boost/serialization/export.hpp>
#include <boost/archive/polymorphic_text_oarchive.hpp>
#include <boost/archive/polymorphic_text_iarchive.hpp>

#include "Interval.h"

BOOST_CLASS_EXPORT_IMPLEMENT(ofc::functionLib::Interval)


PUBLIC_STREAM_SERIALIZATION_DEFINITION(ofc::functionLib::Interval)
