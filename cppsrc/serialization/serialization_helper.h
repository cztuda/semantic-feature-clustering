#pragma once

// #include <boost/archive/polymorphic_text_iarchive.hpp>
// #include <boost/archive/polymorphic_text_oarchive.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/export.hpp>

#include <istream>
#include <ostream>



#define PUBLIC_STREAM_SERIALIZATION_DEFINITION(T)           \
void T::serialize(const T* f, std::ostream& s) {            \
    boost::archive::polymorphic_text_oarchive  ar(s);       \
    ar << f;                                                \
}                                                           \
                                                            \
bool T::unserialize( T** f, std::istream& s) {              \
    T* tmp = new T();                                                 \
    boost::archive::polymorphic_text_iarchive ia(s);        \
    ia >> tmp;                                              \
    *f = tmp;                                               \
    return true;                                            \
}       


#define PUBLIC_STREAM_SERIALIZATION_DECLARATION(T)          \
static void serialize(const T* f, std::ostream& s);         \
static bool unserialize( T** f, std::istream& s);



#define PUBLIC_STREAM_SERIALIZATION(T)                      \
static void serialize(const T* f, std::ostream& s) {        \
    boost::archive::polymorphic_text_oarchive  ar(s);       \
    ar << f;                                                \
}                                                           \
                                                            \
static bool unserialize( T** f, std::istream& s) {          \
    T* tmp = new T();                                       \
    boost::archive::polymorphic_text_iarchive ia(s);        \
    ia >> tmp;                                              \
    *f = tmp;                                               \
    return true;                                            \
} 
