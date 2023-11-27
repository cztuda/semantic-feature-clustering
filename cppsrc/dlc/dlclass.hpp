#ifndef DLCLASS_H
#define DLCLASS_H

#include <memory>
#include <string>
#include <dlfcn.h>
#include <iostream>

template <class T>
class DLClass {

public:
    DLClass(std::string module_name);
    ~DLClass();

    T* make_obj();
    T* make_obj(const T& p);
    T* make_obj(std::istream& s);
    
private:
    struct shared_obj {
        typename T::create_default_t *create_default = NULL;
        typename T::create_copy_t *create_copy = NULL;
        typename T::create_unserialize_t * create_unserialize = NULL;
        typename T::destroy_t *destroy = NULL;
        void * dll_handle = NULL;

        ~shared_obj();
        bool open_module(std::string module);
        void close_module();
    };

    std::string module;
    std::shared_ptr<shared_obj> shared;
};

#include "dlclass.cpp"

#endif
