
template <class T>
DLClass<T>::DLClass(std::string module_name) :
    module(module_name){
    shared = std::make_shared<shared_obj>();
}

template <class T>
DLClass<T>::~DLClass() {
    //close_module();
}

template <class T>
void DLClass<T>::shared_obj::close_module() {
    if(dll_handle) {
        dlclose(dll_handle);
        dll_handle = NULL;
    }
    if(create_default) create_default = NULL;
    if(create_copy) create_copy = NULL;
    if(create_unserialize) create_unserialize = NULL;
    if(destroy) destroy = NULL;
}

template <class T>
bool DLClass<T>::shared_obj::open_module(std::string module) {
    
    dll_handle = dlopen(module.c_str(), RTLD_LAZY);

    if(!dll_handle) {
        std::cerr << "Failed to open library: " << dlerror() << std::endl;
        return false;
    }

    // Reset errors
    dlerror();

    create_default = (typename T::create_default_t*) dlsym(dll_handle, "create_default");
    const char * err = dlerror();
    if(err) {
        std::cerr << "Failed to load default create symbol: " << err << std::endl;
        close_module();
        return false;
    }
    
    
    create_copy = (typename T::create_copy_t*) dlsym(dll_handle, "create_copy");
    err = dlerror();
    if(err) {
        std::cerr << "Failed to load copy create symbol: " << err << std::endl;
        close_module();
        return false;
    }
    
    create_unserialize = (typename T::create_unserialize_t*) dlsym(dll_handle, "create_unserialize");
    err = dlerror();
    if(err) {
        std::cerr << "Failed to load unserialize create symbol: " << err << std::endl;
        close_module();
        return false;
    }
    
    destroy = (typename T::destroy_t*) dlsym(dll_handle, "destroy");
    err = dlerror();
    if(err) {
        std::cerr << "Failed to load destroy symbol: " << err << std::endl;
        close_module();
        return false;
    }

    return true;
}


template <class T>
T* DLClass<T>::make_obj() {
    if(!shared->create_default || !shared->destroy) {
        if(!shared->open_module(module)) {
            return 0;
        }
    }
   
    std::shared_ptr<shared_obj> my_shared = shared;
    return shared->create_default();
}

template <class T>
T* DLClass<T>::make_obj(const T& p) {
    if(!shared->create_copy || !shared->destroy) {
        if(!shared->open_module(module)) {
            return 0;
        }
    }

//    auto create_args = ((T* (*)(Args...))create);    
    std::shared_ptr<shared_obj> my_shared = shared;
    return shared->create_copy(p);
}

template <class T>
T* DLClass<T>::make_obj(std::istream& s) {
    if(!shared->create_unserialize || !shared->destroy) {
        if(!shared->open_module(module)) {
            return 0;
        }
    }
   
    std::shared_ptr<shared_obj> my_shared = shared;
    return shared->create_unserialize(s);
}


template <class T>
DLClass<T>::shared_obj::~shared_obj() {
    close_module();
}
