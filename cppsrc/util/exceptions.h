#pragma once

#include <stdexcept>

namespace ofc::exception {
    
    class Error : public std::exception {
    public:
        Error(){}
        virtual ~Error(){}
        Error(std::string message) {
            msg = message;
        }        
        virtual std::string getMessage() const {
            return msg;
        }        
        virtual void setMessage(const std::string message) {
            msg = message;
        }        
        virtual const char* what() const noexcept override {
            return "Unknown exception";
        }
        
    private:
        std::string msg;
    };
    
    class NotImplementedError : public Error {
    public:
        NotImplementedError() : Error() {}
        NotImplementedError(std::string message) : Error(message){}
        virtual const char* what() const noexcept override {
            return "Functionality not implemented.";
        }
    };
    
    class InconsistentInputError : public Error {
    public:
        InconsistentInputError() : Error() {}
        InconsistentInputError(std::string message) : Error(message){}
        virtual const char* what() const noexcept override {
            return "Input dimensions are not consistent.";
        }
    };
    
    
    class FileError : public Error {
    public:
        FileError() : Error() {}
        FileError(std::string file) : Error("File error " + file), fileName(file) {}
        virtual ~FileError() {}
        virtual std::string getMessage() const override {
            return "File error in: " + getFileName();
        }
        virtual void setFileName(const std::string file) {
            fileName = file;
        }
        std::string getFileName() const {
            return fileName;
        }
        virtual const char* what() const noexcept override {
            return "File error";
        }
    private:
        std::string fileName;
    };
    
    class FileNotFoundError : public FileError {
    public:
        FileNotFoundError(): FileError() {}
        FileNotFoundError(std::string file) : FileError(file) {}
        virtual ~FileNotFoundError() {}
        virtual std::string getMessage() const override {
            return "File " + getFileName() + "has not been found.";
        }
        virtual const char* what() const noexcept override {
            return "File not found error";
        }
    };
    
    
    class MatlabInterfaceError : public Error {
    public:
        MatlabInterfaceError() : Error() {}
        MatlabInterfaceError(std::string message) : Error(message) {}
        virtual ~MatlabInterfaceError() {}
        virtual const char* what() const noexcept override {
            return "MATLAB interface error";
        }
    };
    
    
    class MatlabInterfaceReadError : public MatlabInterfaceError {
    public:
        MatlabInterfaceReadError(std::string fieldName) : MatlabInterfaceError(fieldName) {
            setFieldName(fieldName);
        }    
        virtual ~MatlabInterfaceReadError(){}
        
        void setFieldName(const std::string name) {
            fieldName = name;
        }
        
        std::string getFieldName() const {
            return fieldName;
        }
        
        std::string getMessage() const override {
            return "Error in MATLAB interface reading " + getFieldName() + ".";
        }
        
        virtual const char* what() const noexcept override {
            return "MATLAB interface read error";
        }
    private:
        std::string fieldName;
    };
    
    
    class FileReadError : public FileError {  
    public:
        FileReadError(std::string file) : FileError("(reading) " + file) {
            setFileName(file);
        }
        virtual ~FileReadError() noexcept {}
        virtual  std::string getMessage() const override {
            return "Unable to read file: " + getFileName();
        }
        
        virtual const char* what() const noexcept override {
            return "File read error";
        }
    };
    
    
    class StreamFormatError : public Error {
    public:
        StreamFormatError(std::string field) : Error("Error when expecting field: " + field) {
            setFieldName(field);
        }
        StreamFormatError(std::string field, std::string note) : Error(field + ": " + note) {
            setFieldName(field);
        }
        virtual ~StreamFormatError() {}
        std::string getFieldName() const {
            return fieldName;
        }
        void setFieldName(const std::string name) {
            fieldName = name;
        }
        virtual const char* what() const noexcept override {
            return "Stream format error";
        }
        
    private:
        std::string fieldName;
    };
    
    
    class FileFormatError : public FileError {
    public:
        FileFormatError(std::string fileName, StreamFormatError& err) : FileError(err.getMessage() + " in " + fileName) {}
        FileFormatError(std::string fileName, MatlabInterfaceReadError& err) : FileError(err.getMessage() + " in " + fileName) {}
        FileFormatError(std::string fileName) : FileError("File format error in  " + fileName) {}
        virtual ~FileFormatError() {}
        virtual const char* what() const noexcept override {
            return "File format error";
        }
        
    };
}
