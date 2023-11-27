#pragma once

#include <unordered_map>
#include <string>
#include <vector>

namespace ofc::util {

class ArgumentParser {
    /**
     * Provides easy registration and parsing of command line arguments (flags, words and arbitrary input).
     * - The attribute <numberOfExpectedInputs> gives the number of required regular arguments (like in 'grep "text"'). Note that there may be more arguments given.
     * - Options can be provided as flags ('grep -r "text"') or as words ('grep --recursive "text"'). One of them can be not set (register with empty string ""), but at least one of them must be given.
     * - Argument options are options that require a second consecutive options (like in 'exec --directory "/here" "argument"').
     * - The default value for all options is false. For all argument options, a custom default value must be given.
     * All unknown arguments are silently ignored. The number of ignored arguments is stored in <numberOfIgnoredArguments>.
     * If some error occurs, the message is stored in <lasterror>
     */
    
public:
    ArgumentParser();
    
    bool registerOption(const std::string word, const std::string flag);
    bool registerArgumentOption(const std::string word, const std::string flag, const std::string defaultValue);
    
    int parseArguments(const int argc, const char* argv[]);
    
    void* getInput(size_t index);
    int numberOfRegularInputs() const {return inputs.size();};
    std::string getArgumentOption(const std::string option);
    bool getOption(const std::string option);
    int numberOfIgnoredArguments;
    
    std::string lasterror;
    void clearError() {lasterror = "";};
private:
    std::vector<void*> inputs;
    std::vector<std::string> argumentOptions;
    std::vector<bool> options;
    
    std::unordered_map<std::string, int> optionFlags;
    std::unordered_map<std::string, int> optionWords;    
    std::unordered_map<std::string, bool> optionHasArgument;
    
    bool checkRegisterInput(int &result, const std::string& word, const std::string& flag);
};



}
