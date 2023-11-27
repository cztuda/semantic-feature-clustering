


#include "argumentparser.h"
#include <utility>

using namespace std;


namespace ofc::util {

const string ERR = "Error in ArgumentParser::";
const string ERR_WORD_OR_FLAG = ": Both word and flag are missing.";
const string ERR_UNKNOWN_OPTION = ": Given option has not been registered.";
const string ERR_EXISTING_WORD = ": Given word already exists.";
const string ERR_EXISTING_FLAG = ": Given flag already exists.";
const string ERR_INDEX = ": Given index out of bounds.";

    
ArgumentParser::ArgumentParser()
    : numberOfIgnoredArguments(0), lasterror("")
{    
}

bool ArgumentParser::checkRegisterInput(int &result, const std::string& word, const std::string& flag)
{
    result = 0;
    if(word.size() > 0) {
        result += 1;
        if(optionWords.find(word) == optionWords.end()) {
            lasterror = ERR + "checkRegisterInput" + ERR_EXISTING_WORD;
            return false;
        }
    }
    if(flag.size() > 0) {
        result += 2;
        if(optionFlags.find(flag) == optionFlags.end()) {
            lasterror = ERR + "checkRegisterInput" + ERR_EXISTING_FLAG;
            return false;
        }
    }
}


bool ArgumentParser::registerOption(const std::string word, const std::string flag)
{
    int n;
    if(!checkRegisterInput(n, word, flag)) {
        return false;
    }
    
    switch(n) {
        case 1:
            options.push_back(false);
            optionWords.insert(std::make_pair(word, options.size()-1));
            break;
        case 2:
            options.push_back(false);
            optionFlags.insert(make_pair(flag, options.size()-1));
            break;
        case 3:
            options.push_back(false);
            optionWords.insert(make_pair(word, options.size()-1));
            optionFlags.insert(make_pair(flag, options.size()-1));
            break;
        default:
            lasterror = ERR + "registerOption" + ERR_WORD_OR_FLAG;
            return false;        
    }
    optionHasArgument.insert(make_pair(word, false));
    return true;
}


bool ArgumentParser::registerArgumentOption(const std::string word, const std::string flag, const std::string defaultValue)
{
    int n;
    if(!checkRegisterInput(n, word, flag)) {
        return false;
    }
    
    switch(n) {
        case 1:
            argumentOptions.push_back(defaultValue);
            optionWords.insert(make_pair(word, argumentOptions.size()-1));
            break;
        case 2:
            argumentOptions.push_back(defaultValue);
            optionFlags.insert(make_pair(flag, argumentOptions.size()-1));
            break;
        case 3:
            argumentOptions.push_back(defaultValue);
            optionWords.insert(make_pair(word, argumentOptions.size()-1));
            optionFlags.insert(make_pair(flag, argumentOptions.size()-1));
            break;
        default:
            lasterror = ERR + "registerArgumentOption" + ERR_WORD_OR_FLAG;
            return false;        
    }
    optionHasArgument.insert(make_pair(word, true));
    return true;
}


int ArgumentParser::parseArguments(const int argc, const char * argv[])
{
    int unknownArguments = 0;
    string label; // use this to store the current console argument
    int type = 0; // 0=unknown, 1=argOption, 2=flag, 3=word, 4=regArg
    int index; // store the index here if the option requires a second argument
    unordered_map<string, int>::const_iterator it; // temporarily store the result of the searches here
    for(int i = 1; i < argc; ++i){
        
        label = string(argv[i]);
        if(type != 1) {            
            if(label.size() > 2 && label[1] == '-' && label[0] == '-') {
                label = label.substr(2);
                type = 3;
            }
            else if(label.size() > 1 && label[0] == '-') {
                label = label.substr(1);
                type = 2;
            }
            else {
                type = 4;
            }        
        }
        
        switch(type) {
            case 1: // argOption
                argumentOptions[index] = label;
                type = 0; // reset!
                break;
                
            case 2: // flag
                if((it = optionFlags.find(label)) != optionFlags.end()) {
                    if(optionHasArgument[label])
                        options[it->second] = true;
                    else
                        index = it->second;
                }
                else {
                    unknownArguments++;
                }
                break;
                
            case 3: // word
                if((it = optionWords.find(label)) != optionWords.end()) {
                    if(optionHasArgument[label])
                        options[it->second] = true;
                    else
                        index = it->second;
                }
                else {
                    unknownArguments++;
                }
                break;
                
            case 4: // regular argument
                inputs.push_back((void*)argv[i]);
                break;
            default: // unknown
                break; // this should be impossible
        }
        
        numberOfIgnoredArguments = unknownArguments;
        if(unknownArguments > 0) {
            lasterror = "Warning in ArgumentParser::parseArguments: " + std::to_string(unknownArguments) + " unknown options have been ignored.";
        }
    }
    
}


void * ArgumentParser::getInput(size_t index)
{
    if(index < inputs.size()){
        return inputs[index];
    }
    lasterror = ERR + "getInput" + ERR_INDEX;
    return 0;
}


std::string ArgumentParser::getArgumentOption(const std::string option)
{
    auto ptr1 = optionFlags.find("-"+option);
    if(ptr1 == optionFlags.end()) {
        auto ptr2 = optionWords.find("--"+option);
        if(ptr2 == optionWords.end()) {
            lasterror = ERR + "getArgumentOption" + ERR_UNKNOWN_OPTION;
            return "";
        }
        else {
            return argumentOptions[ptr2->second];
        }
    }
    else {
        return argumentOptions[ptr1->second];
    }
}


bool ArgumentParser::getOption(const std::string option)
{
    auto ptr1 = optionFlags.find("-"+option);
    if(ptr1 == optionFlags.end()) {
        auto ptr2 = optionWords.find("--"+option);
        if(ptr2 == optionWords.end()) {
            lasterror = ERR + "getOption" + ERR_UNKNOWN_OPTION;
            return false;
        }
        else {
            return options[ptr2->second];
        }
    }
    else {
        return options[ptr1->second];
    }
}



}
