#include "../global.h"
#include "string_process.h"


void __M_Log(const char* expr_str, const char* file, int line, const char* Msg)
{
    const std::string file_string(file);
    const std::string delim("/");
    const std::vector<std::string> vector_file_name = split(file_string, delim);
    const int nb_tokens = vector_file_name.size();
    const std::string folder = vector_file_name[nb_tokens-2];
    const std::string filename = vector_file_name[nb_tokens-1];

    std::cout << folder << "/" << filename << " - l. " << line << " | " <<  (std::strcmp(Msg, "") ? Msg :  expr_str) << ": ";
}

template<typename T>
void print_iterator(const T& array_like){
    std::cout << "Array : { ";
    for (auto it = array_like.begin(); it!=array_like.end(); it++){
        std::cout << *it << ", ";
    }
    std::cout << "}" << std::endl;
}