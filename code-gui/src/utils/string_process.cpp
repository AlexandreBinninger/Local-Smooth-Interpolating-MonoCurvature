#include "string_process.h"

// Source: https://stackoverflow.blog/2019/10/11/c-creator-bjarne-stroustrup-answers-our-top-five-c-questions/
template<typename Delim>
std::string get_word(std::istream& ss, Delim d)
{
    std::string word;
    for (char ch; ss.get(ch); )    // skip delimiters
        if (!d(ch)) {
            word.push_back(ch);
            break;
        }
    for (char ch; ss.get(ch); )    // collect word
        if (!d(ch))
            word.push_back(ch);
        else
            break;
    return word;
}

std::vector<std::string> split(const std::string& s, const std::string& delim)
{
    std::stringstream ss(s);
    auto del = [&](char ch) { for (auto x : delim) if (x == ch) return true; return false; };

    std::vector<std::string> words;
    for (std::string w; (w = get_word(ss, del))!= ""; ) words.push_back(w);
    return words;
}