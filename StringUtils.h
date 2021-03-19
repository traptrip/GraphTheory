//
// Created by Андрей on 02.03.2021.
//

#include <string>
#include <vector>


#ifndef GRAPH_STRINGUTILS_H
#define GRAPH_STRINGUTILS_H

class StringUtils
{
public:
    static std::vector<std::string> split(const std::string& str, const char& sep);
};

std::vector<std::string> StringUtils::split(const std::string &str, const char &sep)
{
    std::vector<std::string> tokens;
    std::string token;
    for (char i : str)
    {
        if (i == sep)
        {
            tokens.push_back(token);
            token = "";
        }
        else
            token += i;
    }
    if (!token.empty())
        tokens.push_back(token);

    return tokens;
}

#endif //GRAPH_STRINGUTILS_H
