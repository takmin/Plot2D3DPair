#ifndef __UTIL__
#define __UTIL__

#include <string>

std::string AskQuestionGetString(const std::string& question);
double AskQuestionGetDouble(const std::string& question);
int AskQuestionGetInt(const std::string& question);
std::string Int2String(int id);

#endif