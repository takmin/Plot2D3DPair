#include "util.h"
#include <iostream>

std::string AskQuestionGetString(const std::string& question) {
	std::cout << question;
	std::string ans;
	std::cin >> ans;
	return ans;
}

int AskQuestionGetInt(const std::string& question) {
	std::string ans = AskQuestionGetString(question);
	return atoi(ans.c_str());
}


double AskQuestionGetDouble(const std::string& question) {
	std::string ans = AskQuestionGetString(question);
	return atof(ans.c_str());
}


std::string Int2String(int id)
{
	char num[64];
	sprintf(num, "%d", id);
	return std::string(num);
}
