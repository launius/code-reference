#include <iostream>
using namespace std;

// C code to implement the strstr() function
char* my_strstr(char *str, char *substr)
{
	while (*str) {
		char *s1 = str;
		char *s2 = substr;
		
		while (*s1 && *s2 && *s1 == *s2) {
			s1++;
			s2++;
		}

		if (*s2 == '\0')
			return str;

		str++;
	}

	return nullptr;
}

int main()
{
	char str[] = "This is a abcdef sample string";
	char substr[] = "abc";

	cout << my_strstr(str, substr) << endl;
}
