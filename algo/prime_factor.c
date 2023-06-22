/*
 * (c) 2022 Yunjae Lim <launius@gmail.com>
 *
 * print all prime factors of a given number
 *
 */

#include <stdio.h>

void prime_factor(int n)
{
	int i = 2;

	while (n > 1) {
		if (n % i == 0) {
			n /= i;
			printf("%d ", i);
		}
		else
			i++;
	}
}

int main(int argc, char *argv[])
{
	int n;

	scanf("%d", &n);
	printf("%d = ", n);

	prime_factor(n);
	printf("\n");

	return 0;
}

