/*
 * (c) 2022 Yunjae Lim <launius@gmail.com>
 *
 * func 1. print all prime factors of a given number
 *
 * func 2. print all prime numbers within a range
 *
 * func 3. print all prime numbers less than a given number
 *			using the Sieve of Eratosthenes
 *
 */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

void prime_factor(int n)
{
	int i = 2;

	printf("%s: %d = ", __func__, n);

	while (n > 1) {
		if (n % i == 0) {
			n /= i;
			printf("%d ", i);
		}
		else
			i++;
	}

	printf("\n");
}

void prime_number(int n)
{
	int num, i;

	printf("%s: <= %d\n", __func__, n);
	
	if (n > 1)
		printf("%d ", 2);
		
	for (num = 3 ; num <= n ; num += 2) {
		for (i = 3 ; i*i <= num ; i += 2)
			if (num%i == 0)
				break;
		
		if (i*i > num)
			printf("%d ", num);
	}

	printf("\n");
}

#define MAX_N	10001
void prime_number_Sieve_of_Eratosthenes(int n)
{
	bool is_prime[MAX_N];
	int i, j;

	printf("%s: <= %d\n", __func__, n);
	
	memset(is_prime, true, sizeof(is_prime));
	
	for (i = 2 ; i*i <= n ; i++)	// check-out to i*i is enough.
		if (is_prime[i])
			for (j = i*i ; j <= n ; j += i)
				is_prime[j] = false;
	
	for (i = 2 ; i <= n ; i++)
		if (is_prime[i])
			printf("%d ", i);
	
	printf("\n");
}

int main(int argc, char *argv[])
{
	int n;

	scanf("%d", &n);

	prime_factor(n);
	
	prime_number(n);

	prime_number_Sieve_of_Eratosthenes(n);

	return 0;
}
