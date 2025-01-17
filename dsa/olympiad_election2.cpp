/*
 * (c) 2023 Yunjae Lim <launius@gmail.com>
 *
 * Australian Informatics Olympiad 2022
 *
 * Election II : https://orac2.info/problem/aio22election/
 *
 * Election time is here again, and you are in charge of determining the outcome. There were N voters
 * this year, each voting for one of three candidates: A, B or C. Which candidate received the most votes
 * and therefore won the election? Or is there a tie this year?
 *
 * Input
 *	• The first line of input contains the integer N.
 *	• The second line of input contains a string of N characters representing the votes.
 *
 * Output
 *	If some or all of the candidates are tied for the most votes, your program must output T (for tie).
 *	Otherwise, it must output the winner.
 *
 * Sample Input 1
	4
	BBAC
 *
 * Sample Output 1
	B
 *
 */

#include <cstdio>

/* N is the number of votes. */
int N;

/* votes contains the sequence of votes. */
char votes[100005];

char answer;

int main(void) {
    /* Open the input and output files. */
    FILE *input_file = fopen("elecin.txt", "r");
    FILE *output_file = fopen("elecout.txt", "w");

    /* Read the value of N and the votes from the input file. */
    fscanf(input_file, "%d", &N);
    fscanf(input_file, "%s", votes);

    /*
     * TODO: This is where you should compute your solution. Store the winning
     * candidate ('A', 'B' or 'C'), or 'T' if there is a tie, into the variable
     * answer.
     */
	int count[3] = {0, 0, 0};
	int max = 0, winner = -1;
	bool tie = false;
	
	for (int i = 0 ; i < N ; i++) {
		if (votes[i] == 'A')
			count[0]++;
		else if (votes[i] == 'B')
			count[1]++;
		else if (votes[i] == 'C')
			count[2]++;
	}

	for (int i = 0 ; i < 3 ; i++) {
		if (count[i] > max) {
			max = count[i];
			winner = i;
			tie = false;
		}
		else if (count[i] == max)
			tie = true;
	}

	if (tie)
		answer = 'T';
	else if (winner == 0)
		answer = 'A';
	else if (winner == 1)
		answer = 'B';
	else if (winner == 2)
		answer = 'C';

    /* Write the answer to the output file. */
    fprintf(output_file, "%c\n", answer);

    /* Finally, close the input/output files. */
    fclose(input_file);
    fclose(output_file);

    return 0;
}
