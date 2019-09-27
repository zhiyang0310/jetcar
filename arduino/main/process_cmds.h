#ifndef RECV_COMMANDS
#define RECV_COMMANDS

//define self-driving mode
bool self_driving = false;
////////Variables to receive commands///////////
//Variable to hold an input character
char chr;
//A pair of varibles to parse serial commands
int arg = 0;
int index = 0;
// Variable to hold the current single-character command
char cmd;
// Character arrays to hold the first and second arguments
char argv1[20];
char argv2[20];
// The arguments converted to integers
long arg1;
long arg2;
/////////////////////////////////////////////////

void reset_cmd();
void run_cmd();

#endif
