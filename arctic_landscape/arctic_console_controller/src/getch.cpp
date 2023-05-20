#include<unistd.h>
#include<termios.h>

#include"getch.h"

int getch(char &result){
  char buf = 0;
  struct termios old = {0};
  if (tcgetattr(0, &old) < 0)
    return -1;
  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;
  if (tcsetattr(0, TCSANOW, &old) < 0)
    return -2;
  if (read(0, &buf, 1) < 0)
    return -3;
  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;
  if (tcsetattr(0, TCSADRAIN, &old) < 0)
    return -4;
  result = buf;
  return 0;
}

