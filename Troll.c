#include<life.h>
#include<love.h>
#include<work.h>

#define INF girl_in_the_world
#define all_girl INF

void find_money() {
  work();
}

int find_girlfriend() {
  for(int i = 1; i <= all_girl; i++)
    if(match[you][girl[i]]){
      girlfriend = i;
      break;
    }
}

void get_married() {
  find_money();
  girlfriend = find_girlfriend();
  marry(i);
}

own life(){
  get_married();
}
