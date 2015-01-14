#include "_inputcheck.h"
/*++++++++++++++++++++++++++++++++++++++++++++++*/
/*    getnschar...get next non-space character  */
/*    非空白類文字を1文字読み込む               */
/*    引数 : なし                               */
/*    返り値 : 得られた非空白類文字をint型で    */
/*++++++++++++++++++++++++++++++++++++++++++++++*/
int getnschar(void)
{
  int ch;
  while(isspace(ch = getchar()) && ch != EOF)
    ;
  return ch;
}
/*++++++++++++++++++++++++++++++++++++++++++++++*/
/*    kakunin...確認                            */
/*    引数 : なし                               */
/*    返り値 : 1...1                            */
/*             2...2                            */
/*             others...0                      */
/*++++++++++++++++++++++++++++++++++++++++++++++*/
int kakunin(void)
{
  int ch;
  do{
    ch = getnschar();
    if(ch == '1')
      return 1;
    if(ch == '2')
      return 2;
  }while(ch != EOF);
  return 0;
}

int flag_check(bool flag)
{
  if(flag == true){
    return 1;  
  }else{
    return -1;
  }
}
