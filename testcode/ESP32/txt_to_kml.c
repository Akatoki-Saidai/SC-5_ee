%%file txt_to_kml_replacing_as_txt.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
int main(void)
{
  FILE *inp,*out;
  inp = fopen("gps(rainavrage).txt","r");
  out = fopen("gps(rainavrage)(to_kml).txt","w");
  if(inp==NULL)
  {
    printf("Could not open properly");
    exit(EXIT_FAILURE);//異常終了
  }
  int i;
  char line1[1024];
  char line2[1024];
  char line3[1024];
  char line4[1024];
  char c =',';
  char kensaku1[] = "LAT";
  char kensaku2[] = "LONG";
  while(fgets(line1,1024,inp) != NULL)//次の行を読み込むときに自動改行してくれてるのかも
  {
    if(strstr(line1,kensaku1) != NULL)//line1にkensaku1の中身が含まれれば
    {
     strcpy(line2,line1);
    }
    else if(strstr(line1,kensaku2) != NULL)
    {
      for(i=21;i<=34;i++)
      {
        line3[i-21] = line1[i];
        line3[13] = c;
      }
      for(i=21;i<=34;i++)
      {
        line4[i-21] = line2[i];
      }
     fputs(line3,out);
     fputs(line4,out);//書き込み
    }
  }
  fclose(inp);
  return 0;
}
