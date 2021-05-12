%%file bmpdata_to_csv.c 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
int main(void)
{
  FILE *inp,*out; //出力ファイルはAltitude,Pressure at sealevel (calculated) 
  inp = fopen("test2.txt","r");
  out = fopen("test2(to_excel).txt","w");
  if(inp==NULL)
  {
    printf("Could not open properly");
    exit(EXIT_FAILURE);//異常終了
  }
  int i;
  char line1[1024];
  char line2[1024];
  char line3[1024];
  char c =',';
  char kensaku1[] = "Altitude";
  char kensaku2[] = "Pressure at sealevel (calculated)";
  while(fgets(line1,1024,inp) != NULL)
  {
    if(strstr(line1,kensaku1) != NULL)
    {
      for(i=11;i<=16;i++)
      {
        line2[i-11] = line1[i];
        line2[5] = c;
      }
      fputs(line2,out);
    }
    else if(strstr(line1,kensaku2) != NULL)
    {
      for(i=36;i<=42;i++)
      {
        line3[i-36] = line1[i];
      } 
      fputs(line3,out);//配列が3つ以上あると、出力がうまくいかないのかも
      fputs("\n",out);
    }
  }
  fclose(inp);
  return 0;
}
