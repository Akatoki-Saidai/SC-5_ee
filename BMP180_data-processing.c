%%file test3.c
#include<stdio.h>
#include<stdlib.h>
#include<string.h>

int main(void)
{
  int i;
  int m;
  FILE *fp1,*fp2;
  char data[] = "test2.txt";
  fp1=fopen("test2.txt","r");
  fp2=fopen("test2(to_excel).txt","w");
  
  //ファイルが開けているかどうか
  if(fp1 == NULL){   
      printf("%s file is not open",data);
  }else{
      printf("%s file is open",data);
  }
  
  char line1[1024];
  char line2[1024];
  char c = ',';
  
  //判定したいのは圧力と高度で３つ
  char kensaku1[]="Pressure";
  char kensaku2[]="Altitude"; 
  char kensaku3[]="Pressure at sealevel (calculated)"; 
  

  while(fgets(line1,1024,fp1)){
  
      //Pressureの文字があったら、11文字目から17文字目までを抜き出して使う
      if(strstr(line1,kensaku1) != NULL){
          for(i>11;i<=17;i++){
          line2[i-11] = line1[i];
          line2[6] = c;
          }
      }
      //Altitudeの文字があったら、11文字目から16文字目までを抜き出して使う
      else if(strstr(line1,kensaku2) != NULL){
          for(i>11;i<=16;i++){
          line2[i-11] = line1[i];
          line2[5] = c;
          }
      }
      //Pressure at...の文字があったら、36文字目から42文字目までを抜き出して使う
      else if(strstr(line1,kensaku3) != NULL){
          for(i>36;i<=42;i++){
          line2[i-36] = line1[i];
          line2[6] = c;
          }
      }
      fputs(line2,fp2);
      }
  fclose(fp1);
  return 0;
}
