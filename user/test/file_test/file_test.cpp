/*
 * @Author: your name
 * @Date: 2021-10-01 05:34:08
 * @LastEditTime: 2021-10-01 05:47:21
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /robot_software/user/test/file_test/file_test.cpp
 */

#include <stdio.h>
#include <unistd.h>//"usleep" function

int main()
{
    FILE* fp = fopen("file_test.txt", "w");
    
    int a[6] = {0,1,2,3,4,5};
    float b[6] = {0.1, 1.1, 2.2, 3.3, 4.4, 5.5};
    char c[6] = {'a', 'b', 'c', 'd', 'e', 'f'};

    for(int i = 0; i < 6; i++)
    fprintf(fp, "%d  %.3f  %c\n",a[i],b[i],c[i]);
    fclose(fp);

    return 1;
}
