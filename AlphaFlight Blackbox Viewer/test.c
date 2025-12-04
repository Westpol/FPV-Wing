#include <stdio.h>

int main(){
    int i = 1000000000;
    void* pointer = &i;
    float* floatPointer = pointer;
    printf("%d, %f\n\n", i, *floatPointer);
}