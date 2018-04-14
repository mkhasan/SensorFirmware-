#ifndef COMMON_H_
#define COMMON_H_

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

#endif