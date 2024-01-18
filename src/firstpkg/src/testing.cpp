#include <cstdio>
#include <cstdlib>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  if (argc == 3){
    printf("teste\n");
    printf("%d\n", atoi(argv[1]) * atoi(argv[2]));
  }
  else
  printf("hello world firstpkg package\n");
  
  return 0;
}
