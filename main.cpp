#include "global_router.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int parseFlag(const char *arg, const char *prefix, int *value) {
  if (!arg || !prefix || !value) return 0;

  size_t len = strlen(prefix);
  if (strncmp(arg, prefix, len) != 0) return 0;

  const char *numStr = arg + len;
  if (*numStr != '0' && *numStr != '1') return 0;
  if (*(numStr + 1) != '\0') return 0;

  *value = (*numStr == '1') ? 1 : 0;
  return 1;
}

int main(int argc, char **argv)
{
  if (argc != 5) {
    printf("Usage : ./ROUTE.exe -d=0|1 -n=0|1 <input_benchmark_name> <output_file_name>\n");
    return 1;
  }

  int dFlag = 0;
  int nFlag = 0;

  if (!parseFlag(argv[1], "-d=", &dFlag)) {
    printf("ERROR: invalid d flag. Expected -d=0 or -d=1\n");
    return 1;
  }

  if (!parseFlag(argv[2], "-n=", &nFlag)) {
    printf("ERROR: invalid n flag. Expected -n=0 or -n=1\n");
    return 1;
  }

  char *inputFileName = argv[3];
  char *outputFileName = argv[4];

  setRoutingMode(dFlag, nFlag);

  routingInst *rst = new routingInst;

  int status = readBenchmark(inputFileName, rst);
  if (status == 0) {
    printf("ERROR: reading input file\n");
    delete rst;
    return 1;
  }

  status = solveRouting(rst);
  if (status == 0) {
    printf("ERROR: running routing\n");
    release(rst);
    delete rst;
    return 1;
  }

  status = writeOutput(outputFileName, rst);
  if (status == 0) {
    printf("ERROR: writing the result\n");
    release(rst);
    delete rst;
    return 1;
  }

  release(rst);
  delete rst;

  printf("\nDONE!\n");
  return 0;
}