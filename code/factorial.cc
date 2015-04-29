#include "mex.h"
#include <stdint.h>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

int dims[2];
dims[0] = mxGetM(prhs[0]);
dims[1] = mxGetN(prhs[0]);
plhs[0] = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL);

double* in  = (double*) mxGetData(prhs[0]);
double* out = (double*) mxGetData(plhs[0]);

for (int i = 0; i<dims[0]*dims[1]; i++) {
    out[i] = 1;
    for (int j = 2; j<=in[i]; j++) {
        out[i] = out[i]*j;
    }
}

}
