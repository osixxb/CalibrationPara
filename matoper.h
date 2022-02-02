
#include "common.h"

void matadd(FLOAT64_T c[], const FLOAT64_T a[], const FLOAT64_T b[],	const INT32_T row_c, const INT32_T col_c);
void matsub(FLOAT64_T c[], const FLOAT64_T a[], const FLOAT64_T b[],	const INT32_T row_c, const INT32_T col_c);
void matmult(FLOAT64_T c[], const FLOAT64_T a[], const FLOAT64_T b[], const INT32_T row_a, const INT32_T col_a, const INT32_T col_b);
void mattran(FLOAT64_T b[], const FLOAT64_T a[], const INT32_T row_a, const INT32_T col_a);

/* matrix operation */
void matadd(FLOAT64_T c[], const FLOAT64_T a[], const FLOAT64_T b[], const INT32_T row_c, const INT32_T col_c)
{
    int i,j;

    for (i=0; i<row_c; i++)
        for (j=0; j<col_c; j++)
            c[i*col_c+j] = a[i*col_c+j]+b[i*col_c+j];
}

void matsub(FLOAT64_T c[], const FLOAT64_T a[], const FLOAT64_T b[], const INT32_T row_c, const INT32_T col_c)
{
    int i,j;

    for (i=0; i<row_c; i++)
        for (j=0; j<col_c; j++)
            c[i*col_c+j] = a[i*col_c+j]-b[i*col_c+j];
}

/************************************************************************\
* matrix product
* c = a*b,
* row_a - the row of matrix forward,
* col_a - the column of matrix forward and the row of matrix backward,
* col_b - the column of matrix backward
2009.7.5 AC LuQuancong
\************************************************************************/
void matmult(FLOAT64_T c[], const FLOAT64_T a[], const FLOAT64_T b[], const INT32_T row_a, const INT32_T col_a, const INT32_T col_b)
{
    int i,j,k;

    for (i=0; i<row_a; i++)
        for (j=0; j<col_b; j++)
            for (c[i*col_b+j]=0.0,k=0; k<col_a; k++)
                c[i*col_b+j] += a[i*col_a+k]*b[k*col_b+j];
}

/************************************************************************\
 * matrix transpose
 * b = a'
 * row_a - the row of a
 * col_a - the column of b
2009.7.6 LuQuancong
\************************************************************************/
void mattran(FLOAT64_T b[], const FLOAT64_T a[], const INT32_T row_a, const INT32_T col_a)
{
    int i,j;

    for (i=0; i<row_a; i++)
        for (j=0; j<col_a; j++)
            b[j*row_a+i] = a[i*col_a+j];
}
