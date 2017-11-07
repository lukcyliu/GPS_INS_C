#include "matrix_lc.h"
#include "SVD.h"
// (det用)功能：求逆序对的个数
int inver_order(int list[], int n) {
    int ret = 0;
    for (int i = 1; i < n; i++)
        for (int j = 0; j < i; j++)
            if (list[j] > list[i])
                ret++;
    return ret;
}

// (det用)功能：符号函数，正数返回1，负数返回-1
int sgn(int order) {
    return order % 2 ? -1 : 1;
}

// (det用)功能：交换两整数a、b的值
void swap(int *a, int *b) {
    int m;
    m = *a;
    *a = *b;
    *b = m;
}

// 功能：求矩阵行列式的核心函数
double det(double *p, int n, int k, int list[], double sum) {
    if (k >= n) {
        int order = inver_order(list, n);
        double item = (double) sgn(order);
        for (int i = 0; i < n; i++) {
            //item *= p[i][list[i]];
            item *= *(p + i * n + list[i]);
        }
        return sum + item;
    } else {
        for (int i = k; i < n; i++) {
            swap(&list[k], &list[i]);
            sum = det(p, n, k + 1, list, sum);
            swap(&list[k], &list[i]);
        }
    }
    return sum;
}

// 功能：矩阵显示
// 形参：(输入)矩阵首地址指针Mat，矩阵行数row和列数col。
// 返回：无
void MatShow(double *Mat, int row, int col) {
    for (int i = 0; i < row * col; i++) {
        printf("%16lf ", Mat[i]);
        if (0 == (i + 1) % col) printf("\n");
    }
}

// 功能：矩阵相加
// 形参：(输入)矩阵A首地址指针A，矩阵B首地址指针B，矩阵A(也是矩阵B)行数row和列数col
// 返回：A+B
double *MatAdd(double *A, double *B, int row, int col) {
    double *Out = (double *) malloc(sizeof(double) * row * col);
    for (int i = 0; i < row; i++)
        for (int j = 0; j < col; j++)
            Out[col * i + j] = A[col * i + j] + B[col * i + j];
    return Out;
}

// 功能：矩阵相减
// 形参：(输入)矩阵A，矩阵B，矩阵A(也是矩阵B)行数row和列数col
// 返回：A-B
double *MatSub(double *A, double *B, int row, int col) {
    double *Out = (double *) malloc(sizeof(double) * row * col);
    for (int i = 0; i < row; i++)
        for (int j = 0; j < col; j++)
            Out[col * i + j] = A[col * i + j] - B[col * i + j];
    return Out;
}

// 功能：矩阵相乘
// 形参：(输入)矩阵A，矩阵A行数row和列数col，矩阵B，矩阵B行数row和列数col
// 返回：A*B
double *MatMul(double *A, int Arow, int Acol, double *B, int Brow, int Bcol) {
    double *Out = (double *) malloc(sizeof(double) * Arow * Bcol);
    if (Acol != Brow) {
        printf("        Shit!矩阵不可乘!\n");
        return NULL;
    }
    if (Acol == Brow) {
        int i, j, k;
        for (i = 0; i < Arow; i++)
            for (j = 0; j < Bcol; j++) {
                Out[Bcol * i + j] = 0;
                for (k = 0; k < Acol; k++)
                    Out[Bcol * i + j] = Out[Bcol * i + j] + A[Acol * i + k] * B[Bcol * k + j];
            }
        return Out;
    }
}

//功能：三矩阵相乘
//形参：矩阵ABC首地址指针，A行数Arow，B行列数Brow和Bcol，C列数Ccol
//返回：ABC，
double *MatMulThree(double *A, double *B, double *C, int Arow, int Brow, int Bcol, int Ccol) {
    double *out = (double *) malloc(sizeof(double) * Arow * Ccol);
    double *AB = (double *) malloc(sizeof(double) * Arow * Bcol);
    AB = MatMul(A, Arow, Brow, B, Brow, Bcol);
    out = MatMul(AB, Arow, Bcol, C, Bcol, Ccol);
    return out;
}

// 功能：矩阵数乘(实数k乘以矩阵A)
// 形参：(输入)矩阵A首地址指针，矩阵行数row和列数col，实数k
// 返回：kA
double *MatMulk(double *A, int row, int col, double k) {
    double *Out = (double *) malloc(sizeof(double) * row * col);
    for (int i = 0; i < row * col; i++) {
        *Out = *A * k;
        Out++;
        A++;
    }
    Out = Out - row * col;
    return Out;
}

// 功能：矩阵转置
// 形参：(输入)矩阵A首地址指针A，行数row和列数col
// 返回：A的转置
double *MatT(double *A, int row, int col) {
    double *Out = (double *) malloc(sizeof(double) * row * col);
    for (int i = 0; i < row; i++)
        for (int j = 0; j < col; j++)
            Out[row * j + i] = A[col * i + j];
    return Out;
}

// 功能：求行列式值
// 形参：(输入)矩阵A首地址指针A，行数row
// 返回：A的行列式值
double MatDet(double *A, int row) {
    int *list = (int *) malloc(sizeof(int) * row);
    for (int i = 0; i < row; i++)
        list[i] = i;
    double Out = det(A, row, 0, list, 0.0);
    free(list);
    return Out;
}

// 功能：矩阵的逆
// 形参：(输入)矩阵A首地址指针A，行数row和列数col
// 返回：A的逆
double *MatInv(double *A, int row, int col) {
    double *Out = (double *) malloc(sizeof(double) * row * col);
    double det = MatDet(A, row); //求行列式
    if (det == 0) {
        return MatPInv(A,row,col);
    }
    if (det != 0) {
        Out = MatAdj(A, row, col); //求伴随矩阵
        int len = row * row;
        for (int i = 0; i < len; i++)
            *(Out + i) /= det;
        return Out;
    }
}
double* MatPInv(double*A, int row, int col){
    int MN = MAX(row,col);
    double** input = one_two(A,MN,MN);

    //奇异值分解要求的矩阵
    double** U = Make2DArray(MN,MN);
    double** V = Make2DArray(MN,MN);
    double* S = malloc(sizeof(double) * MN);
    SetZero(U,MN);
    for(int i = 0;i < MN;i++)
        for (int j = 0;j < MN; j++)
            U[i][j] = input[i][j];
    dsvd(U,MN,MN,S,V);
    //U*S*V' = A
    //现在求A的伪逆：V*S'*U'
    //V和U是正交阵，逆为自己的转置，S是对角阵，转置为非0取倒
    double* U_1 = two_one(U,MN,MN);
    double* V_1 = two_one(V,MN,MN);
    for(int i = 0;i < MN;i++)
        if(S[i] != 0)
            S[i] = 1 / S[i];
    U_1 = MatT(U_1,MN,MN);
    S = MatDiag(S,MN);
    double** Out = MatMul(MatMul(V_1,MN,MN,S,MN,MN),MN,MN,U_1,MN,MN);
    return Out;
}

// 功能：求代数余子式
// 形参：(输入)矩阵A首地址指针A，矩阵行数row, 元素a的下标m，n(从0开始)，
// 返回：NxN 矩阵中元素A(mn)的代数余子式
double MatACof(double *A, int row, int m, int n) {
    int len = (row - 1) * (row - 1);
    double *cofactor = (double *) malloc(sizeof(double) * len);

    int count = 0;
    int raw_len = row * row;
    for (int i = 0; i < raw_len; i++)
        if (i / row != m && i % row != n)
            *(cofactor + count++) = *(A + i);
    double ret = MatDet(cofactor, row - 1);
    if ((m + n) % 2)
        ret = -ret;
    free(cofactor);
    return ret;
}

// 功能：求伴随矩阵
// 形参：(输入)矩阵A首地址指针A，行数row和列数col
// 返回：A的伴随矩阵
double *MatAdj(double *A, int row, int col) {
    double *Out = (double *) malloc(sizeof(double) * row * col);
    int len = row * row;
    int count = 0;
    for (int i = 0; i < len; i++) {
        *(Out + count++) = MatACof(A, row, i % row, i / row);
    }
    return Out;
}

//功能：创建对角矩阵
//形参：（输入）对角数组A首地址指针A，以及对角元素个数n
//返回：n×n的对角矩阵,对角元素为数组A的值
double *MatDiag(double *A, int n) {
    int row = n;
    int col = n;
    double *out = (double *) malloc(sizeof(double) * row * col);
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            if (i == j) {
                out[col * i + j] = A[i];
            } else {
                out[col * i + j] = 0;
            }
        }
    }
    return out;
}

//功能：创建单位矩阵
//形参：对角元素个数n
//返回：n×n的单位矩阵
double *MatEye(int n) {
    int row = n;
    int col = n;
    double *out = (double *) malloc(sizeof(double) * row * col);
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            if (i == j) {
                out[col * i + j] = 1;
            } else {
                out[col * i + j] = 0;
            }
        }
    }
    return out;
}

// 读取文件行数
int FileReadRow(const char *filename) {
    FILE *f = fopen(filename, "r");
    int i = 0;
    char str[4096];
    while (NULL != fgets(str, 4096, f))
        ++i;
    printf("数组行数：%d\n", i);
    return i;
}

// 读取文件每行数据数(逗号数+1)
int FileReadCol(const char *filename) {
    FILE *f = fopen(filename, "r");
    int i = 0;
    char str[4096];
    fgets(str, 4096, f);
    for (int j = 0; j < strlen(str); j++) {
        if (',' == str[j]) i++;
    }
    i++;// 数据数=逗号数+1
    printf("数组列数：%d\n", i);
    return i;
}

// 逗号间隔数据提取
void GetCommaData(char str_In[4096], double double_Out[1024]) {
    int str_In_len = strlen(str_In);
    //printf("str_In_len:%d\n", str_In_len);
    char str_Data_temp[128];
    int j = 0;
    int double_Out_num = 0;
    for (int i = 0; i < str_In_len; i++) {
        //不是逗号，则是数据，存入临时数组中
        if (',' != str_In[i]) str_Data_temp[j++] = str_In[i];
        //是逗号或\n(最后一个数据)，则数据转换为double，保存到输出数组
        if (',' == str_In[i] || '\n' == str_In[i]) {
            str_Data_temp[j] = '\0';
            j = 0; /*printf("str_Data_temp:%s\n", str_Data_temp); */double_Out[double_Out_num++] = atof(str_Data_temp);
            memset(str_Data_temp, 0, sizeof(str_Data_temp));
        }
    }
}

// 功能：从csv文件读矩阵，保存到指针中
// 形参：(输入)csv文件名，预计行数row和列数col
// 返回：矩阵指针A
double *MatRead(char *csvFileName) {
    int row = FileReadRow(csvFileName);
    int col = FileReadCol(csvFileName);
    double *Out = (double *) malloc(sizeof(double) * row * col);
    FILE *f = fopen(csvFileName, "r");
    char buffer[4096];
    while (fgets(buffer, sizeof(buffer), f)) {
        //printf("buffer[%s]\n",buffer);
        double double_Out[128] = {0};
        GetCommaData(buffer, double_Out);
        for (int i = 0; i < col; i++) {
            //printf("double_Out:%lf\n", double_Out[i]);
            *Out = double_Out[i];
            Out++;
        }

    }
    Out = Out - row * col;//指针移回数据开头
    fclose(f);
    return Out;
}

// 功能：将矩阵A存入csv文件中
// 形参：(输入)保存的csv文件名，矩阵A首地址指针A，行数row和列数col
// 返回：无
void MatWrite(const char *csvFileName, double *A, int row, int col) {
    FILE *DateFile;
    double *Ap = A;
    DateFile = fopen(csvFileName, "w");//追加的方式保存生成的时间戳
    for (int i = 0; i < row * col; i++) {
        if ((i + 1) % col == 0) fprintf(DateFile, "%lf\n", *Ap);//保存到文件，到列数换行
        else fprintf(DateFile, "%lf,", *Ap);//加逗号
        Ap++;
    }
    fclose(DateFile);
}

double* two_one(double** input,int row,int col){
    double* output = (double*)malloc(sizeof(double) * row * col);
    for(int i = 0;i < row;i++)
        for (int j = 0; j < col; ++j)
            output[i * row + j] = input[i][j];
    return output;
}
double** one_two(double* input,int row,int col){
    double** output = Make2DArray(row,col);
    for(int i = 0;i < row;i++)
        for(int j = 0;j < col;j++)
            output[i][j] = input[i * row + j];
    return output;
}
