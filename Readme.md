### Introduction

We have already packaged the CBC solver into the third_party directory, so you don't need to install the CBC solver yourself. The code currently only supports Linux operating system and requires Cmake and GCC. It is not compatible with other operating systems at the moment.

### Complie

```
mkdir build
cd build
cmake ..
make 
```

### Run

```
./cbc_demo
```

### 0 CBC 求解器简介

CBC Solver（Coin-or Branch and Cut Solver）是一个开源的线性和混合整数规划求解器。它基于Coin-OR计划，是一个强大的数学优化工具。CBC Solver 提供了一系列高效的算法和技术，用于处理线性规划（LP）和混合整数规划（MIP）问题。它具有广泛的应用领域，包括生产调度、供应链优化、资源分配等。

考虑使用 CBC Solver 而不使用 Gurobi 或者 CPLEX 求解器的主要原因在于商用场景之下 CBC Solver 是完全免费完全开源的，同时从安全性角度来讲 CBC Solver 也是相对来说最安全的 （被断供被封杀的可能性最小），这也是某大厂经常使用 CBC Solver 的原因。当然 CBC Solver 的缺点是显而易见的：

1. 求解速度很慢，只能作为一个 weak solver 来用，解个线性规划解个几百个变量的整数规划还可以，问题规模稍大一点就不行了。非商业目的只是学术科研的话就不需要考虑用 CBC Solver
2. CBC的文档写得稀烂，当然对于开源项目来说我们不能要求太多，人家都把源码给你白嫖了，你再要求有好的文档是有点过分了，所以很多时候用户只能去读源代码来做自己的开发。这就造成开发效率比较低下，上手的难度大大增加。
3. 可参考教程少，网上关于 Gurobi 和 CPLEX 等主流求解器的资料很多，实际应用案例也非常多，但是关于 CBC Solver 的几乎没有几个，所以遇到问题也无从查起。

鉴于此也是撰写本文的原因，把常用的 CBC Solver 的API做一个 demo，形成一个初步的开发文档，方便大家使用。

### 1 添加决策变量

#### 1.1 添加与删除决策变量

```C++
// Add variables
  int numVar = 5;
  CoinPackedVector v0;
  for (int i = 0; i < numVar; i++)
      solver1.addCol(v0, 0, 1, 0.0);
```

CoinPackerVector 是 CBC Solver 自己定义的一种数据类型，主要是为了存储向量而设计的。不同于常见的 vector，CoinPackerVector 会存储向量的索引和元素值。例如一个数组 {2,0,5,6,0,7}，在CoinPackerVector 会这样表示 {(0,2), (2,5), (3,6), (5,7)}.

solver1.addCol(v0, 0, 1, 0.0); 中 v0 表示该变量在约束中的向量，0和1分别代表变量的上下界，0.0代表变量在目标函数的系数。这里我们先不指定变量在哪些约束中出现所有 v0 只需要放一个空的 vector即可，因为我们会在后边添加约束。

#### 1.2 修改决策变量上下界，变量名和变量类型

```C++
  // Set var upper bound and lower bound
  for (int i = 0; i < numVar; i++){
     solver1.setColUpper(i, 1.0);
     solver1.setColLower(i, 0.0);
     solver1.setInteger(i); // set variable to be integer
     // solver1.setContinuous(i); // set variable to be ocntinuous
     solver1.setColName(i, "x" + std::to_string(i)); // set variable name
  }
```

设置决策变量上下界可以通过在 solver1.addCol(v0, 0, 1, 0.0) 中就设定好，也可以通过以上代码进行设置。

#### 1.3 删除决策变量

```C++
   std::vector<int> colIdx = {0, 1};
   solver1.deleteCols(colIdx.size(), colIdx.data());
```

删除掉x0和x1两个决策变量。

### 2 添加与删除约束

#### 2.1 添加小于等于约束

```C++
// Add constraint: 11*x0 + 20*x1 + 3*x2 <= 12
  CoinPackedVector v1;
  v1.insert(0, 11.0); // add x0 coefficient 11.0
  v1.insert(1, 20.0); // add x1 coefficient 20.0
  v1.insert(2, 3.0);  // add x2 coefficient 3.0
  char rowsen = 'L'; //  constrants type is equal or less than
  double rowrhs = 12.0; // right hand side of the constraint
  double rowrng = 0.0; // A relaxation degree of 0 indicates that the constraint is strict.
  solver1.addRow(v1, rowsen, rowrhs, rowrng);
```

约束类型如下所示：'L': <= constraint，'E': =  constraint，'G': >= constraint，'R': ranged constraint
'N': free constraint

#### 2.2 添加 range 约束

```C++
// Add constraint: 22 <= 0.5*x1 + x2 + 21.8*x3 <= DBL_MAX
  CoinPackedVector v2;
  double rowLowerBound = 22;  
  double rowUpperBound = DBL_MAX;
  v2.insert(1, 0.5);
  v2.insert(2, 1.0);
  v2.insert(3, 21.8);
  solver1.addRow(v2, rowLowerBound, rowUpperBound);
```

和上面添加约束的方式不同在于这种方式可以添加形如这样的约束：
22 <= 0.5*x1 + x2 + 21.8*x3 <= DBL_MAX

#### 2.3 删除约束

```C++
  std::vector<int> rowIdx = {0, 1};
  solver1.deleteRows(rowIdx.size(), rowIdx.data());
```

删除掉第一和第二条约束。

### 3 设置目标函数

```C++
// Set objective function: x0+x1+x2+x3+x4
  for (int i = 0; i < numVar; i++)
      solver1.setObjCoeff(i, 1.0); // set objective function coefficient
```

```C++
 // Set objective function sense (1 for min (default), -1 for max,)
  solver1.setObjSense(1);
```

至此建模部分已经完成，接下来讨论求解和获取数据相关的内容。

### 4 设置求解相关参数

```C++
  CbcModel model(solver1);
// Set the number of threads to use in parallel
  model.setNumberThreads(16);
  model.setLogLevel(1); // log level range from 0-3
  model.setMaximumSeconds(60.0); // set time limit
```

我们首先需要将 solver1 装载到 model 中，然后分别设置求解线程数，log水平和最大求解时间。log 水平从0-3，0表示不打印任何log，1表示打印常规的log，2和3打印最多的log，用户可以根据情况自行选择 log 水平。

```C++
 // set initial solution
  std::vector<std::pair< std::string, double > > initSol;
  for (int i = 0; i < numVar; i++)
      initSol.push_back(std::make_pair(data.colName[i], 0.0));
  model.setMIPStart(initSol);
```

如果是求解混合整数线性规划问题，我们还可以给求解器输入一个初始解来加速求解器的求解。model.setMIPStart(initSol) 就是将初始解导入到求解器的API函数。initSol 表示初始解，它是一个字典，key是变量名，value是变量的初始值。如何过去变量名(data.colName)我们会在后边讲解。

```C++
 // start branch and bound tree search
  model.branchAndBound();
  int status = model.status(); 
  /** Final status of problem
        -1 before branchAndBound
        0 finished - check isProvenOptimal or isProvenInfeasible to see if solution found
        (or check value of best solution)
        1 stopped - on maxnodes, maxsols, maxtime
        2 difficulties so run was abandoned
        (5 event user programmed event occurred)
    */
```

开启 branch and bound 搜索，求解结束后即可获取求解的状态 status，我们在注释中也详细标明了status的含义。

### 5 获取模型相关信息

#### 5.1 获取最优解

```C++
if (model.isProvenOptimal())
  {
      const double *solution = model.bestSolution();
      for (int i = 0; i < numVar; i++) {
          double value = solution[i];
          std::cout << data.colName[i] << ":" << value << ", ";
      }
      std::cout << std::endl;
      std::cout << "Obj value: " << model.getObjValue() << std::endl; // get objecitve value
  }
```

完成求解之后最常用的操作就是获取最优解，那么首先通过 model.isProvenOptimal() 判断是否求到了最优解，如果已经求到了最优解那么直接用 model.bestSolution() 就可以获得最优解了

#### 5.2 获取模型大小信息

```C++
    data.numRows = model.solver()->getNumRows(); // get the number of constraints
    data.numCols = model.solver()->getNumCols(); // get the number of vars
    data.numNonZeros = model.solver()->getNumElements(); // get the number of nonzeros elements
    data.objSense = model.getObjSense(); // get the objective sense (1 for min (default), -1 for max,)
```

获取决策变量维数，约束数量，非零元素个数和目标函数极大化极小化的信息。

#### 5.3 获取变量信息

```C++
/* col types:
       - 0 - continuous
       - 1 - binary
       - 2 - general integer
       - 3 - if supported - semi-continuous
       - 4 - if supported - semi-continuous integer
 */
data.varTypes = std::vector<char>(data.numCols);
for (int i = 0; i < data.numCols; i++)
    {
      if (model.solver()->isContinuous(i)) {
            data.varTypes[i] = 'C';
        } else if (model.solver()->isInteger(i)) {
            data.varTypes[i] = 'I';
        } else {
            std::cout << "Var " << i << " is neither continuous nor integer." << std::endl;
        }
        // model.solver()->isBinary(i)
        // model.solver()->isIntegerNonBinary(i)
    }

    const double *lb = model.solver()->getColLower();
    const double *ub = model.solver()->getColUpper();
    data.lb = std::vector<double>(data.numCols);
    data.ub = std::vector<double>(data.numCols);
    for(int i = 0; i < data.numCols; i++) {
        data.lb[i] = lb[i];
        data.ub[i] = ub[i];
    }
```

获取变量类型和获取变量上下界，变量类型一般有两种整数和连续的（其实可以再细分，我们这里就不做进一步细分了，只是笼统的分一下，详细的变量类型已经标注在注释中，需要的可以根据情况自行采用）。

#### 5.4 获取约束信息

```C++
const char* rowtypes = model.solver()->getRowSense();
data.rowtypes = std::vector<char>(data.numRows);
for(int i = 0; i < data.numRows; i++) data.rowtypes[i] = rowtypes[i];

const double* rhs = model.solver()->getRightHandSide();
data.rhs = std::vector<double>(data.numRows);
for(int i = 0; i < data.numRows; i++) data.rhs[i] = rhs[i];
```

分别获取约束类型和约束右端常数项。约束类型如下所示：'L': <= constraint，'E': =  constraint，'G': >= constraint，'R': ranged constraint
'N': free constraint

```C++
// get CSR matrix
    const CoinPackedMatrix * matrixByRow = model.solver()->getMatrixByRow();
    const double *colCoeffs = matrixByRow->getElements();
    const int *colIdxs = matrixByRow->getIndices();
    const int *rowStart = matrixByRow->getVectorStarts();
    // data.colCoeffs[data.rowStart[i]:data.rowStart[i+1]] means the nonzero elements of row i
    // data.colIdxs[data.rowStart[i]:data.rowStart[i+1]] means the column index of the nonzero elements of row i
    data.colCoeffs = std::vector<double>(data.numNonZeros); // coefficient of each nonzero element
    data.rowStart = std::vector<int>(data.numRows + 1); // start index of each row
    data.colIdxs = std::vector<int>(data.numNonZeros); // column index of each nonzero element

    for (int i = 0; i < data.numNonZeros; i++) {
        data.colCoeffs[i] = colCoeffs[i];
        data.colIdxs[i] = colIdxs[i];
    }

    for (int i = 0; i < data.numNonZeros + 1; i++) 
        data.rowStart[i] = rowStart[i];
```

这一段代码稍微复杂一点，主要是为了获取约束矩阵，由于约束矩阵的大小为 $m \times n$ ，m为约束数量，n为决策变量维数，其规模非常庞大。如果直接按照常规的矩阵存储方式会占用非常多内存，导致 memory error 的问题。我们观察可以发现约束矩阵通常来说是非常稀疏，大量的0元素占据了庞大的内存空间，所以在求解器中我们一般是采用压缩矩阵的方式来存储约束矩阵。

那么其中一种常用的压缩矩阵方式就是 Compressed Sparse Row （CSR）也叫做行压缩矩阵。行压缩矩阵用三个 vector 来存储矩阵：

1. colCoeffs，用来存储矩阵中的非零元素的值；
2. colIdxs，第i个元素记录了V[i]元素的列数；
3. rowStart, 第i个元素记录了前i-1行包含的非零元素的数量。

如果我们想要表示这样一个约束矩阵：

$$
\left( \begin{matrix}
	10&		3&		0&		0&		0&		0\\
	0&		30&		0&		40&		0&		0\\
	0&		0&		50&		60&		5&		0\\
	0&		0&		0&		0&		0&		12\\
\end{matrix} \right)
$$

colCoeffs: $\left[ 10,3,30,40,50,60,5,12 \right]$

colIdxs: $\left[ 0,1,1,3,2,3,4,5 \right]$

rowStart: $\left[ 0,2,4,7,8 \right]$

如何访问某一行的元素，我们在代码注释里边添加了，这里就不再赘述了。更多关于行压缩矩阵的内容可以参考：

https://zhuanlan.zhihu.com/p/623366713

#### 5.5 获取变量名和约束名

```C++
for (int i = 0; i < data.numCols; i++)
       data.colName.push_back(model.solver()->getColName(i));
for (int i = 0; i < data.numRows; i++)
       data.rowName.push_back(model.solver()->getRowName(i));
```

#### 5.6 获取目标函数系数

```C++
 const double* objCoeffs = model.solver()->getObjCoefficients();
 data.objCoeffs = std::vector<double>(data.numCols);
 for(int i = 0; i < data.numCols; i++) 
     data.objCoeffs[i] = objCoeffs[i];
```

### 7 输入输出MPS文件

```C++
// Read the mps file
int numMpsReadErrors = solver1.readMps(argv[1], "");
assert(numMpsReadErrors == 0);
// Write MPS file
solver1.writeMps("./new_model.mps", "");
```

#### 参考文献：

【1】https://coin-or.github.io/Cbc/Doxygen/annotated.html

【2】https://coin-or.github.io/Cbc/

【3】https://github.com/coin-or/Cbc
