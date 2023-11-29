#include "CbcModel.hpp"

// Using CLP as the solver
#include "OsiClpSolverInterface.hpp"
#include <fstream>

struct ProblemInstance
{
    int numCols;
    std::vector<char> varTypes;
    std::vector<double> lb;
    std::vector<double> ub;
    std::vector<double> objCoeffs;
    
    int numRows;
    int numNonZeros;
    int objSense;
    std::vector<char> rowtypes;
    std::vector<double> rhs;
    std::vector<double> rhsrange;
    std::vector<int> rowStart;
    std::vector<int> colIdxs;
    std::vector<double> colCoeffs;
    std::vector<std::string> colName;
    std::vector<std::string> rowName;
};


ProblemInstance getProblemData(CbcModel& model)
{
    ProblemInstance data;
    data.numRows = model.solver()->getNumRows();
    data.numCols = model.solver()->getNumCols();
    data.numNonZeros = model.solver()->getNumElements(); // get the number of nonzeros elements 
    data.objSense = model.getObjSense(); // get the objective sense (1 for min (default), -1 for max,)
    
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
    
    const double* objCoeffs = model.solver()->getObjCoefficients();
    data.objCoeffs = std::vector<double>(data.numCols);
    for(int i = 0; i < data.numCols; i++) data.objCoeffs[i] = objCoeffs[i];

    const char* rowtypes = model.solver()->getRowSense();
    data.rowtypes = std::vector<char>(data.numRows);
    for(int i = 0; i < data.numRows; i++) data.rowtypes[i] = rowtypes[i];

    /** Get pointer to array[getNumRows()] of row constraint senses.
        <li>'L': <= constraint
        <li>'E': =  constraint
        <li>'G': >= constraint
        <li>'R': ranged constraint
        <li>'N': free constraint
    */

    const double* rhs = model.solver()->getRightHandSide();
    data.rhs = std::vector<double>(data.numRows);
    for(int i = 0; i < data.numRows; i++) data.rhs[i] = rhs[i];
    
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
    for (int i = 0; i < data.numNonZeros + 1; i++) data.rowStart[i] = rowStart[i];
    
    for (int i = 0; i < data.numCols; i++)
       data.colName.push_back(model.solver()->getColName(i));
    
    for (int i = 0; i < data.numRows; i++)
       data.rowName.push_back(model.solver()->getRowName(i));

    return data;
}


int main(int argc, const char *argv[]) {
  OsiClpSolverInterface solver1;

  // if (argc < 2) {
  //   printf("Please input the MPS file path\n");
  //   return 0;
  // }
  
  // // Read the mps file
  // int numMpsReadErrors = solver1.readMps(argv[1], "");
  // assert(numMpsReadErrors == 0);

  // Add variables
  int numVar = 5;
  CoinPackedVector v0;
  for (int i = 0; i < numVar; i++)
      solver1.addCol(v0, 0, 1.0, 0.0);

  // Set var upper bound and lower bound
  for (int i = 0; i < numVar; i++){ 
     solver1.setColUpper(i, 1.0);
     solver1.setColLower(i, 0.0);
     solver1.setInteger(i); // set variable to be integer
     // solver1.setContinuous(i); // set variable to be ocntinuous
     solver1.setColName(i, "x" + std::to_string(i)); // set variable name
  }

  // Add constraint: 11*x0 + 20*x1 + 3*x2 <= 12
  CoinPackedVector v1;
  v1.insert(0, 11.0); // add x0 coefficient 11.0
  v1.insert(1, 20.0); // add x1 coefficient 20.0
  v1.insert(2, 3.0);  // add x2 coefficient 3.0

  char rowsen = 'L'; //  constrants type is equal or less than 
  double rowrhs = 12.0; // right hand side of the constraint 
  double rowrng = 0.0; // A relaxation degree of 0 indicates that the constraint is strict.
  solver1.addRow(v1, rowsen, rowrhs, rowrng); 
  
  // Add constraint: 22 <= 0.5*x1 + x2 + 21.8*x3 <= DBL_MAX
  CoinPackedVector v2;
  double rowLowerBound = 22;  
  double rowUpperBound = DBL_MAX; 
  v2.insert(1, 0.5); 
  v2.insert(2, 1.0); 
  v2.insert(3, 21.8); 
  solver1.addRow(v2, rowLowerBound, rowUpperBound);
  
  // Remove rows and cols
  // std::vector<int> rowIdx = {0, 1};
  // solver1.deleteRows(rowIdx.size(), rowIdx.data()); 
  // std::vector<int> colIdx = {0, 1};
  // solver1.deleteCols(colIdx.size(), colIdx.data());
  
  // Set objective function: x0+x1+x2+x3+x4
  for (int i = 0; i < numVar; i++)
      solver1.setObjCoeff(i, 1.0); // set objective function coefficient
  
  // Set objective function sense (1 for min (default), -1 for max,)
  solver1.setObjSense(1);

  OsiClpSolverInterface solver2;
  solver2 = solver1; // copy the model 
  
  CbcModel model(solver1);

  ProblemInstance data = getProblemData(model);
  // data.printStat();

  // Set the number of threads to use in parallel
  model.setNumberThreads(16);

  model.setLogLevel(1); // log level range from 0-3
  model.setMaximumSeconds(60.0); // set time limit
  
  // set initial solution
  std::vector<std::pair< std::string, double > > initSol;
  for (int i = 0; i < numVar; i++)
      initSol.push_back(std::make_pair(data.colName[i], 0.0)); 
  model.setMIPStart(initSol);
  
  // start branch and bound tree search
  model.branchAndBound();
  
  /* Print the solution.  CbcModel clones the solver so we
     need to get current copy from the CbcModel */
  
  std::cout << std::endl << "----------         Solution         ----------" << std::endl;
  int status = model.status();

  if (status == -1)
      std::cout << "Problem has not been solved" << std::endl;
  else if (status == 0)
      std::cout << "Solution process is finished" << std::endl;
  

    /** Final status of problem
        -1 before branchAndBound
        0 finished - check isProvenOptimal or isProvenInfeasible to see if solution found
        (or check value of best solution)
        1 stopped - on maxnodes, maxsols, maxtime
        2 difficulties so run was abandoned
        (5 event user programmed event occurred)
    */
  
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

  
  // Write MPS file
  solver1.writeMps("./new_model.mps", "");


  return 0;
}
