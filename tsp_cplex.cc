
#include <iostream>

// #define CPLEX_BASE_COMMANDLINEFLAGS_H_

#include "tsptw_data_dt.h"
#ifndef OR_TOOLS_BASE_COMMANDLINEFLAGS_H_
#define OR_TOOLS_BASE_COMMANDLINEFLAGS_H_

#include <gflags/gflags.h>

#endif
// #include "limits.h"

#include <cmath>
#include <string>

#include <cstdlib>
#include <stdlib.h>
#include <algorithm>
#include <vector>
#include "cplex_result.pb.h"

#include <iostream>
#include <sstream>
#include <string>

using namespace std;

#include <ilcplex/ilocplex.h>
#include <ilcp/cp.h>
#include <ilconcert/ilocsvreader.h>
#include <ilconcert/iloexpression.h>

DEFINE_string(instance_file, "", "instance file name or data");
DEFINE_string(solution_file, "", "instance file name or data");

IloNum TWBuilder(const TSPTWDataDT &data, string filename){

IloEnv env;

try {

IloModel model(env);

GOOGLE_PROTOBUF_VERIFY_VERSION;
cplex_result::Result result;

const int size_missions = data.SizeMissions();
const int size_missions_multipleTW = data.size_missions_multipleTW();
const int nbVechicle = data.NbVehicles();
const vector<int> Capa = data.Capa_Vec();
const vector<int> demand = data.demand();
const vector<int> duration = data.duration();
const vector<int> tw_start = data.TimeWindow_Start();
const vector<int> tw_end = data.TimeWindow_End();
const vector<vector<float>> Matrix = data.Matrice();
const vector<vector<int>> indiceMultipleTW = data.indiceMultipleTW();

cout << "MISSIONS " << size_missions_multipleTW << endl;
cout << "MISSIONS " << size_missions << endl;

if (tw_start.size() != 0){
for (int i=0; i<size_missions_multipleTW; i++){
  cout << i << " TimeWindow " << tw_start[i] << " " << tw_end[i] << endl;
}
}
for (int i=0; i<nbVechicle; i++){
  cout << i << " Capacity" << " " << Capa[i] << endl;
}

for (int i=0; i<size_missions_multipleTW; i++){
  cout << i << " Demand" << " " << demand[i] << endl;
}

char name[64];

IloIntervalVarArray visit(env,size_missions_multipleTW+2);
IloIntervalVarArray2 tvisit2(env,size_missions_multipleTW+2);
for (IloInt i=0; i<size_missions_multipleTW+2; i++){
  if (i==0 || i==(size_missions_multipleTW+1)){
    visit[i] = IloIntervalVar(env);
  }else{
    sprintf(name, "Node-%ld", (long) i+1);
    visit[i] = IloIntervalVar(env, duration[i-1], name);
  }
  tvisit2[i] = IloIntervalVarArray(env, nbVechicle);
  for (IloInt j=0; j<nbVechicle; j++){
    if (i==0 || i==(size_missions_multipleTW+1)){
      tvisit2[i][j] = IloIntervalVar(env);
    }else{
      tvisit2[i][j] = IloIntervalVar(env);
      tvisit2[i][j].setOptional();
    }
  }
}

IloIntervalVarArray2 tvisit(env,nbVechicle);
for (IloInt i=0; i<nbVechicle; i++){
  tvisit[i] = IloIntervalVarArray(env, size_missions_multipleTW+2);
  for (IloInt j=0; j<size_missions_multipleTW+2; j++) {
    tvisit[i][j] = tvisit2[j][i];
  }
}

// Matric declaration
IloTransitionDistance Dist(env, size_missions_multipleTW+2);

// if (size_missions_multipleTW == size_missions){
// If there is not services whith multiple timewindows, no work on the matrice
for (IloInt i=0; i<size_missions+2; i++) {
  for (IloInt j=0; j<size_missions+2; j++){
    Dist.setValue(i,j,Matrix[i][j]);
  }
}
// }else{
// // If there is not services whith multiple timewindows, no work on the matrice
// int k=0;
// for (IloInt i=0; i<size_missions_multipleTW+2; i++) {
//   for (IloInt j=0; j<size_missions_multipleTW+2; j++){
//     if (i < indiceMultipleTW[k][0]){
//       if (j < indiceMultipleTW[k][indiceMultipleTW[k].size()-1]){
//         Dist.setValue(i,j,Matrix[i][j]);
//       }else{
//         Dist.setValue(i,j,Matrix[i][j-1]);
//       }
//     }
//     if (i == indiceMultipleTW[k][0]){
//       if (j == i){
//         Dist.setValue(i,j,Matrix[i][j]);
//       }else{
//         Dist.setValue(i,j,Matrix[i][j-1]);
//       }
//     }
//     if (i > indiceMultipleTW[k][0]){
//       if (j == indiceMultipleTW[k][0]){
//         Dist.setValue(i,j,Matrix[i-1][j]);
//       }else{
//         Dist.setValue(i,j,Matrix[i-1][j-1]);
//       }
//     }
//   }
// }
// }

// Sequence Variable declaration + IloNoOverlap constraint
IloIntervalSequenceVarArray seq(env,nbVechicle);
for (int i=0; i<nbVechicle; i++){
  seq[i] = IloIntervalSequenceVar(env, tvisit[i]);
  model.add(IloNoOverlap(env,seq[i],Dist));
}

// Set timewindows
if (tw_start.size() != 0){
  for (IloInt j=0; j<nbVechicle; j++){
    for (IloInt i=1; i<size_missions_multipleTW+1; i++){
      tvisit2[i][j].setStartMin(tw_start[i-1]);
      tvisit2[i][j].setEndMax(tw_end[i-1]);
    }
  }
}

// We can take only one of the duplicated services (a service is duplicate if it has severals timewindows)
// if (indiceMultipleTW.size() != 0){
// for (IloInt i=0; i<indiceMultipleTW.size(); i++){
//   IloIntExpr eprr(env);
//   for (IloInt j=0; j<indiceMultipleTW[i].size(); j++){
//     for (IloInt k=0; k<nbVechicle; k++){
//       eprr += IloPresenceOf(env, tvisit2[indiceMultipleTW[i][j]][k]);
//     }
//   }
//   model.add(eprr == 1);
// }
// }

// TimeWindow constraint (si plusieurs time window)
// for (IloInt i=1; i<size_missions+1; i++){
//   IloNumToNumStepFunction StepFunction(env,start[i-1], end[i-1]);
//   // StepFunction.setValue(tw_start[i-1], tw_end[i-1],0);
//   // cout << "stepfunction :" << StepFunction << endl;
//   for (IloInt j=0; j<nbVechicle; j++){
//     tvisit2[i][j].setIntensity(StepFunction);
//     model.add(IloForbidExtent(env, tvisit2[i][j], StepFunction));
//   }
// }

// IloFirst constraint
for (IloInt i=0; i<nbVechicle; i++){
  model.add(IloFirst(env,seq[i],tvisit[i][0]));
}

// Objective declaration
IloIntExpr ends(env);
for (IloInt i=0; i<nbVechicle; i++){
  IloIntervalVar prec;
  prec = tvisit2[size_missions_multipleTW+1][i];
  ends+=(IloEndOf(prec));
}
IloObjective objective = IloMinimize(env,(ends));
model.add(objective);

// Capacity constraint
if (demand.size() != 0){
for (IloInt i=0; i<nbVechicle; i++){
  IloIntExpr expr2(env);
  for (IloInt j=1; j<size_missions_multipleTW+1; j++){
    expr2 += IloPresenceOf(env,tvisit[i][j])*demand[j-1];
  }
  model.add(expr2 <= Capa[i]);
}
}

//IloLast constraint
for (IloInt i=0; i<nbVechicle; i++){
  model.add(IloLast(env,seq[i],tvisit[i][size_missions_multipleTW+1]));
}

// IloAlternative constraint
for (IloInt i=1; i<size_missions_multipleTW+1; i++){
  if (i==0 || i==(size_missions_multipleTW+1)){
    // Do nothing
  }else {
    model.add(IloAlternative(env,visit[i], tvisit2[i]));
  }
}

// Declaration of cp resolution
// IloCP cp(model);
// cp.setParameter(IloCP::LogPeriod, IloIntMax);
// cp.setParameter(IloCP::NoOverlapInferenceLevel, IloCP::Extended);
// cp.setParameter(IloCP::SearchType, IloCP::DepthFirst);
// // cp.setParameter(IloCP::LogVerbosity,IloCP::Quiet);
// cp.setParameter(IloCP::TimeLimit, 1800);
// IloSearchPhaseArray phaseArray(env);
// phaseArray.add(IloSearchPhase(env, seq));

IloCP cp(model);
cp.setParameter(IloCP::LogPeriod, IloIntMax);
cp.setParameter(IloCP::NoOverlapInferenceLevel, IloCP::Extended);
cp.setParameter(IloCP::TimeLimit, 36);
IloSearchPhaseArray phaseArray(env);
phaseArray.add(IloSearchPhase(env, seq));

cp.solve();


// A mettre dans un fichier !
if (tw_start.size() != 0){
for (int i=0; i<size_missions_multipleTW; i++){
  cout << i << " TimeWindow " << tw_start[i] << " " << tw_end[i] << endl;
}
}
for (int i=0; i<nbVechicle; i++){
  cout << i << " Capacity" << " " << Capa[i] << endl;
}

for (int i=0; i<size_missions_multipleTW; i++){
  cout << i << " Demand" << " " << demand[i] << endl;
}

for (IloInt j=0; j<nbVechicle; j++) {
  for (IloInt i=0; i<size_missions_multipleTW+2; i++){
    cp.out() << i << " " << j << " " << cp.domain(tvisit2[i][j]) << std::endl;
  }
  cout << endl;
}

int cost = (cp.getObjValue());

result.set_cost(cost);

google::protobuf::ShutdownProtobufLibrary();
return cp.getObjValue();

} catch (IloException& ex) {
  env.out() << "Caught exception: " << ex << std::endl;
}
env.end();
}





int main(int argc, char **argv){

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  TSPTWDataDT tsptw_data(FLAGS_instance_file);
  
  // cout << "Solution : " << endl;
  IloNum cost = TWBuilder(tsptw_data, FLAGS_solution_file);
  cout << "Objective value : " << cost << endl;

  gflags::ShutDownCommandLineFlags();

  return 0;
}
