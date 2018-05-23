
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

void CheckSolutionMikea(IloCP cp, IloIntervalSequenceVar seq, int j, IloIntervalVarArray2 tvisitTV, int size_missions_multipleTW) {
  // n Solution

  for (IloInt i=0; i<size_missions_multipleTW+2; i++){
    assert(tvisitTV[i][j].getImpl() != 0);
    cp.out() << i << " " << j << " " << cp.domain(tvisitTV[i][j]) << std::endl;
  }
  cout << endl;
  for(IloIntervalVar a = cp.getFirst(seq); a.getImpl()!=0; a = cp.getNext(seq, a))
      cp.out() << "Truck : "  << ":\t" << cp.domain(a) << endl; 
}

void CheckSolution(IloCP cp, IloIntervalSequenceVar seq) {
  // n Solution
  for(IloIntervalVar a = cp.getFirst(seq); a.getImpl()!=0; a = cp.getNext(seq, a))
      cp.out() << "Truck : "  << ":\t" << cp.domain(a) << endl; 
}


IloNum TWBuilder(const TSPTWDataDT &data, string filename) {

  IloEnv env;
  int cost;

  try {

    IloModel model(env);

    GOOGLE_PROTOBUF_VERIFY_VERSION;
    cplex_result::Result result;

    const int size_missions = data.SizeMissions();
    const int size_missions_multipleTW = data.size_missions_multipleTW();
    const int nbVehicle = data.NbVehicles();
    const vector<int> Capa = data.Capa_Vec();
    const vector<int> demand = data.demand();
    const vector<int> duration = data.duration();
    const vector<vector<int>> tw_start = data.TimeWindow_Start();
    const vector<vector<int>> tw_end = data.TimeWindow_End();
    const vector<int> tw_start_car = data.tw_start_car();
    const vector<int> tw_end_car = data.tw_end_car();
    const vector<vector<float>> Matrix = data.Matrice();
    const vector<vector<int>> indiceMultipleTW = data.indiceMultipleTW();

    char name[64];
    vector<int> t;

    cout << "NbVehicles : " << nbVehicle << " SizeMissions : " << size_missions 
    << " size_missions_multipleTW : " << size_missions_multipleTW << endl;
    for (int i=0; i<size_missions_multipleTW; i++){
      for (int j=0; j<tw_start[i].size(); j++){
        cout << "i:" << i << " j:" << j <<  " TimeWindow " << tw_start[i][j] << " " << tw_end[i][j] << endl;
      }
    }

    // A mettre dans un fichier !
    
    for (int i=0; i<nbVehicle; i++){
      cout << i << " Capacity" << " " << Capa[i] << endl;
    }

    for (int i=0; i<size_missions_multipleTW; i++){
      cout << i << " Demand" << " " << demand[i] << endl;
    }


    IloIntervalVarArray visit(env,size_missions_multipleTW+2); // real visits + 2  from and back
    IloIntervalVarArray2 tvisitTV(env,size_missions_multipleTW+2); // truck visits + from & back indexed by Visit x Vehicle
    for (IloInt i=0; i<size_missions_multipleTW+2; i++){
      if (i==0 || i==(size_missions_multipleTW+1)){
        sprintf(name,"FB-%ld",(long)i);
        visit[i] = IloIntervalVar(env,name);
        visit[i].setAbsent();
      }else {
        sprintf(name, "V-%ld-Node-%ld", (long) i -1, (long) i+1);
        visit[i] = IloIntervalVar(env, duration[i-1], name);
      }
      // assert ( 1 == 2);
      tvisitTV[i] = IloIntervalVarArray(env, nbVehicle);
      for (IloInt j=0; j<nbVehicle; j++){
        if (i==0 || i==(size_missions_multipleTW+1)){
          sprintf(name, "FB-%ld", (long) j);
          tvisitTV[i][j] = IloIntervalVar(env, name);
        }else {
          // sprintf(name, "Node-%ld", (long) i+1);
          tvisitTV[i][j] = IloIntervalVar(env);
          tvisitTV[i][j].setOptional();
          tvisitTV[i][j].setName(name);

        }
      }
    }


    IloIntervalVarArray2 tvisit(env,nbVehicle);  // truckvisit indexed by Vehicle X Visit
    for (IloInt i=0; i<nbVehicle; i++){
      tvisit[i] = IloIntervalVarArray(env, size_missions_multipleTW+2);
      for (IloInt j=0; j<size_missions_multipleTW+2; j++) {
        tvisit[i][j] = tvisitTV[j][i];
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

// Sequence Variable declaration + IloNoOverlap constraint
    IloIntervalSequenceVarArray seq(env,nbVehicle);
    for (int i=0; i<nbVehicle; i++){
      seq[i] = IloIntervalSequenceVar(env, tvisit[i]);
      model.add(IloNoOverlap(env,seq[i],Dist));
    }


// TimeWindow constraint (si plusieurs time window)
    for (IloInt i=0; i<size_missions+2; i++){
      IloNumToNumStepFunction StepFunction(env);
      if (i==0){
        for (IloInt j=0; j<tw_start_car.size(); j++){
          StepFunction.setValue(tw_start_car[j], tw_end_car[j], 1);
          tvisitTV[i][j].setIntensity(StepFunction);
          model.add(IloForbidExtent(env, tvisitTV[i][j], StepFunction));
        }
      }else if (i==size_missions+1){
        for (IloInt j=0; j<tw_start_car.size(); j++){
          StepFunction.setValue(tw_start_car[j], tw_end_car[j], 1);
          tvisitTV[i][j].setIntensity(StepFunction);
          model.add(IloForbidExtent(env, tvisitTV[i][j], StepFunction));
        }
      }else{
        for (int j=0; j<tw_start[i-1].size(); j++){
          StepFunction.setValue(tw_start[i-1][j], tw_end[i-1][j], 1);
        }
        for (IloInt j=0; j<nbVehicle; j++){
          tvisitTV[i][j].setIntensity(StepFunction);
          model.add(IloForbidExtent(env, tvisitTV[i][j], StepFunction));
        }
      }
    }


// IloFirst constraint
    for (IloInt i=0; i<nbVehicle; i++){
      model.add(IloFirst(env,seq[i],tvisit[i][0]));
      model.add(IloLast(env,seq[i],tvisit[i][size_missions+1]));
    }

// Objective declaration



    IloIntExpr ends(env);
    for (IloInt i=0; i<nbVehicle; i++){
      IloIntervalVar prec;
      prec = tvisitTV[size_missions_multipleTW+1][i];
      ends+=(IloEndOf(prec));
    }
    IloObjective objective = IloMinimize(env,(ends));
    model.add(objective);


// Capacity constraint
    if (demand.size() != 0){
      for (IloInt i=0; i<nbVehicle; i++){
        IloIntExpr expr2(env);
        for (IloInt j=1; j<size_missions_multipleTW+1; j++){
          expr2 += IloPresenceOf(env,tvisit[i][j])*demand[j-1];
        }
        model.add(expr2 <= Capa[i]);
      }
    }


// IloAlternative constraint
    for (IloInt i=1; i<size_missions_multipleTW+1; i++){
      if (i==0 || i==(size_missions_multipleTW+1)){
    // Do nothing
      }else {
        model.add(IloAlternative(env,visit[i], tvisitTV[i]));
      }
    }

    IloCP cp(model);
    cp.setParameter(IloCP::TimeLimit, 10);
    cp.setParameter(IloCP::FailLimit, 20000);  
    // cp.setParameter(IloCP::LogPeriod, IloIntMax);
    // cp.setParameter(IloCP::NoOverlapInferenceLevel, IloCP::Extended);
    // cp.setParameter(IloCP::TimeLimit, 3600);
    // IloSearchPhaseArray phaseArray(env);
    // phaseArray.add(IloSearchPhase(env, seq));



    
    if (!cp.solve()) {
      cout << "No solution - ERROR " << endl;
      return -1;
    } 
    
    cout << "Solution per truck :" << endl;
    for (IloInt j=0; j<nbVehicle; j++) {
      cout << "Truck " << j << endl;
      CheckSolution(cp, seq[j]);
    }

   
    cost = (cp.getObjValue());

    result.set_cost(cost);
  }
  
  catch (IloException& ex) {
    env.out() << "Caught exception: " << ex << std::endl;

    google::protobuf::ShutdownProtobufLibrary();
    env.end();

    return cost;
  }
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
