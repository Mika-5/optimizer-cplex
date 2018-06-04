
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
#include <boost/algorithm/string.hpp>

DEFINE_string(instance_file, "", "instance file name or data");
DEFINE_string(solution_file, "", "instance file name or data");

void CheckSolutionMikea(IloCP cp, IloIntervalSequenceVar seq, int j, IloIntervalVarArray2 tvisitTV, int size_missions_multipleTW) {
  // n Solution

  for (IloInt i=0; i<size_missions_multipleTW+2; i++){
    assert(tvisitTV[i][j].getImpl() != 0);
    cp.out() << i << " " << j << " " << cp.domain(tvisitTV[i][j]) << std::endl;
  }
  cout << endl;
  // for(IloIntervalVar a = cp.getFirst(seq); a.getImpl()!=0; a = cp.getNext(seq, a))
  //     cp.out() << "Truck : "  << ":\t" << cp.domain(a) << endl; 
}

void CheckSolution(IloCP cp, IloIntervalSequenceVar seq) {
  // n Solution
  for(IloIntervalVar a = cp.getFirst(seq); a.getImpl()!=0; a = cp.getNext(seq, a))
      cp.out() << "Truck : "  << ":\t" << cp.domain(a) << endl; 
}


IloInt TWBuilder(const TSPTWDataDT &data, string filename) {

  GOOGLE_PROTOBUF_VERIFY_VERSION;
  cplex_result::Result result;

  IloEnv env;
  IloInt cost;
  IloIntExpr realCost(env);

  try {

    IloModel model(env);

    const int size_missions = data.SizeMissions();
    const int size_missions_multipleTW = data.SizeMissionsMultipleTW();
    const int nbVehicle = data.NbVehicles();
    const vector<int> Capa = data.CapaVecs();
    const vector<int> demand = data.Demands();
    const vector<int> duration = data.Durations();
    const vector<vector<int>> tw_start = data.TimeWindowStarts();
    const vector<vector<int>> tw_end = data.TimeWindowEnds();
    const vector<int> tw_start_car = data.TwStartCar();
    const vector<int> tw_end_car = data.TwEndCar();
    const vector<vector<float>> Matrix = data.Matrice();
    const vector<vector<int>> indiceMultipleTW = data.IndiceMultipleTW();

    char name[64];
    vector<int> t;

    int max = 0;
    for (int i=0; i<Matrix.size(); i++) {
      for (int j=0; j<Matrix[i].size(); j++) {
        if (Matrix[i][j] > max){
          max = Matrix[i][j];
        }
      }
    }

    IloIntervalVarArray visit(env, size_missions_multipleTW+2); // real visits + 2  from and back
    IloIntervalVarArray2 tvisitTV(env, size_missions_multipleTW+2); // truck visits + from & back indexed by Visit x Vehicle
    for (IloInt i=0; i<size_missions_multipleTW+2; i++) {
      if (i==size_missions_multipleTW || i==(size_missions_multipleTW+1)) {
        // sprintf(name, "FB-%ld", (long)i);
        visit[i] = IloIntervalVar(env, name);
        // visit[i].setAbsent();
      }else {
        // sprintf(name, "V-%ld-Node-%ld", (long) i -1, (long) i+1);
        visit[i] = IloIntervalVar(env, duration[i], name);
        visit[i].setOptional();  // to enable to find always a solution.
        // NEED to add a cost to penalize non performing visit.
        // visit[i].setOptional();
      }
      // assert ( 1 == 2);
      tvisitTV[i] = IloIntervalVarArray(env, nbVehicle);
      for (IloInt j=0; j<nbVehicle; j++) {
        if (i==size_missions_multipleTW || i==(size_missions_multipleTW+1)) {
          sprintf(name, "FB-%ld", (long) j);
          tvisitTV[i][j] = IloIntervalVar(env, name);
        }else {
          sprintf(name, "Node-%ld", (long) i);
          tvisitTV[i][j] = IloIntervalVar(env);
          tvisitTV[i][j].setOptional();
          tvisitTV[i][j].setName(name);
        }
      }
    }


    IloIntervalVarArray2 tvisit(env, nbVehicle);  // truckvisit indexed by Vehicle X Visit
    for (IloInt i=0; i<nbVehicle; i++) {
      tvisit[i] = IloIntervalVarArray(env, size_missions_multipleTW+2);
      for (IloInt j=0; j<size_missions_multipleTW+2; j++) {
        tvisit[i][j] = tvisitTV[j][i];
      }
    }


// Matric declaration
    IloTransitionDistance Dist(env, size_missions_multipleTW+2);
    for (IloInt i=0; i<size_missions+2; i++) {
      for (IloInt j=0; j<size_missions+2; j++) {
        Dist.setValue(i, j, Matrix[i][j]);
      }
    }

// Sequence Variable declaration + IloNoOverlap constraint
    IloIntervalSequenceVarArray seq(env, nbVehicle);
    for (int i=0; i<nbVehicle; i++) {
      seq[i] = IloIntervalSequenceVar(env, tvisit[i]);
      model.add(IloNoOverlap(env, seq[i], Dist));
    }


// TimeWindow constraint (si plusieurs time window)
    for (IloInt i=0; i<size_missions+2; i++) {
      IloNumToNumStepFunction StepFunction(env);
      if (i==size_missions) {
        for (IloInt j=0; j<tw_start_car.size(); j++) {
          StepFunction.setValue(tw_start_car[j], tw_end_car[j], 100);
          tvisitTV[i][j].setIntensity(StepFunction);
          model.add(IloForbidExtent(env, tvisitTV[i][j], StepFunction));
        }
      }else if (i==size_missions+1) {
        for (IloInt j=0; j<tw_start_car.size(); j++) {
          StepFunction.setValue(tw_start_car[j], tw_end_car[j], 100);
          tvisitTV[i][j].setIntensity(StepFunction);
          model.add(IloForbidExtent(env, tvisitTV[i][j], StepFunction));
        }
      }else {
        for (int j=0; j<tw_start[i].size(); j++) {
          StepFunction.setValue(tw_start[i][j], tw_end[i][j], 100);
        }
        for (IloInt j=0; j<nbVehicle; j++) {
          tvisitTV[i][j].setIntensity(StepFunction);
          model.add(IloForbidExtent(env, tvisitTV[i][j], StepFunction));
        }
      }
    }


// IloFirst constraint
    for (IloInt i=0; i<nbVehicle; i++) {
      model.add(IloFirst(env, seq[i], tvisit[i][size_missions]));
      model.add(IloLast(env, seq[i], tvisit[i][size_missions+1]));
    }

    IloIntExpr penalty(env);
    for (IloInt i=0; i<size_missions; i++) {
      penalty += 1 - IloPresenceOf(env, visit[i]);
    }


// Objective declaration
    IloIntExpr ends(env);
    for (IloInt i=0; i<nbVehicle; i++) {
      IloIntervalVar prec;
      prec = tvisitTV[size_missions_multipleTW+1][i];
      IloIntervalVar prec2;
      prec2 = tvisitTV[size_missions][i];
      ends+=(IloEndOf(prec) - IloStartOf(prec2));
    }
    ends+= max*10*penalty;
    IloObjective objective = IloMinimize(env, (ends));
    model.add(objective);


// Capacity constraint
    if (demand.size() != 0) {
      for (IloInt t = 0 ; t < nbVehicle ; t++) {
        IloCumulFunctionExpr truckLoad(env);
        for (IloInt i = 0 ; i < size_missions_multipleTW ; i++) 
          truckLoad += IloStepAtStart(tvisitTV[i][t], demand[i]);
        model.add(truckLoad <= Capa[t]);
      } 
    }


// IloAlternative constraint
    for (IloInt i=0; i<size_missions_multipleTW; i++) {
      if (i==size_missions_multipleTW || i==(size_missions_multipleTW+1)) {
    // Do nothing
      }else {
        model.add(IloAlternative(env, visit[i], tvisitTV[i]));
      }
    }

    IloCP cp(model);
    cp.setParameter(IloCP::LogPeriod, IloIntMax);
    cp.setParameter(IloCP::NoOverlapInferenceLevel, IloCP::Extended);
    cp.setParameter(IloCP::TimeLimit, 300);
    IloSearchPhaseArray phaseArray(env);
    phaseArray.add(IloSearchPhase(env, seq));

    if (!cp.solve()) {
      return -1;
    } 

   
    cost = (cp.getObjValue());


    result.set_cost(cost);
    result.clear_routes();
    for (int i=0; i<nbVehicle; i++) {
      cplex_result::Route* route = result.add_routes();
      int index=1;
      int quant = 0;
      for(IloIntervalVar a = cp.getFirst(seq[i]); a.getImpl()!=0; a = cp.getNext(seq[i], a)) {
        // cp.out() << "Truck : "  << ":\t" << cp.domain(a) << endl;
        cplex_result::Activity* activity = route->add_activities();
        vector<string> nickname;
        string str = a.getName();
        boost::split(nickname, str, [](char c){return c == '-';});
        int service_nb = stoi(nickname[1]);
        if (nickname[0] == "FB" && index==1) {
          activity->set_start_time(cp.getStart(a));
          activity->set_type("start");
          activity->set_index(0);
          index=2;
        }else if(nickname[0] == "FB" && index==2) {
          activity->set_start_time(cp.getStart(a));
          activity->set_type("end");
          activity->set_index(0);
          index=1;
        }else {
          activity->set_start_time(cp.getStart(a));
          activity->set_type("service");
          int ind = service_nb;
          activity->set_index(ind);
          quant += demand[ind];
          activity->add_quantities(quant);
        }
      }
    }
    fstream output(filename, ios::out | ios::trunc | ios::binary);
    if (!result.SerializeToOstream(&output)) {
      cout << "Failed to write result." << endl;
      return -1;
    }
    output.close();
  }

  catch (IloException& ex) {
    env.out() << "Caught exception: " << ex << std::endl;
    google::protobuf::ShutdownProtobufLibrary();
    env.end();
  }

  return cost;
}


int main(int argc, char **argv){

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  TSPTWDataDT tsptw_data(FLAGS_instance_file);
  IloInt cost = TWBuilder(tsptw_data, FLAGS_solution_file);
  cout << "Objective value : " << cost << endl;
  gflags::ShutDownCommandLineFlags();

  return 0;
}
