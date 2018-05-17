#ifndef CPLEX_TUTORIALS_CPLUSPLUS_TSPTW_DATA_DT_H
#define CPLEX_TUTORIALS_CPLUSPLUS_TSPTW_DATA_DT_H

#include <ostream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <inttypes.h>
#include <fstream>
#include <math.h>
#include <string>

#include "cplex_vrp.pb.h"
// #include "routing_common/routing_common.h"

#define CUSTOM_MAX_INT (int) pow(2,30)

enum ShiftPref { ForceStart = 2, ForceEnd = 1, MinimizeSpan = 0 };

namespace std {

class TSPTWDataDT {
public:
  explicit TSPTWDataDT(string filename) {
    LoadInstance(filename);
  }
  void LoadInstance(const string & filename);

  vector<int> demand() const{
    return demand_;
  }

  vector<int> duration() const{
    return duration_;
  }

  vector<int> Capa_Vec() const {
    return CapaVec_;
  }

  int SizeMissions() const {
    return size_missions_;
  }

  int NbVehicles() const {
    return nbVecs_;
  }

  vector< vector<float> > Matrice() const{
    return matrice_;
  }

  vector<int> TimeWindow_Start() const {
    return timewindow_start_ ;
  }

  vector<int> TimeWindow_End() const {
    return timewindow_end_ ;
  }

  vector<vector<int>> indiceMultipleTW() const {
    return indiceMultipleTW_;
  }

  int size_missions_multipleTW() const {
    return size_missions_multipleTW_;
  }

private:
  void ProcessNewLine(char* const line);


  int size_missions_;
  int size_missions_multipleTW_;
  int nbVecs_;
  vector<int> CapaVec_;
  vector<int> demand_;
  vector<int> duration_;
  vector<int> timewindow_start_;
  vector<int> timewindow_end_;
  vector<vector<float>> matrice_;
  vector<vector<int>> indiceMultipleTW_;
  vector<int> indice_;
};

void TSPTWDataDT::LoadInstance(const string & filename) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  cplex_vrp::Problem problem;

  {
    fstream input(filename, ios::in | ios::binary);
    if (!problem.ParseFromIstream(&input)) {
      cout << "Failed to parse pbf." << endl;
    }
  }


  int nbService=0;
  for (const cplex_vrp::Service& service: problem.services()){
    nbService++;
  }
  size_missions_ = nbService;
  size_missions_multipleTW_ = size_missions_;

  int nbVehicle = 0;
  for (const cplex_vrp::Vehicle& vehicle: problem.vehicles()) {
    nbVehicle +=1;
  }
  nbVecs_=nbVehicle;

  int quant=0;
  int j=-1;
  for (const cplex_vrp::Service& service: problem.services()){
    j+=1;
    // if (service.duration() != 0){
      duration_.push_back(service.duration());
    // }
    for (const int& quantity: service.quantities()) {
      demand_.push_back(quantity/1000);
    }

    // if (service.time_windows().size()!=1 && service.time_windows().size()!=0){
    //   for (const int& quantity: service.quantities()) {
    //     demand_.push_back(quantity/1000);
    //   }
    //   duration_.push_back(service.duration());
    //   size_missions_multipleTW_+=1;
    //   for (int i=0; i<service.time_windows().size(); i++){
    //     indice_.push_back(i+j+1);
    //   }
    //   indiceMultipleTW_.push_back(indice_);
    //   indice_.clear();
    // }

    for (const cplex_vrp::TimeWindow& tw: service.time_windows()) {
      timewindow_start_.push_back(tw.start());
      timewindow_end_.push_back(tw.end());
    }
  }

  for (const cplex_vrp::Vehicle& vehicle: problem.vehicles()) {
    for (const cplex_vrp::Capacity& capacity: vehicle.capacities()) {
      CapaVec_.push_back(capacity.limit()/1000);
    }
  }

  for (int i=0; i<nbService+2; i++){
    vector<float> tab;
    for (int j=0; j<nbService +2; j++){
      for (const cplex_vrp::Matrix& matrix: problem.matrices()){
      tab.push_back(static_cast<float>(matrix.time(i * (nbService+2) + j)));
      } 
    }
    matrice_.push_back(tab);
  }

}

}  //  namespace operations_research

#endif //  OR_TOOLS_TUTORIALS_CPLUSPLUS_TSP_DATA_DT_H
