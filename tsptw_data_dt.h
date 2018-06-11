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

#define CUSTOM_MAX_INT (int) pow(2,30)

enum ShiftPref { ForceStart = 2, ForceEnd = 1, MinimizeSpan = 0 };

namespace std {

class TSPTWDataDT {
public:
  explicit TSPTWDataDT(string filename) {
    LoadInstance(filename);
  }
  void LoadInstance(const string & filename);

  vector<int> Demands() const {
    return Demands_;
  }

  vector<int> Durations() const {
    return Durations_;
  }

  vector<int> CapaVecs() const {
    return CapaVec_;
  }

  int SizeMissions() const {
    return size_missions_;
  }

  int NbVehicles() const {
    return nbVecs_;
  }

  vector<vector<float>> Matrice() const {
    return matrice_;
  }

  vector<vector<int>> TimeWindowStarts() const {
    return timewindow_start_ ;
  }

  vector<vector<int>> TimeWindowEnds() const {
    return timewindow_end_ ;
  }

  vector<vector<int>> IndiceMultipleTW() const {
    return indiceMultipleTW_;
  }

  int SizeMissionsMultipleTW() const {
    return size_missions_multipleTW_;
  }

  vector<int> TwStartCar() const {
    return tw_start_car_;
  }

  vector<int> TwEndCar() const {
    return tw_end_car_;
  }

private:
  void ProcessNewLine(char* const line);


  int size_missions_;
  int size_missions_multipleTW_;
  int nbVecs_;
  vector<int> CapaVec_;
  vector<int> Demands_;
  vector<int> Durations_;
  vector<int> tw_start_car_;
  vector<int> tw_end_car_;
  vector<vector<int>> timewindow_start_;
  vector<vector<int>> timewindow_end_;
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
  for (const cplex_vrp::Service& service: problem.services()) {
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
  int j=0;
  int tws = 0;
  vector<int> start;
  vector<int> end;
  for (const cplex_vrp::Service& service: problem.services()) {
    j+=1;
    // if (service.Durations() != 0) {
    Durations_.push_back(service.duration());
    // }
    for (const int& quantity: service.quantities()) {
      Demands_.push_back(quantity/1000);
    }

    // if (service.time_windows().size()!=1 && service.time_windows().size()!=0) {
    //   for (const int& quantity: service.quantities()) {
    //     Demands_.push_back(quantity/1000);
    //   }
    //   Durations_.push_back(service.Durations());
    //   size_missions_multipleTW_+=1;
    //   for (int i=0; i<service.time_windows().size(); i++) {
    //     indice_.push_back(i+j+1);
    //   }
    //   indiceMultipleTW_.push_back(indice_);
    //   indice_.clear();
    // }
    for (const cplex_vrp::TimeWindow& tw: service.time_windows()) {
      start.push_back(tw.start());
      end.push_back(tw.end());
    }
    timewindow_start_.push_back(start);
    timewindow_end_.push_back(end);
    start.clear();
    end.clear();
  }

  for (const cplex_vrp::Vehicle& vehicle: problem.vehicles()) {
    for (const cplex_vrp::Capacity& capacity: vehicle.capacities()) {
      CapaVec_.push_back(capacity.limit()/1000);
    }
    tw_start_car_.push_back(vehicle.time_window().start());
    tw_end_car_.push_back(vehicle.time_window().end());
  }


  for (const cplex_vrp::Matrix& matrix: problem.matrices()) {
    int size_matrix5 = sqrt(matrix.time().size());
    if (size_matrix5 == size_missions_ + 2) {
      int size_matrix = sqrt(matrix.time().size());
      for (int i=0; i<size_matrix; i++) {
        vector<float> tab;
        for (int j=0; j<size_matrix; j++) {
        // for (const cplex_vrp::Matrix& matrix: problem.matrices()) {
          tab.push_back(static_cast<float>(matrix.time(i * (size_matrix) + j)));
        } 
        matrice_.push_back(tab);
      }
    } else {
      int size_matrix = sqrt(matrix.time().size()) - (CapaVec_.size()-1)*2;
      for (int i=0; i<size_matrix; i++) {
        vector<float> tab;
        for (int j=0; j<size_matrix; j++) {
          // for (const cplex_vrp::Matrix& matrix: problem.matrices()) {
          tab.push_back(static_cast<float>(matrix.time(i * (size_matrix + (CapaVec_.size()-1)*2) + j)));
        } 
        matrice_.push_back(tab);
        // for (int i=0; i<matrice_.size(); i++) {
        //   cout << matrice_[i] << " ";
        // }
        // cout << endl;
      }
    }
  }
}

}  //  namespace operations_research

#endif //  OR_TOOLS_TUTORIALS_CPLUSPLUS_TSP_DATA_DT_H
