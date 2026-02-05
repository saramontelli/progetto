/*#include <TCanvas.h>
#include <TGraphErrors.h>
#include <TStyle.h>
#include <TAxis.h>


void grafici() {

  // ===== Canvas 1: distanza media =====
  TCanvas *c1 = new TCanvas("c1", "Distanza media vs tempo", 800, 600);

  // Colonne:
  // 1 = tempo
  // 2 = distanza media
  // 3 = dev std distanza
  TGraphErrors *gDist =
    new TGraphErrors("simulation_data.txt",
                     "%lg %lg %lg %*lg %*lg");

  gDist->SetTitle("Distanza media vs tempo;Tempo;Distanza media");
  gDist->SetMarkerStyle(20);
  gDist->SetMarkerSize(1.0);
  gDist->SetLineWidth(2);

  gDist->Draw("AP");

  c1->SaveAs("distanza_media.pdf");


  // ===== Canvas 2: velocità media =====
  TCanvas *c2 = new TCanvas("c2", "Velocita media vs tempo", 800, 600);

  // Colonne:
  // 1 = tempo
  // 4 = velocità media
  // 5 = dev std velocità
  TGraphErrors *gVel =
    new TGraphErrors("simulation_data.txt",
                     "%lg %*lg %*lg %lg %lg");

  gVel->SetTitle("Velocita media vs tempo;Tempo;Velocita media");
  gVel->SetMarkerStyle(21);
  gVel->SetMarkerSize(1.0);
  gVel->SetLineWidth(2);

  gVel->Draw("AP");

  c2->SaveAs("velocita_media.pdf");
}
*/
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "TCanvas.h"
#include "TGraph.h"
void simulation() {
  std::ifstream file("simulation_data.txt");
  float time;
  float mean_dist;
  float dev_dist;
  float mean_speed;
  float dev_speed;
  std::vector<float> times;
  std::vector<float> mean_dists;
  std::vector<float> mean_speeds;
  std::string line;
  std::getline(file, line);
  while (std::getline(file, line)) {
    std::istringstream stream(line);
    if (stream >> time >> mean_dist >> dev_dist >> mean_speed >> dev_speed) {
      times.push_back(time);
      mean_speeds.push_back(mean_speed);
      mean_dists.push_back(mean_dist);
    } else {
      std::cout << "Error in " << line << '\n';
    }
  }
  TGraph *graph_speed =
      new TGraph(times.size(), times.data(), mean_speeds.data());
  graph_speed->SetTitle("Media della velocit#grave{a} vs Tempo");
  graph_speed->GetXaxis()->SetTitle("Tempo (s)");
  graph_speed->GetYaxis()->SetTitle("Velocit#grave{a} Media");
  graph_speed->SetMarkerStyle(20);
  graph_speed->SetMarkerColor(kBlue);
  TGraph *graph_distance =
      new TGraph(times.size(), times.data(), mean_dists.data());
  graph_distance->SetTitle("Media della distanza vs Tempo");
  graph_distance->GetXaxis()->SetTitle("Tempo (s)");
  graph_distance->GetYaxis()->SetTitle("Distanza Media");
  graph_distance->SetMarkerStyle(21);
  graph_distance->SetMarkerColor(kRed);
  TCanvas *c1 = new TCanvas("c1", "Mean Distance vs Time", 800, 600);
  graph_distance->Draw("APL");
  TCanvas *c2 = new TCanvas("c2", "Mean Speed vs Time", 800, 600);
  graph_speed->Draw("APL");
  c1->Update();
  c2->Update();
}