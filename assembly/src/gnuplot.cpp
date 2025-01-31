
#include "chrono_postprocess/ChGnuPlot.h"

int main(int argc, char **argv) {
  assert(argc == 2);
  const std::string inputFile(argv[1]);

  {
    const std::string outputFile = inputFile + "_tractor.png";
    chrono::postprocess::ChGnuPlot mplot("/tmp/tmp_gnuplot_tractor.gpl");
    mplot.SetGrid();
    mplot.SetLabelX("time");
    mplot.SetLabelY("radians");
    mplot.OutputPNG(outputFile, 1000, 800);
    mplot.Plot(inputFile, 0, 4, "tractor roll", " with lines lt -1 lw 2");
    mplot.Plot(inputFile, 0, 6, "tractor yaw", " with lines lt 2 lw 2");
  }
  {
    const std::string outputFile = inputFile + "_revoy.png";
    chrono::postprocess::ChGnuPlot mplot("/tmp/tmp_gnuplot_revoy.gpl");
    mplot.SetGrid();
    mplot.SetLabelX("time");
    mplot.SetLabelY("radians");
    mplot.OutputPNG(outputFile, 1000, 800);
    mplot.Plot(inputFile, 0, 10, "revoy roll", " with lines lt -1 lw 2");
    mplot.Plot(inputFile, 0, 12, "revoy yaw", " with lines lt 2 lw 2");
  }
}
