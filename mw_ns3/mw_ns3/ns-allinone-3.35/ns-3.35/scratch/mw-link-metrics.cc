#include "ns3/core-module.h"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

using namespace ns3;

struct Metrics
{
  double delay;
  double loss;
  double rate;
};

static Metrics
ComputeMetrics (double altitude)
{
  const double delay = std::min (160.0, std::max (0.0, 10.0 + altitude));
  const double loss = std::min (30.0, std::max (0.0, 0.3 * altitude));
  const double rate = std::max (300.0, std::min (6000.0, 6000.0 - 40.0 * altitude));
  return {delay, loss, rate};
}

int
main (int argc, char *argv[])
{
  std::string positions = "positions.txt";
  CommandLine cmd;
  cmd.AddValue ("positions", "Path to positions.txt", positions);
  cmd.Parse (argc, argv);

  double altitude = 0.0;
  std::ifstream input (positions.c_str (), std::ios::in);
  if (input.good ())
    {
      std::string line;
      while (std::getline (input, line))
        {
          if (line.empty ())
            {
              continue;
            }
          std::istringstream iss (line);
          double x = 0.0, y = 0.0, z = 0.0;
          if (iss >> x >> y >> z)
            {
              altitude = std::max (0.0, z);
            }
        }
    }
  else
    {
      NS_LOG_WARN ("mw-link-metrics: unable to open " << positions << "; using altitude = 0");
    }

  const Metrics metrics = ComputeMetrics (altitude);

  std::cout.setf (std::ios::fixed);
  std::cout << std::setprecision (2);
  std::cout << "DELAY_MS:" << metrics.delay << std::endl;
  std::cout << "LOSS_PCT:" << metrics.loss << std::endl;
  std::cout << "RATE_KBPS:" << metrics.rate << std::endl;
  return 0;
}
