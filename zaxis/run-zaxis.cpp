
#include "zaxis/make-scenarios.h"

#include <map>
#include <set>

using namespace revoy;

enum ScenarioType {
  Swerve,
  StartStop,
};

std::map<std::string, Zaxis::AssemblyType> ASSEMBLIES{
    {"tractor-trailer", Zaxis::AssemblyType::TractorTrailerType},
    {"tractor-revoy-trailer", Zaxis::AssemblyType::TractorRevoyTrailerType},
};

std::map<std::string, ScenarioType> SCENARIOS{
    {"swerve", ScenarioType::Swerve},
    {"start-stop", ScenarioType::StartStop},
};

int main(int argc, char **argv) {

  const std::vector<std::string> args(argv + 1, argv + argc);

  assert(args.size() >= 2);

  const std::string assemblyName = args[0];
  const std::string scenarioName = args[1];

  assert(ASSEMBLIES.find(assemblyName) != ASSEMBLIES.end());
  assert(SCENARIOS.find(scenarioName) != SCENARIOS.end());

  Zaxis::Scenario scenario;
  ScenarioType scenarioType = SCENARIOS[scenarioName];
  Zaxis::AssemblyType assembly = ASSEMBLIES[assemblyName];

  if (scenarioType == ScenarioType::Swerve) {

    assert(args.size() == 5);
    const double speed = std::stod(args[2]);
    const double heading = std::stod(args[3]);
    const uint8_t count = std::stod(args[4]);

    scenario = MakeSwervingScenario(speed, heading, count);
    scenario.name = std::format("{}-{}", assemblyName, scenario.name);

  } else if (scenarioType == ScenarioType::StartStop) {

    assert(args.size() == 3);
    const double speed = std::stod(args[2]);
    scenario = MakeStartStopScenario(speed);
    scenario.name = std::format("{}-{}", assemblyName, scenario.name);

  } else {
    std::cout << "unexpected scenario type not handled: " << scenarioName
              << std::endl;
    exit(1);
  }

  Zaxis zaxis(std::move(scenario), assembly);

  zaxis.runThroughAllGoals();

  return 0;
}
