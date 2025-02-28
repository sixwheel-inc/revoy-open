#define MCAP_IMPLEMENTATION

#include "planning/simpl-mcap.h"
#include "planning/foxglove-utils.h"
#include "planning/occupancy-grid.h"

#include "build-proto-fds/build-proto-fds.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <unordered_set>

namespace planning {

namespace {

const static std::string GRID_TOPIC = "occupancy";
const static std::string ACTORS_TOPIC = "actors";
const static std::string SCENARIO_TOPIC = "scenario";

// planner debug topics, configurable to multiple planners
constexpr std::string GRAPH_TOPIC = "{}-graph";
constexpr std::string PATH_TOPIC = "{}-path";

} // namespace

SimplMcap::SimplMcap(const std::string outputFilename) {

  // default mcap writer
  auto options = mcap::McapWriterOptions("");
  const auto res = writer.open(outputFilename, options);

  // hard exit if mcap isn't possible
  // TODO: more graceful exit case?
  if (!res.ok()) {
    std::cerr << "Failed to open " << outputFilename
              << " for writing: " << res.message << std::endl;
    exit(1);
  }

  // the following sets up all the topics that will be written to in the mcap
  // the expected topics are hardcoded, this could be upgraded to be more
  // configurable

  // will visualize the occupancy grid
  addTopic("foxglove.Grid",
           revoy::BuildFileDescriptorSet(foxglove::Grid::descriptor())
               .SerializeAsString(),
           GRID_TOPIC);

  // will visualize changes to Revoy and other agents
  addTopic("foxglove.SceneUpdate",
           revoy::BuildFileDescriptorSet(foxglove::SceneUpdate::descriptor())
               .SerializeAsString(),
           ACTORS_TOPIC);

  // TODO replace hardcoded values with configuration
  static const std::vector<std::string> PLANNER_NAMES{"proximity"};

  // will visualize provided planner debug info, could be costly
  for (const std::string &name : PLANNER_NAMES) {

    // will visualize all the nodes in the search graph
    addTopic("foxglove.SceneUpdate",
             revoy::BuildFileDescriptorSet(foxglove::SceneUpdate::descriptor())
                 .SerializeAsString(),
             std::format(GRAPH_TOPIC, name));

    // will visualize the solution (or approximate solution)
    addTopic("foxglove.SceneUpdate",
             revoy::BuildFileDescriptorSet(foxglove::SceneUpdate::descriptor())
                 .SerializeAsString(),
             std::format(PATH_TOPIC, name));
  }

  // will write scenario, will display bounds, start and goal poses, etc
  addTopic("foxglove.SceneUpdate",
           revoy::BuildFileDescriptorSet(foxglove::SceneUpdate::descriptor())
               .SerializeAsString(),
           SCENARIO_TOPIC);
}

void SimplMcap::write(const Simpl &simpl, int64_t time) {
  const Scene scene = SimplToScene(simpl, time);
  write(scene, time);
}

void SimplMcap::write(const Scene &scene, int64_t writeTime) {

  // add Revoy and other agent footprints to scene
  const auto actors =
      MakeActorSceneUpdate(scene, writeTime).SerializeAsString();
  writeTopic(actors, ACTORS_TOPIC, writeTime);

  // show occupancy grid (indicating which tiles are full, which are empty)
  const auto grid =
      MakeGrid(scene.grid, scene.revoyPose, writeTime).SerializeAsString();
  writeTopic(grid, GRID_TOPIC, writeTime);

  // show the bounds and start/goal poses
  // TODO: static, could just be written once
  const auto scenario =
      MakeScenarioSceneUpdate(scene.scenario, writeTime).SerializeAsString();
  writeTopic(scenario, SCENARIO_TOPIC, writeTime);

  // show search graphs and solutions
  for (const auto &[name, plannerViz] : scene.planners) {

    // somewhat costly debug visualization of the entire search graph
    const auto graph =
        MakeGraphSceneUpdate(plannerViz.graph, writeTime).SerializeAsString();
    writeTopic(graph, std::format(GRAPH_TOPIC, name), writeTime);

    // output path
    const auto solution =
        MakePathSceneUpdate(plannerViz.solution, writeTime).SerializeAsString();
    writeTopic(solution, std::format(PATH_TOPIC, name), writeTime);
  }

  // increments frame for next mcap write
  frameIndex++;
}

void SimplMcap::addTopic(const std::string &type, const std::string &desc,
                         const std::string &topic) {
  mcap::Schema schema(type, "protobuf", desc);
  writer.addSchema(schema);

  mcap::Channel channel(topic, "protobuf", schema.id);
  writer.addChannel(channel);
  channelIds.insert({std::string(topic), channel.id});
};

void SimplMcap::writeTopic(const std::string &serialized,
                           const std::string &topic, double writeTime) {

  mcap::Message msg;
  msg.channelId = channelIds.at(topic);
  msg.sequence = static_cast<uint32_t>(frameIndex);
  msg.publishTime =
      static_cast<uint64_t>(writeTime) * static_cast<int64_t>(1e3);
  msg.logTime = static_cast<uint64_t>(writeTime) * static_cast<int64_t>(1e3);
  msg.data = reinterpret_cast<const std::byte *>(serialized.data());
  msg.dataSize = serialized.size();
  const auto res = writer.write(msg);
  if (!res.ok()) {
    std::cerr << "Failed to write message: " << res.message << "\n";
    writer.terminate();
    return;
  }
}

SimplMcap::~SimplMcap() { writer.close(); }

Scene SimplMcap::SimplToScene(const Simpl &simpl, int64_t time) {

  Scene scene;
  scene.scenario = simpl.getScenario();
  scene.revoy = simpl.getRevoyEv().getBody(scene.scenario.bodyParams);
  scene.revoyPose = simpl.getRevoyEv().getHookedPose();
  scene.visibleEntities = simpl.getVisibleFootprints(time);
  scene.grid = simpl.getProximityPlanner().getLastOccupancyGrid();

  // TODO for now this is hardcoded but we can make this configurable
  scene.planners["proximity"].solution =
      simpl.getProximityPlanner().getLastSolution();
  scene.planners["proximity"].graph =
      simpl.getProximityPlanner().getLastGraph();

  return scene;
}

} // namespace planning
