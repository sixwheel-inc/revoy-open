#include "revoy-chrono/chrono-mcap.h"
#include "build-proto-fds/build-proto-fds.h"
#include "foxglove/FrameTransforms.pb.h"
#include "foxglove/SceneUpdate.pb.h"

#include <cstdint>
#include <string>

namespace revoy {

namespace {
foxglove::CubePrimitive MakeCube(const Eigen::Vector3d &dims,
                                 const Eigen::Vector3d &offset);

foxglove::SceneUpdate
MakeSceneUpdate(double writeTime,

                /// TODO: passing 3 bools like this is not ideal, but I don't
                /// want this function to be a member.
                bool doTractor, bool doRevoy, bool doTrailer);

} // namespace

ChronoMcap::ChronoMcap(mcap::McapWriter &writer) : writer(writer) {

  {
    mcap::Schema schema(
        "foxglove.FrameTransforms", "protobuf",
        revoy::BuildFileDescriptorSet(foxglove::FrameTransforms::descriptor())
            .SerializeAsString());
    writer.addSchema(schema);
    mcap::Channel channel("transforms", "protobuf", schema.id);
    writer.addChannel(channel);
    channelIds.insert({std::string("transforms"), channel.id});
  }
  {
    mcap::Schema schema(
        "foxglove.SceneUpdate", "protobuf",
        revoy::BuildFileDescriptorSet(foxglove::SceneUpdate::descriptor())
            .SerializeAsString());
    writer.addSchema(schema);
    mcap::Channel channel("scene", "protobuf", schema.id);
    writer.addChannel(channel);
    channelIds.insert({std::string("scene"), channel.id});
  }
}

/// call once for each vehicle that needs to be rendered
void ChronoMcap::attachVehicle(
    const std::string &vehicleName,
    std::shared_ptr<chrono::vehicle::ChVehicle> vehicle) {
  vehicles[vehicleName] = vehicle;
}

/// ChWheeledTrailer doesn't inherit from ChVehicle
/// call once for each trailer that needs to be rendered
void ChronoMcap::attachTrailer(
    const std::string &vehicleName,
    std::shared_ptr<chrono::vehicle::ChWheeledTrailer> trailer) {
  trailers[vehicleName] = trailer;
}

void ChronoMcap::step(double writeTime) {

  /// time since epoch in nanos
  const uint64_t nanos = static_cast<uint64_t>(writeTime * 1e9);

  /// split time into seconds and nanoseconds (that's how Foxglove / ROS do it)
  const uint64_t secondsPart = static_cast<uint64_t>(writeTime);
  const uint64_t nanosPart =
      static_cast<uint64_t>(writeTime * 1e9) % static_cast<uint64_t>(1e9);

  /// update each vehicles transform from the fixed frame
  foxglove::FrameTransforms transforms;
  for (auto &[name, vehicle] : vehicles) {

    foxglove::FrameTransform tf;
    tf.set_parent_frame_id("fixed");
    tf.set_child_frame_id(name);

    tf.mutable_timestamp()->set_seconds(secondsPart);
    tf.mutable_timestamp()->set_nanos(nanosPart);

    const auto rot = vehicle->GetRot();
    const auto pos = vehicle->GetPos();

    tf.mutable_translation()->set_x(pos.x());
    tf.mutable_translation()->set_y(pos.y());
    tf.mutable_translation()->set_y(pos.y());

    tf.mutable_rotation()->set_w(rot.e0());
    tf.mutable_rotation()->set_x(rot.e1());
    tf.mutable_rotation()->set_y(rot.e2());
    tf.mutable_rotation()->set_z(rot.e3());

    transforms.mutable_transforms()->Add(std::move(tf));
  }

  /// trailer has a different base class so handle them in a similar loop
  for (auto &[name, trailer] : trailers) {

    foxglove::FrameTransform tf;
    tf.set_parent_frame_id("fixed");
    tf.set_child_frame_id(name);

    tf.mutable_timestamp()->set_seconds(secondsPart);
    tf.mutable_timestamp()->set_nanos(nanosPart);

    const auto rot = trailer->GetChassis()->GetRot();
    const auto pos = trailer->GetChassis()->GetPos();

    tf.mutable_translation()->set_x(pos.x());
    tf.mutable_translation()->set_y(pos.y());
    tf.mutable_translation()->set_y(pos.y());

    tf.mutable_rotation()->set_w(rot.e0());
    tf.mutable_rotation()->set_x(rot.e1());
    tf.mutable_rotation()->set_y(rot.e2());
    tf.mutable_rotation()->set_z(rot.e3());

    transforms.mutable_transforms()->Add(std::move(tf));
  }

  std::string serialized = transforms.SerializeAsString();
  mcap::Message msg;
  msg.channelId = channelIds.at(std::string("transforms"));
  msg.sequence = static_cast<uint32_t>(frameIndex);
  msg.publishTime = nanos;
  msg.logTime = nanos;
  msg.data = reinterpret_cast<const std::byte *>(serialized.data());
  msg.dataSize = serialized.size();
  const auto res = writer.write(msg);
  if (!res.ok()) {
    std::cerr << "Failed to write message: " << res.message << "\n";
    writer.terminate();
    return;
  }

  const bool doTractor = vehicles.find("tractor") != vehicles.end();
  const bool doRevoy = vehicles.find("revoy") != vehicles.end();
  const bool doTrailer = trailers.find("trailer") != trailers.end();

  const foxglove::SceneUpdate scene =
      MakeSceneUpdate(writeTime, doTractor, doRevoy, doTrailer);
  {
    std::string serialized = scene.SerializeAsString();
    mcap::Message msg;
    msg.channelId = channelIds.at(std::string("scene"));
    msg.sequence = static_cast<uint32_t>(frameIndex);
    msg.publishTime = nanos;
    msg.logTime = nanos;
    msg.data = reinterpret_cast<const std::byte *>(serialized.data());
    msg.dataSize = serialized.size();
    const auto res = writer.write(msg);
    if (!res.ok()) {
      std::cerr << "Failed to write message: " << res.message << "\n";
      writer.terminate();
      return;
    }
  }

  frameIndex++;
  return;
}

namespace {
foxglove::CubePrimitive MakeCube(const Eigen::Vector3d &dims,
                                 const Eigen::Vector3d &offset) {

  foxglove::CubePrimitive cube;
  cube.mutable_size()->set_x(dims.x());
  cube.mutable_size()->set_y(dims.y());
  cube.mutable_size()->set_z(dims.z());
  cube.mutable_pose()->mutable_position()->set_x(offset.x());
  cube.mutable_pose()->mutable_position()->set_y(offset.y());
  cube.mutable_pose()->mutable_position()->set_z(offset.z());
  return cube;
};

foxglove::SceneEntity MakeEntity(const std::string &name, double writeTime) {

  const uint64_t secondsPart = static_cast<uint64_t>(writeTime);
  const uint64_t nanosPart =
      static_cast<uint64_t>(writeTime * 1e9) % static_cast<uint64_t>(1e9);

  foxglove::SceneEntity entity;
  entity.set_frame_id(name);
  entity.set_id(name);
  entity.mutable_timestamp()->set_seconds(secondsPart);
  entity.mutable_timestamp()->set_nanos(nanosPart);
  return entity;
};

foxglove::SceneUpdate MakeSceneUpdate(double writeTime, bool doTractor,
                                      bool doRevoy, bool doTrailer) {
  foxglove::SceneUpdate sceneUpdate;

  if (doTractor) {
    foxglove::SceneEntity tractor = MakeEntity("tractor", writeTime);
    auto tractorCube = MakeCube({7, 2.5, 4}, {0, 0, 2});
    /// blue
    tractorCube.mutable_color()->set_b(1);
    tractorCube.mutable_color()->set_a(0.8);
    tractor.mutable_cubes()->Add(std::move(tractorCube));
    sceneUpdate.mutable_entities()->Add(std::move(tractor));
  }

  if (doRevoy) {
    foxglove::SceneEntity revoy = MakeEntity("revoy", writeTime);
    auto revoyCube = MakeCube({7, 2.5, 4}, {0, 0, 2});
    /// orange
    revoyCube.mutable_color()->set_r(1);
    revoyCube.mutable_color()->set_g(0.5);
    revoyCube.mutable_color()->set_a(0.8);
    revoy.mutable_cubes()->Add(std::move(revoyCube));
    sceneUpdate.mutable_entities()->Add(std::move(revoy));
  }

  if (doTrailer) {
    foxglove::SceneEntity trailer = MakeEntity("trailer", writeTime);
    auto trailerCube = MakeCube({12, 2.5, 4}, {-1, 0, 2});
    /// white
    trailerCube.mutable_color()->set_r(1);
    trailerCube.mutable_color()->set_g(1);
    trailerCube.mutable_color()->set_b(1);
    trailerCube.mutable_color()->set_a(0.8);
    trailer.mutable_cubes()->Add(std::move(trailerCube));
    sceneUpdate.mutable_entities()->Add(std::move(trailer));
  }

  return sceneUpdate;
}

} // namespace

} // namespace revoy
