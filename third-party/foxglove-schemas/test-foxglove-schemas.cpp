#include <catch2/catch_test_macros.hpp>

#include "foxglove/Grid.pb.h"
#include "foxglove/Pose.pb.h"
#include "foxglove/PoseInFrame.pb.h"
#include "foxglove/Vector3.pb.h"

#include <iostream>

TEST_CASE("test foxglove protos cpp schemas") {
  foxglove::Vector3 v;
  v.set_x(1);
  v.set_y(2);
  v.set_z(3);
  std::cout << "test foxglove schema, Vector3 " << v.DebugString() << std::endl;

  foxglove::Quaternion q;
  q.set_x(1);
  q.set_y(2);
  q.set_z(3);
  q.set_w(4);
  std::cout << "test foxglove schema, Quat" << q.DebugString() << std::endl;

  foxglove::Grid g;
  g.set_column_count(4);

  foxglove::Pose p;
  *p.mutable_position() = v;
  std::cout << "test foxglove schema, Pose " << p.position().DebugString()
            << std::endl;

  foxglove::PoseInFrame pf;
  pf.set_frame_id("aaaa");
  *pf.mutable_pose() = p;
  std::cout << "test foxglove schema, PoseInFrame " << pf.pose().DebugString()
            << std::endl;
}
