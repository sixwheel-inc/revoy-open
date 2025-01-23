#pragma once

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>

namespace revoy {

///// copied from foxglove/mcap github repository
///// Builds a FileDescriptorSet of this descriptor and all transitive
///// dependencies, for use as a channel schema.
google::protobuf::FileDescriptorSet
BuildFileDescriptorSet(const google::protobuf::Descriptor *toplevelDescriptor);

} // namespace revoy
