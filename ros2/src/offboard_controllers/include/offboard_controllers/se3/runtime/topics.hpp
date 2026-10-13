// PX4 uXRCE-DDS topic constants generated from the shared catalog.

#pragma once

namespace px4_topics {
#define PX4_TOPIC(name, topic) inline constexpr char name[] = topic;
#include "px4_topics.def"
#undef PX4_TOPIC
} // namespace px4_topics
