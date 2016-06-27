#ifndef OSRM_EXTRACTOR_CLASSIFICATION_DATA_HPP_
#define OSRM_EXTRACTOR_CLASSIFICATION_DATA_HPP_

#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>

#include <osmium/osm.hpp>

namespace osrm
{
namespace extractor
{
namespace guidance
{

struct RoadClassification
{
    // a class that behaves like a motorway (separated directions)
    bool motorway_class : 1;
    // all types of link classes
    bool link_class : 1;
    // a low priority class is a pure connectivity way. It can be ignored in multiple decisions
    // (e.g. fork on a primary vs service will not happen)
    bool may_be_ignored : 1;
    // the road priority is used as an indicator for forks. If the roads are of similar priority
    // (difference <=1), we can see the road as a fork. Else one of the road classes is seen as
    // obvious choice
    unsigned priority : 5;

    // default construction
    RoadClassification()
        : motorway_class(false), link_class(false), may_be_ignored(false), priority(0)
    {
    }

    RoadClassification(bool motorway_class, bool link_class, bool may_be_ignored, unsigned priority)
        : motorway_class(motorway_class), link_class(link_class), may_be_ignored(may_be_ignored),
          priority(priority)
    {
    }

    inline bool isMotorwayClass() const { return motorway_class && !link_class; }

    inline bool isRampClass() const { return motorway_class && link_class; }

    inline bool isLinkClass() const { return link_class; }

    inline bool isLowPriorityRoadClass() const { return may_be_ignored; }

    inline bool operator==(const RoadClassification &other) const
    {
        return motorway_class == other.motorway_class && link_class == other.link_class &&
               may_be_ignored == other.may_be_ignored && priority == other.priority;
    }

    inline std::string toString() const
    {
        return std::string() + (motorway_class ? "motorway" : "normal") +
               (link_class ? "_link" : "") + (may_be_ignored ? " ignorable" : " important") +
               std::to_string(priority);
    }

    inline void set(const osmium::Way &way)
    {
        const static auto class_hash = [] {
            std::unordered_map<std::string, unsigned> hash;
            hash["motorway"] = 0;
            hash["motorway_link"] = 10;
            hash["trunk"] = 2;
            hash["trunk_link"] = 10;
            hash["primary"] = 4;
            hash["primary_link"] = 10;
            hash["secondary"] = 6;
            hash["secondary_link"] = 10;
            hash["tertiary"] = 8;
            hash["tertiary_link"] = 10;
            hash["unclassified"] = 10;
            hash["residential"] = 11;
            hash["service"] = 12;
            hash["living_street"] = 10;
            hash["track"] = 14;
            hash["road"] = 14;
            hash["path"] = 14;
            hash["driveway"] = 14;
            return hash;
        }();

        const std::string type = way.get_value_by_key("highway");
        motorway_class = [&]() {
            if (type == "motorway" || type == "trunk" || type == "motorway_link" ||
                type == "trunk_link")
                return true;
            else
                return false;
        }();

        link_class = [&]() {
            if (type == "motorway_link" || type == "trunk_link" || type == "primary_link" ||
                type == "secondary_link" || type == "tertiary_link")
                return true;
            else
                return false;
        }();

        may_be_ignored = [&]() {
            if (type == "service" || type == "track" || type == "road" || type == "path" ||
                type == "driveway")
                return true;
            else
                return false;
        }();

        if (class_hash.find(type) != class_hash.end())
            priority = class_hash.find(type)->second;
        else
            priority = 14;
    }
};

inline bool canBeSeenAsFork(const RoadClassification first, const RoadClassification second)
{
    return std::abs(static_cast<int>(first.priority) - static_cast<int>(second.priority)) <= 1;
}

// TODO augment this with all data required for guidance generation
struct RoadClassificationData
{
    // the road classification
    RoadClassification road_classification;

    // construction
    RoadClassificationData() : road_classification(RoadClassification()) {}
    RoadClassificationData(const osmium::Way &way) { road_classification.set(way); }
};

inline bool operator==(const RoadClassificationData lhs, const RoadClassificationData rhs)
{
    return lhs.road_classification == rhs.road_classification;
}

} // namespace guidance
} // namespace extractor
} // namespace osrm

#endif // OSRM_EXTRACTOR_CLASSIFICATION_DATA_HPP_
