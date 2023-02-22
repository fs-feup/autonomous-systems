#include <gtest/gtest.h>

#include "../pathplanner.h"

using testing::Eq;

TEST(PathPlanner, middlePath1) {
    Track* track = new Track();
    track->fillTrack("files\\map_test1.txt");

    PathPlanner* pathplanner = new PathPlanner(track);
    pathplanner->middlePath();

    vector<pair<float, float>> finalPath = pathplanner->getPath();

    vector<pair<float, float>> expected{
        make_pair(0, 0),
        make_pair(2, 0),
        make_pair(4, 0),
        make_pair(6, 0),
        make_pair(8, 0),
        };

    EXPECT_EQ(expected, finalPath);
}

TEST(PathPlanner, middlePath2) {
    Track* track = new Track();
    track->fillTrack("files\\map_test2.txt");

    PathPlanner* pathplanner = new PathPlanner(track);
    pathplanner->middlePath();

    vector<pair<float, float>> finalPath = pathplanner->getPath();

    vector<pair<float, float>> expected{
            make_pair(0.5, 0),
            make_pair(0.5, 1),
            make_pair(0.875, 1.75),
            make_pair(1.25, 2.25),
            make_pair(1.5, 2.5),
            make_pair(1.75, 2.25),
            make_pair(2.25, 1.75),
    };

    EXPECT_EQ(expected, finalPath);
}

TEST(PathPlanner, middlePath3) {
    Track* track = new Track();
    track->fillTrack("files\\map_test3.txt");

    PathPlanner* pathplanner = new PathPlanner(track);
    pathplanner->middlePath();

    vector<pair<float, float>> finalPath = pathplanner->getPath();

    vector<pair<float, float>> expected{
            make_pair(-0.5, 0),
            make_pair(-0.5, 1),
            make_pair(-0.875, 1.75),
            make_pair(-1.25, 2.25),
            make_pair(-2.25, 1.75),
    };

    EXPECT_EQ(expected, finalPath);
}

