#include "tracker.hpp"

cv::Ptr<Tracker> createFaceTracker();
// TODO: Declare your implementation here
// cv::Ptr<Tracker> createTrackerYourName();

cv::Ptr<Tracker> createTracker(const std::string &impl_name)
{
    if (impl_name == "FaceTracker")
        return createFaceTracker();
    // TODO: Add case for your implementation
    // else if (impl_name == "your_name"):
    //     return createTrackerYourName();

    return 0;
}
