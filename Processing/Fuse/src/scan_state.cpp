#include "scan_state.h"

#include <pcl/common/transforms.h>

const char* toString(ScanStatus status)
{
    switch (status)
    {
        case ScanStatus::Pending:       return "pending";
        case ScanStatus::Reference:     return "reference";
        case ScanStatus::CoarseAligned: return "coarse";
        case ScanStatus::CoarseFailed:  return "coarse-failed";
        case ScanStatus::Refined:       return "refined";
        case ScanStatus::RefineFailed:  return "refine-failed";
        case ScanStatus::Excluded:      return "excluded";
    }
    return "?";
}

bool contributesToFusion(ScanStatus status)
{
    // Coarse-only scans still contribute: they are positioned well enough
    // to be worth showing and fusing, they just haven't been refined yet.
    return status == ScanStatus::Reference ||
           status == ScanStatus::CoarseAligned ||
           status == ScanStatus::Refined;
}

PointCloudT::Ptr ScanState::cloudInCommonFrame() const
{
    auto out = std::make_shared<PointCloudT>();
    if (!loaded()) return out;
    pcl::transformPointCloud(*prepared, *out, transform);
    return out;
}
