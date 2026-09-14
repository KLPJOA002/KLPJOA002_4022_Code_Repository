#include "scan_state.h"

#include <limits>

#include <pcl/common/io.h>
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

PointCloudNT::Ptr ScanState::cloudWithNormals() const
{
    auto out = std::make_shared<PointCloudNT>();
    if (!loaded()) return out;
    pcl::copyPointCloud(*prepared, *out);

    const bool have_normals = normals && normals->size() == prepared->size();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (size_t i = 0; i < out->size(); ++i)
    {
        auto& p = (*out)[i];
        if (have_normals)
        {
            const auto& n = (*normals)[i];
            p.normal_x = n.normal_x; p.normal_y = n.normal_y; p.normal_z = n.normal_z;
            p.curvature = n.curvature;
        }
        else
        {
            p.normal_x = p.normal_y = p.normal_z = nan;
        }
    }
    return out;
}
