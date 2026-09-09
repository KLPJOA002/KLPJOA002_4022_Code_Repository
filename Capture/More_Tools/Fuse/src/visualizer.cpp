#include "visualizer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkTransform.h>

using namespace std::chrono_literals;

namespace
{
constexpr const char* kPreviewId = "preview_cloud";
constexpr const char* kStatusId  = "status_text";
constexpr const char* kRoiId     = "roi_preview";

// Evenly-spaced hues around the color wheel so consecutive scans are easy
// to tell apart; overlapping regions from different scans then show up as
// interleaved colors rather than a uniform blob.
void hsvToRgb(double h, double s, double v, double& r, double& g, double& b)
{
    int i = static_cast<int>(h * 6.0);
    double f = h * 6.0 - i;
    double p = v * (1.0 - s);
    double q = v * (1.0 - f * s);
    double t = v * (1.0 - (1.0 - f) * s);
    switch (i % 6)
    {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break;
    }
}
} // namespace

FusionVisualizer::FusionVisualizer(const std::string& window_name)
{
    viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>(window_name);
    viewer_->setBackgroundColor(0.05, 0.05, 0.08);
    viewer_->addCoordinateSystem(0.05, "origin");
    viewer_->initCameraParameters();
    viewer_->setCameraPosition(0.3, -0.3, 0.3, 0, 0, 0, 0, 0, 1);

    viewer_->registerKeyboardCallback(
        [this](const pcl::visualization::KeyboardEvent& event) {
            if (!event.keyDown()) return;
            const std::string key = event.getKeySym();

            if (key == "space")
            {
                advance_requested_ = true;
            }
            else if (key == "c")
            {
                retry_requested_ = true;
            }
            else if (key == "s")
            {
                skip_requested_ = true;
            }
            else if (key == "u")
            {
                undo_requested_ = true;
            }
            else if (key == "x")
            {
                roi_no_crop_requested_ = true;
            }
            else if (key == "p")
            {
                auto_play_ = !auto_play_;
                std::cout << "[viewer] auto-play " << (auto_play_ ? "ON" : "OFF") << "\n";
            }
            else if (key == "a")
            {
                std::fill(visible_.begin(), visible_.end(), true);
                for (size_t i = 0; i < visible_.size(); ++i)
                    viewer_->setPointCloudRenderingProperties(
                        pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "scan_" + std::to_string(i));
            }
            else if (key == "r")
            {
                viewer_->resetCamera();
            }
            else if (key.size() == 1 && std::isdigit(static_cast<unsigned char>(key[0])))
            {
                int digit = key[0] - '0';
                for (size_t i = 0; i < visible_.size(); ++i)
                {
                    if (static_cast<int>(i % 10) == digit)
                    {
                        visible_[i] = !visible_[i];
                        std::string id = "scan_" + std::to_string(i);
                        viewer_->setPointCloudRenderingProperties(
                            pcl::visualization::PCL_VISUALIZER_OPACITY,
                            visible_[i] ? 1.0 : 0.0, id);
                    }
                }
            }
            else if (key == "q" || key == "Escape")
            {
                stopped_ = true;
                advance_requested_ = true;
            }
        });

    std::cout << "\n[viewer] controls: space=accept/confirm  c=retry  s=skip scan  u=undo previous\n"
                 "                   p=auto-play  1-0=toggle scan visibility  a=show all  r=reset camera\n"
                 "                   q/ESC=stop viewing (continues headless)\n"
                 "[viewer] before each scan: drag the crop box, space=confirm crop, x=use full cloud\n\n";
}

void FusionVisualizer::colorForIndex(int index, double& r, double& g, double& b)
{
    // Golden-angle hue stepping keeps consecutive indices visually distinct
    // even for many scans, unlike a linear hue ramp.
    double hue = std::fmod(index * 0.61803398875, 1.0);
    hsvToRgb(hue, 0.75, 0.95, r, g, b);
}

void FusionVisualizer::setStatus(const std::string& text)
{
    viewer_->removeText3D(kStatusId);
    viewer_->addText(text, 10, 10, 14, 1.0, 1.0, 1.0, kStatusId);
}

StepDecision FusionVisualizer::waitForDecision()
{
    advance_requested_ = false;
    retry_requested_ = false;
    skip_requested_ = false;
    undo_requested_ = false;

    while (!stopped_ && !viewer_->wasStopped())
    {
        viewer_->spinOnce(50);
        if (advance_requested_ || retry_requested_ || skip_requested_ || undo_requested_)
            break;
        if (auto_play_)
        {
            std::this_thread::sleep_for(400ms);
            break; // auto-play always accepts unless a key already fired above
        }
        std::this_thread::sleep_for(20ms);
    }

    if (viewer_->wasStopped()) stopped_ = true;
    if (stopped_) return StepDecision::Accept; // let the pipeline finish headless

    if (retry_requested_) return StepDecision::Retry;
    if (skip_requested_)  return StepDecision::Skip;
    if (undo_requested_)  return StepDecision::Undo;
    return StepDecision::Accept;
}

PointCloudT::Ptr FusionVisualizer::selectRoi(size_t scan_index,
                                              const ScanInput& scan_input,
                                              const PointCloudT::ConstPtr& raw_cloud)
{
    if (stopped_) return std::make_shared<PointCloudT>(*raw_cloud);

    viewer_->removePointCloud(kRoiId);
    viewer_->addPointCloud<PointT>(raw_cloud, kRoiId);
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, kRoiId);
    viewer_->resetCamera();

    PointT min_pt, max_pt;
    pcl::getMinMax3D(*raw_cloud, min_pt, max_pt);
    double bounds[6] = { min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z };

    box_widget_ = vtkSmartPointer<vtkBoxWidget>::New();
    box_widget_->SetInteractor(viewer_->getRenderWindow()->GetInteractor());
    box_widget_->SetPlaceFactor(1.0);
    box_widget_->PlaceWidget(bounds);
    box_widget_->RotationEnabledOff(); // axis-aligned only - simpler to crop against and enough for a rig scan
    box_widget_->On();

    setStatus("[" + std::to_string(scan_index) + "] " + scan_input.label() +
               "  -  drag the box to crop  -  space=confirm crop  x=use full cloud");

    roi_no_crop_requested_ = false;
    advance_requested_ = false;
    while (!stopped_ && !viewer_->wasStopped())
    {
        viewer_->spinOnce(50);
        if (advance_requested_ || roi_no_crop_requested_) break;
        std::this_thread::sleep_for(20ms);
    }
    if (viewer_->wasStopped()) stopped_ = true;

    bool no_crop = stopped_ || roi_no_crop_requested_;

    // Read the box's world-space extent before tearing it down.
    double corner_min[3] = { 1e30, 1e30, 1e30 };
    double corner_max[3] = { -1e30, -1e30, -1e30 };
    if (!no_crop)
    {
        vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
        box_widget_->GetTransform(transform);
        for (int cx = -1; cx <= 1; cx += 2)
            for (int cy = -1; cy <= 1; cy += 2)
                for (int cz = -1; cz <= 1; cz += 2)
                {
                    double local[3] = { cx * 0.5, cy * 0.5, cz * 0.5 };
                    double world[3];
                    transform->TransformPoint(local, world);
                    for (int k = 0; k < 3; ++k)
                    {
                        corner_min[k] = std::min(corner_min[k], world[k]);
                        corner_max[k] = std::max(corner_max[k], world[k]);
                    }
                }
    }

    box_widget_->Off();
    box_widget_ = nullptr;
    viewer_->removePointCloud(kRoiId);

    if (no_crop)
        return std::make_shared<PointCloudT>(*raw_cloud);

    pcl::CropBox<PointT> crop;
    crop.setInputCloud(raw_cloud);
    crop.setMin(Eigen::Vector4f(static_cast<float>(corner_min[0]), static_cast<float>(corner_min[1]),
                                 static_cast<float>(corner_min[2]), 1.0f));
    crop.setMax(Eigen::Vector4f(static_cast<float>(corner_max[0]), static_cast<float>(corner_max[1]),
                                 static_cast<float>(corner_max[2]), 1.0f));
    auto cropped = std::make_shared<PointCloudT>();
    crop.filter(*cropped);

    if (cropped->empty())
    {
        std::cerr << "[viewer] ROI crop produced zero points for " << scan_input.label()
                   << " - using full cloud instead\n";
        return std::make_shared<PointCloudT>(*raw_cloud);
    }

    std::cout << "[viewer] cropped " << scan_input.label() << " to " << cropped->size()
               << " / " << raw_cloud->size() << " points\n";
    return cropped;
}

StepDecision FusionVisualizer::onStep(size_t scan_index,
                                       const ScanInput& scan_input,
                                       const std::string& stage,
                                       const PointCloudT::ConstPtr& scan_cloud_in_common_frame,
                                       const PointCloudT::ConstPtr& /*merged_so_far*/,
                                       const RegistrationResult& result)
{
    if (stopped_) return StepDecision::Accept;

    // Discard whatever was committed for the previous scan so its color
    // disappears from the view - it's about to be redone.
    auto undoPrevious = [this, scan_index]() {
        if (scan_index == 0) return;
        std::string prev_id = "scan_" + std::to_string(scan_index - 1);
        viewer_->removePointCloud(prev_id);
        if (scan_index - 1 < visible_.size()) visible_[scan_index - 1] = false;
    };

    if (stage == "reference")
    {
        double r, g, b;
        colorForIndex(static_cast<int>(scan_index), r, g, b);
        std::string id = "scan_" + std::to_string(scan_index);
        viewer_->removePointCloud(id);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> color(
            scan_cloud_in_common_frame, r * 255, g * 255, b * 255);
        viewer_->addPointCloud(scan_cloud_in_common_frame, color, id);
        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id);
        viewer_->resetCamera();

        setStatus("[0] " + scan_input.label() + "  (reference frame)  -  space to continue, c to redo ROI");
        StepDecision decision = waitForDecision();
        if (decision == StepDecision::Retry)
        {
            viewer_->removePointCloud(id);
            return decision;
        }
        if (visible_.size() <= scan_index) visible_.resize(scan_index + 1, false);
        visible_[scan_index] = true;
        return StepDecision::Accept;
    }

    if (stage == "coarse")
    {
        // Transient gray preview at the coarse-aligned pose, so you can see
        // how far ICP still has to pull it before committing a color.
        viewer_->removePointCloud(kPreviewId);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> color(
            scan_cloud_in_common_frame, 160, 160, 160);
        viewer_->addPointCloud(scan_cloud_in_common_frame, color, kPreviewId);
        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, kPreviewId);
        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, kPreviewId);

        setStatus("[" + std::to_string(scan_index) + "] " + scan_input.label() +
                   "  -  coarse alignment (gray)  -  space to refine, c to retry, s to skip, u to undo previous");
        StepDecision decision = waitForDecision();
        if (decision == StepDecision::Undo) undoPrevious();
        return decision;
    }

    if (stage == "coarse_failed")
    {
        viewer_->removePointCloud(kPreviewId);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> color(
            scan_cloud_in_common_frame, 220, 40, 40);
        viewer_->addPointCloud(scan_cloud_in_common_frame, color, kPreviewId);
        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, 0.6, kPreviewId);

        setStatus("[" + std::to_string(scan_index) + "] " + scan_input.label() +
                   "  -  COARSE ALIGNMENT FAILED (red)  -  c to retry, s to skip, u to undo previous");
        StepDecision decision = waitForDecision();
        if (decision == StepDecision::Accept) decision = StepDecision::Skip; // failed alignment can't be "accepted"
        if (decision == StepDecision::Undo) undoPrevious();
        viewer_->removePointCloud(kPreviewId);
        return decision;
    }

    if (stage == "refined" || stage == "flagged")
    {
        viewer_->removePointCloud(kPreviewId);

        double r, g, b;
        colorForIndex(static_cast<int>(scan_index), r, g, b);
        if (stage == "flagged")
        {
            // Tint toward red so a quality-flagged merge stands out even
            // once it's committed in its normal per-scan color.
            r = 0.5 * r + 0.5; g *= 0.5; b *= 0.5;
        }

        std::string id = "scan_" + std::to_string(scan_index);
        viewer_->removePointCloud(id);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> color(
            scan_cloud_in_common_frame, r * 255, g * 255, b * 255);
        viewer_->addPointCloud(scan_cloud_in_common_frame, color, id);
        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id);

        std::string flag = stage == "flagged" ? "  [FLAGGED: high ICP fitness error]" : "";
        setStatus("[" + std::to_string(scan_index) + "] " + scan_input.label() +
                   "  -  fitness=" + std::to_string(result.fitness_score) +
                   flag + "  -  space to accept, c to retry, s to skip, u to undo previous");
        StepDecision decision = waitForDecision();

        if (decision == StepDecision::Undo) undoPrevious();

        if (decision != StepDecision::Accept)
        {
            viewer_->removePointCloud(id);
            return decision;
        }

        if (visible_.size() <= scan_index) visible_.resize(scan_index + 1, false);
        visible_[scan_index] = true;
        return StepDecision::Accept;
    }

    return StepDecision::Accept;
}

void FusionVisualizer::showFinal(const PointCloudNT::ConstPtr& final_cloud)
{
    if (stopped_) return;

    for (size_t i = 0; i < visible_.size(); ++i)
        viewer_->removePointCloud("scan_" + std::to_string(i));
    viewer_->removePointCloud(kPreviewId);

    pcl::visualization::PointCloudColorHandlerGenericField<PointNT> color(final_cloud, "z");
    viewer_->addPointCloud<PointNT>(final_cloud, color, "final_cloud");
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "final_cloud");
    viewer_->resetCamera();

    setStatus("Final fused cloud (" + std::to_string(final_cloud->size()) +
               " points, colored by height)  -  q/ESC to close");

    std::cout << "[viewer] showing final result - press q or ESC to close\n";
    while (!viewer_->wasStopped())
    {
        viewer_->spinOnce(100);
        std::this_thread::sleep_for(50ms);
    }
}
