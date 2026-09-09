#pragma once

#include <memory>
#include <string>

#include <pcl/visualization/pcl_visualizer.h>
#include <vtkBoxWidget.h>
#include <vtkSmartPointer.h>

#include "fusion_pipeline.h"

// Step-through viewer: shows each scan being coarsely aligned (translucent
// preview) and then refined (solid, committed color) into the common frame,
// so overlapping regions from different scans become visually apparent as
// matching colors coinciding in space.
//
// Controls (printed to the console on startup too):
//   space       - advance / accept and continue
//   c           - retry (redo this scan's ROI crop + alignment from scratch)
//   s           - skip this scan (leave it out of the fused cloud)
//   u           - undo: discard the previous scan and go back to redo it
//   p           - toggle auto-play (advance automatically with a short delay)
//   1-9, 0      - toggle visibility of scans 1-10 (repeat digits beyond 10 cycle)
//   a           - show all scans / r - reset camera
//   q or ESC    - stop visualizing and finish the run headless
//
// Before each scan is aligned, an interactive box widget lets you crop it
// down to the region that matters (drag the handles, space to confirm the
// crop, x to use the full cloud uncropped).
class FusionVisualizer
{
public:
    explicit FusionVisualizer(const std::string& window_name = "Point Cloud Fusion");

    // Matches FusionStepCallback's signature - bind this as the on_step
    // callback passed to fuseScans().
    StepDecision onStep(size_t scan_index,
                         const ScanInput& scan_input,
                         const std::string& stage,
                         const PointCloudT::ConstPtr& scan_cloud_in_common_frame,
                         const PointCloudT::ConstPtr& merged_so_far,
                         const RegistrationResult& result);

    // Matches RoiSelectionCallback's signature - bind this as the
    // select_roi callback passed to fuseScans().
    PointCloudT::Ptr selectRoi(size_t scan_index,
                                const ScanInput& scan_input,
                                const PointCloudT::ConstPtr& raw_cloud);

    // Call once after fuseScans() returns, to show the final cleaned
    // result and let the user inspect it before closing.
    void showFinal(const PointCloudNT::ConstPtr& final_cloud);

    // True once the user has pressed q/ESC or closed the window - callers
    // should stop passing further steps (or just skip building previews)
    // once this is set, since the window is gone.
    bool stopped() const { return stopped_; }

private:
    // Spins the viewer until the user makes a decision (or closes the
    // window, which always resolves to Accept so the pipeline finishes
    // headless rather than getting stuck retrying forever).
    StepDecision waitForDecision();
    void setStatus(const std::string& text);
    static void colorForIndex(int index, double& r, double& g, double& b);

    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    vtkSmartPointer<vtkBoxWidget> box_widget_;
    bool stopped_ = false;
    bool auto_play_ = false;
    bool advance_requested_ = false;
    bool retry_requested_ = false;
    bool skip_requested_ = false;
    bool undo_requested_ = false;
    bool roi_no_crop_requested_ = false;
    std::vector<bool> visible_;
};
