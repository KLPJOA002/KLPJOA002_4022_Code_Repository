#pragma once

#include <memory>
#include <string>
#include <vector>

#include <pcl/visualization/pcl_visualizer.h>
#include <vtkBoxWidget.h>
#include <vtkSmartPointer.h>

#include "fusion_pipeline.h"
#include "fusion_session.h"
#include "legend_panel.h"

// Viewer for the staged fusion pipeline. It renders whatever the session
// currently holds - every scan at its current transform, in its own color -
// alongside a legend panel listing the scans by name with their alignment
// error, and drives the review stages between batches.
//
// It owns no pipeline state: every redraw reads the FusionSession, so a
// stage that changes a transform only has to ask for a redraw rather than
// tell the viewer what changed.
//
// Selecting a scan (any of these):
//   click a legend row       - the list on the left
//   shift + click a point    - selects whichever scan owns that point
//   Up / Down                - move through the list
//
// Acting on the selected scan:
//   v  show / hide it               i  isolate it (hide all others)
//   a  show all                     d  exclude it / put it back
//   n  manual nudge mode (arrows move it, < > rotate, PgUp/PgDn for z)
//
// Re-running alignment:
//   Enter        re-run the current stage for the selected scan
//   Shift+Enter  re-run the current stage for every scan
//   z / Z        re-run coarse alignment for the selected scan / every scan
//   t            flip the rotation direction of the selected scan's rotation
//                group and re-align it - the interactive answer to "which way
//                did I turn the rig?" when the automatic test was unsure
//
// Stage control:
//   space  accept and continue      b  back to the previous review stage
//   m      show / hide this help    q  finish without further review
//
// During the ROI crop before each scan loads:
//   drag the box handles, space to confirm the crop, d to use the full cloud.
//
// NOTE ON KEY CHOICE: PCLVisualizer reserves p w s j c f e q g u o r l x h,
// the digits, and +/- for its own camera and rendering controls, and our
// callback runs *in addition* to those - it cannot override them. Every
// binding above deliberately avoids that set. In particular `e` exits the
// interactor outright, which is why exclude is bound to `d`.
class FusionVisualizer
{
public:
    explicit FusionVisualizer(const std::string& window_name = "Point Cloud Fusion");

    // Matches RoiProvider - pass this to FusionSession::loadAll().
    bool selectRoi(const ScanInput& input,
                    const PointCloudT::ConstPtr& raw_cloud,
                    Eigen::Vector4f& out_min,
                    Eigen::Vector4f& out_max);

    // Redraws every contributing scan at its current transform, plus the
    // legend. Cheap enough to call whenever the session changes.
    void showSession(const FusionSession& session, const std::string& status);

    // Keeps the window responsive during a long batch stage and reports
    // where the batch is up to. Matches ProgressFn.
    void showProgress(Stage stage, size_t done, size_t total, const std::string& label);

    // Blocks until the user chooses what to do next. Takes the session by
    // non-const reference because the review stage is where per-scan edits
    // happen - excluding a scan, nudging it, re-running just that one.
    ReviewAction review(FusionSession& session, Stage stage);

    // Shows the final cleaned result and waits for the window to close.
    void showFinal(const PointCloudNT::ConstPtr& final_cloud);

    // True once the user pressed q/ESC or closed the window. Callers should
    // stop drawing and let the run finish headless.
    bool stopped() const { return stopped_; }

private:
    // --- Input handling -------------------------------------------------
    void onKeyboard(const pcl::visualization::KeyboardEvent& event);
    void onMouse(const pcl::visualization::MouseEvent& event);
    void onPointPick(const pcl::visualization::PointPickingEvent& event);

    // --- Rendering ------------------------------------------------------
    void setStatus(const std::string& text);

    // Draws the always-on key reference. Which keys are listed depends on
    // what the viewer is currently doing, so the panel never shows a key
    // that would do nothing right now.
    void setHelp(const std::vector<std::string>& lines);
    std::vector<std::string> reviewHelpLines() const;
    void getWindowSize(int& width, int& height) const;
    void clearScanClouds();
    void applyVisibility(size_t index);
    void ensureVisibilitySize(size_t count);
    std::string reviewStatusLine(const FusionSession& session, Stage stage) const;

    // --- Review actions -------------------------------------------------
    void nudgeSelected(FusionSession& session, const Eigen::Vector3f& translation,
                        float rotation_about_z_deg);

    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    vtkSmartPointer<vtkBoxWidget> box_widget_;
    LegendPanel legend_;

    bool stopped_ = false;

    // Requests raised by the keyboard/mouse callbacks and consumed by the
    // blocking wait loops. Keeping them as flags (rather than acting inside
    // the callback) keeps all session mutation on the main thread, inside
    // review(), where it is easy to follow.
    bool continue_requested_ = false;
    bool back_requested_ = false;
    bool rerun_coarse_all_requested_ = false;
    bool rerun_refine_all_requested_ = false;
    bool rerun_coarse_one_requested_ = false;
    bool rerun_refine_one_requested_ = false;
    bool toggle_exclude_requested_ = false;
    bool rerun_stage_one_requested_ = false;
    bool rerun_stage_all_requested_ = false;
    bool flip_rotation_requested_ = false;
    bool roi_no_crop_requested_ = false;
    bool redraw_requested_ = false;

    // Pending manual nudge, accumulated by the keyboard callback.
    bool            nudge_pending_ = false;
    Eigen::Vector3f nudge_translation_ = Eigen::Vector3f::Zero();
    float           nudge_rotation_deg_ = 0.0f;

    bool help_visible_ = true;
    bool camera_initialised_ = false;
    int  selected_ = -1;        // scan index, or -1 for none
    bool nudge_mode_ = false;
    float nudge_step_m_ = 0.002f;   // 2 mm per arrow press
    float nudge_step_deg_ = 0.5f;

    std::vector<bool> visible_;
    std::vector<std::string> drawn_ids_;

    // Set by the point-picking callback; consumed by the wait loop, which
    // turns it back into a scan index.
    std::string picked_cloud_id_;
};
