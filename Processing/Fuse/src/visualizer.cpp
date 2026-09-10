#include "visualizer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>
#include <thread>

#include <Eigen/Geometry>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkTransform.h>

#include "scan_colors.h"

using namespace std::chrono_literals;

namespace
{
constexpr const char* kStatusId = "status_text";
constexpr const char* kHelpId   = "help_text";
constexpr const char* kRoiId    = "roi_preview";
constexpr const char* kScanIdPrefix = "scan_";

std::string scanCloudId(size_t index)
{
    return kScanIdPrefix + std::to_string(index);
}

// Inverse of scanCloudId - the point-picking callback hands back a cloud
// name, and this turns it back into the scan it belongs to.
int scanIndexFromCloudId(const std::string& id)
{
    const std::string prefix = kScanIdPrefix;
    if (id.rfind(prefix, 0) != 0) return -1;
    try
    {
        return std::stoi(id.substr(prefix.size()));
    }
    catch (const std::exception&)
    {
        return -1;
    }
}
} // namespace

// -------------------------------------------------------------------------
FusionVisualizer::FusionVisualizer(const std::string& window_name)
{
    viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>(window_name);
    viewer_->setBackgroundColor(0.05, 0.05, 0.08);
    viewer_->addCoordinateSystem(0.05, "origin");
    viewer_->initCameraParameters();
    viewer_->setCameraPosition(0.3, -0.3, 0.3, 0, 0, 0, 0, 0, 1);

    legend_.attach(viewer_);

    viewer_->registerKeyboardCallback(
        [this](const pcl::visualization::KeyboardEvent& e) { onKeyboard(e); });
    viewer_->registerMouseCallback(
        [this](const pcl::visualization::MouseEvent& e) { onMouse(e); });
    viewer_->registerPointPickingCallback(
        [this](const pcl::visualization::PointPickingEvent& e) { onPointPick(e); });

    std::cout <<
        "\n[viewer] select a scan: click its row in the list, shift+click a point in the\n"
        "         view, or use Up/Down.\n"
        "[viewer] selected scan: v=show/hide  i=isolate  a=show all  e=exclude/include\n"
        "                        c=re-run coarse  f=re-run fine  n=nudge mode\n"
        "[viewer] all scans:     C=re-run coarse (all)  F=re-run fine (all)\n"
        "[viewer] stage:         space=continue  b=back  r=reset camera  q/ESC=finish\n"
        "[viewer] ROI crop:      drag the box, space=confirm, x=use full cloud\n\n";
}

// -------------------------------------------------------------------------
void FusionVisualizer::onKeyboard(const pcl::visualization::KeyboardEvent& event)
{
    if (!event.keyDown()) return;
    const std::string key = event.getKeySym();

    // --- Nudge mode owns the arrow keys while it is active -------------
    if (nudge_mode_ && selected_ >= 0)
    {
        const float d = nudge_step_m_;
        if (key == "Left")  { nudge_translation_ += Eigen::Vector3f(-d, 0, 0); nudge_pending_ = true; return; }
        if (key == "Right") { nudge_translation_ += Eigen::Vector3f( d, 0, 0); nudge_pending_ = true; return; }
        if (key == "Up")    { nudge_translation_ += Eigen::Vector3f(0,  d, 0); nudge_pending_ = true; return; }
        if (key == "Down")  { nudge_translation_ += Eigen::Vector3f(0, -d, 0); nudge_pending_ = true; return; }
        if (key == "Prior") { nudge_translation_ += Eigen::Vector3f(0, 0,  d); nudge_pending_ = true; return; } // PageUp
        if (key == "Next")  { nudge_translation_ += Eigen::Vector3f(0, 0, -d); nudge_pending_ = true; return; } // PageDown
        if (key == "comma")  { nudge_rotation_deg_ -= nudge_step_deg_; nudge_pending_ = true; return; }
        if (key == "period") { nudge_rotation_deg_ += nudge_step_deg_; nudge_pending_ = true; return; }
        if (key == "plus" || key == "equal")
        {
            nudge_step_m_ *= 2.0f; nudge_step_deg_ *= 2.0f;
            std::cout << "[viewer] nudge step: " << nudge_step_m_ * 1000.0f << " mm / "
                      << nudge_step_deg_ << " deg\n";
            redraw_requested_ = true; return;
        }
        if (key == "minus")
        {
            nudge_step_m_ *= 0.5f; nudge_step_deg_ *= 0.5f;
            std::cout << "[viewer] nudge step: " << nudge_step_m_ * 1000.0f << " mm / "
                      << nudge_step_deg_ << " deg\n";
            redraw_requested_ = true; return;
        }
    }

    // --- Selection ------------------------------------------------------
    if (key == "Up" || key == "Down")
    {
        const int count = static_cast<int>(visible_.size());
        if (count > 0)
        {
            const int step = (key == "Up") ? -1 : 1;
            selected_ = (selected_ < 0) ? 0 : (selected_ + step + count) % count;
            redraw_requested_ = true;
        }
        return;
    }

    // --- Per-scan actions ----------------------------------------------
    if (key == "v")
    {
        if (selected_ >= 0 && static_cast<size_t>(selected_) < visible_.size())
        {
            visible_[selected_] = !visible_[selected_];
            applyVisibility(static_cast<size_t>(selected_));
            redraw_requested_ = true;
        }
        return;
    }
    if (key == "i")
    {
        if (selected_ >= 0)
        {
            // Isolate, or un-isolate if the selection is already the only
            // thing showing - one key that toggles rather than two.
            const bool already_isolated =
                std::count(visible_.begin(), visible_.end(), true) == 1 &&
                static_cast<size_t>(selected_) < visible_.size() && visible_[selected_];

            for (size_t i = 0; i < visible_.size(); ++i)
            {
                visible_[i] = already_isolated || (static_cast<int>(i) == selected_);
                applyVisibility(i);
            }
            redraw_requested_ = true;
        }
        return;
    }
    if (key == "a")
    {
        std::fill(visible_.begin(), visible_.end(), true);
        for (size_t i = 0; i < visible_.size(); ++i) applyVisibility(i);
        redraw_requested_ = true;
        return;
    }
    // `d` rather than the obvious `e`: PCLVisualizer binds e/E to "exit the
    // interactor", and our callback runs alongside its built-ins rather than
    // replacing them, so `e` would close the window as well as excluding.
    if (key == "d")
    {
        // Serves both phases: "drop this scan" during review, "drop the crop"
        // during ROI selection. The two wait loops are disjoint and each
        // clears both flags on entry, so neither can eat the other's press.
        toggle_exclude_requested_ = true;
        roi_no_crop_requested_ = true;
        return;
    }
    if (key == "n")
    {
        nudge_mode_ = !nudge_mode_;
        std::cout << "[viewer] nudge mode " << (nudge_mode_ ? "ON" : "OFF") << "\n";
        redraw_requested_ = true;
        return;
    }
    if (key == "m")
    {
        help_visible_ = !help_visible_;
        redraw_requested_ = true;
        return;
    }

    // --- Re-running alignment -------------------------------------------
    // Return re-runs whatever the current stage is about, so there is one
    // key to learn instead of a coarse/fine pair; z reaches back to coarse
    // from the refine review, where a scan ICP cannot rescue needs a fresh
    // coarse pose rather than another refinement.
    if (key == "Return" || key == "KP_Enter")
    {
        if (event.isShiftPressed()) rerun_stage_all_requested_ = true;
        else                        rerun_stage_one_requested_ = true;
        return;
    }
    if (key == "t") { flip_rotation_requested_ = true; return; }
    if (key == "z") { rerun_coarse_one_requested_ = true; return; }
    if (key == "Z") { rerun_coarse_all_requested_ = true; return; }

    // --- Stage control ---------------------------------------------------
    if (key == "space") { continue_requested_ = true; return; }
    if (key == "b") { back_requested_ = true; return; }
    if (key == "q" || key == "Escape")
    {
        stopped_ = true;
        continue_requested_ = true;
        return;
    }
}

// -------------------------------------------------------------------------
void FusionVisualizer::onMouse(const pcl::visualization::MouseEvent& event)
{
    if (event.getType() != pcl::visualization::MouseEvent::MouseButtonPress ||
        event.getButton() != pcl::visualization::MouseEvent::LeftButton)
        return;

    // Both the legend's text placement and VTK's mouse coordinates use the
    // window's bottom-left as origin, so these compare directly.
    const int hit = legend_.hitTest(static_cast<int>(event.getX()),
                                     static_cast<int>(event.getY()));
    if (hit >= 0)
    {
        selected_ = hit;
        redraw_requested_ = true;
    }
}

// -------------------------------------------------------------------------
void FusionVisualizer::onPointPick(const pcl::visualization::PointPickingEvent& event)
{
    // PCL reports which cloud the picked point belongs to, and each scan is
    // its own cloud - so a shift+click in the viewport identifies the scan
    // directly, with no spatial search needed.
    picked_cloud_id_ = event.getCloudName();
}

// -------------------------------------------------------------------------
void FusionVisualizer::getWindowSize(int& width, int& height) const
{
    width = 800;
    height = 600;
    if (auto window = viewer_->getRenderWindow())
    {
        const int* size = window->GetSize();
        if (size && size[0] > 0) width = size[0];
        if (size && size[1] > 0) height = size[1];
    }
}

void FusionVisualizer::setHelp(const std::vector<std::string>& lines)
{
    viewer_->removeShape(kHelpId);
    if (!help_visible_ || lines.empty()) return;

    int width = 800, height = 600;
    getWindowSize(width, height);

    // Top-right, laid out from the top down. vtkTextActor renders embedded
    // newlines, so the whole block is one actor rather than one per line.
    std::string block;
    for (size_t i = 0; i < lines.size(); ++i)
    {
        if (i) block += "\n";
        block += lines[i];
    }

    constexpr int kFontSize = 11;
    constexpr int kLineHeight = 14;
    const int block_height = static_cast<int>(lines.size()) * kLineHeight;
    const int x = std::max(10, width - 300);
    const int y = std::max(10, height - 30 - block_height);

    viewer_->addText(block, x, y, kFontSize, 0.7, 0.75, 0.85, kHelpId);
}

std::vector<std::string> FusionVisualizer::reviewHelpLines() const
{
    if (nudge_mode_)
    {
        return {
            "NUDGE MODE  (n to leave)",
            "  arrows      move in x / y",
            "  PgUp PgDn   move in z",
            "  , .         rotate about z",
            "  + -         change step size",
        };
    }

    return {
        "SELECT  click a row, shift+click a",
        "        point, or Up / Down",
        "VIEW    v hide   i isolate   a all",
        "EDIT    d exclude    n nudge",
        "REDO    Enter this scan",
        "        Shift+Enter all scans",
        "        z coarse this   Z coarse all",
        "ROTATE  t flip this scan's rotation",
        "          direction (and later ones)",
        "STAGE   space continue   b back",
        "        m hide help      q finish",
    };
}

// -------------------------------------------------------------------------
void FusionVisualizer::setStatus(const std::string& text)
{
    viewer_->removeShape(kStatusId);
    viewer_->addText(text, 10, 26, 14, 1.0, 1.0, 1.0, kStatusId);
}

void FusionVisualizer::clearScanClouds()
{
    for (const auto& id : drawn_ids_)
        viewer_->removePointCloud(id);
    drawn_ids_.clear();
}

void FusionVisualizer::ensureVisibilitySize(size_t count)
{
    if (visible_.size() < count) visible_.resize(count, true);
}

void FusionVisualizer::applyVisibility(size_t index)
{
    const std::string id = scanCloudId(index);
    if (std::find(drawn_ids_.begin(), drawn_ids_.end(), id) == drawn_ids_.end())
        return; // not currently drawn (excluded, or not yet placed)

    const bool show = index < visible_.size() ? visible_[index] : true;
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, show ? 1.0 : 0.0, id);
}

// -------------------------------------------------------------------------
void FusionVisualizer::showSession(const FusionSession& session, const std::string& status)
{
    if (stopped_) return;

    ensureVisibilitySize(session.size());
    clearScanClouds();

    for (size_t i = 0; i < session.size(); ++i)
    {
        const auto& scan = session.scan(i);
        if (!scan.loaded() || !contributesToFusion(scan.status)) continue;

        double r, g, b;
        ScanColors::forIndex(static_cast<int>(i), r, g, b);
        if (scan.flagged) ScanColors::tintFlagged(r, g, b);

        auto cloud = scan.cloudInCommonFrame();
        const std::string id = scanCloudId(i);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> color(
            cloud, r * 255, g * 255, b * 255);
        viewer_->addPointCloud<PointT>(cloud, color, id);
        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            static_cast<int>(i) == selected_ ? 4 : 2, id); // selection reads as bolder points
        drawn_ids_.push_back(id);
        applyVisibility(i);
    }

    // Fit the view to the data once, the first time there is anything to
    // look at. Only the ROI step used to do this, so skipping ROI left the
    // camera pointing at empty space.
    if (!camera_initialised_ && !drawn_ids_.empty())
    {
        viewer_->resetCamera();
        camera_initialised_ = true;
    }

    legend_.draw(session, selected_, visible_);
    setHelp(reviewHelpLines());
    setStatus(status);
    viewer_->spinOnce(1);
}

// -------------------------------------------------------------------------
void FusionVisualizer::showProgress(Stage stage, size_t done, size_t total,
                                     const std::string& label)
{
    if (stopped_) return;
    setStatus(std::string(toString(stage)) + ": " + std::to_string(done + 1) + "/" +
               std::to_string(total) + "  " + label);
    // A single spin keeps the window responsive during a long batch without
    // meaningfully slowing the batch down.
    viewer_->spinOnce(1);
    if (viewer_->wasStopped()) stopped_ = true;
}

// -------------------------------------------------------------------------
std::string FusionVisualizer::reviewStatusLine(const FusionSession& session, Stage stage) const
{
    // Deliberately short: the help panel carries the key bindings, so this
    // line only has to say where we are and what is selected, and must fit
    // the window width without running off the edge.
    std::string line = std::string(toString(stage)) + "  |  " +
        std::to_string(session.countContributing()) + "/" + std::to_string(session.size()) +
        " placed";

    if (selected_ >= 0 && static_cast<size_t>(selected_) < session.size())
    {
        const auto& scan = session.scan(static_cast<size_t>(selected_));
        line += "  |  [" + std::to_string(selected_) + "] " +
                 scan.input.shortLabel() + " (" + toString(scan.status) + ")";
    }
    else
    {
        line += "  |  nothing selected";
    }

    if (nudge_mode_)
    {
        std::ostringstream nudge;
        nudge << "  |  NUDGE step " << nudge_step_m_ * 1000.0f << "mm";
        line += nudge.str();
    }
    return line;
}

// -------------------------------------------------------------------------
void FusionVisualizer::nudgeSelected(FusionSession& session,
                                      const Eigen::Vector3f& translation,
                                      float rotation_about_z_deg)
{
    if (selected_ < 0) return;

    // Rotate about the selected scan's own centroid, not the world origin -
    // rotating a scan about a distant origin swings it away instead of
    // turning it in place, which is never what a manual nudge means.
    auto cloud = session.scan(static_cast<size_t>(selected_)).cloudInCommonFrame();
    Eigen::Vector4f centroid = Eigen::Vector4f::Zero();
    if (!cloud->empty())
        pcl::compute3DCentroid(*cloud, centroid);

    const float rad = rotation_about_z_deg * static_cast<float>(M_PI) / 180.0f;
    Eigen::Matrix3f R(Eigen::AngleAxisf(rad, Eigen::Vector3f::UnitZ()));

    Eigen::Matrix4f delta = Eigen::Matrix4f::Identity();
    delta.block<3, 3>(0, 0) = R;
    // t = c - R*c + translation  puts the rotation's fixed point at the centroid.
    delta.block<3, 1>(0, 3) = centroid.head<3>() - R * centroid.head<3>() + translation;

    session.nudgeScan(static_cast<size_t>(selected_), delta);
}

// -------------------------------------------------------------------------
ReviewAction FusionVisualizer::review(FusionSession& session, Stage stage)
{
    if (stopped_) return ReviewAction::Continue;

    ensureVisibilitySize(session.size());

    // Land the cursor on the first scan that wants attention, so the common
    // case (something went wrong, go look at it) needs no navigation.
    if (selected_ < 0)
    {
        selected_ = 0;
        for (size_t i = 0; i < session.size(); ++i)
        {
            const auto& scan = session.scan(i);
            if (scan.flagged || scan.status == ScanStatus::CoarseFailed ||
                scan.status == ScanStatus::RefineFailed)
            {
                selected_ = static_cast<int>(i);
                break;
            }
        }
    }

    showSession(session, reviewStatusLine(session, stage));

    continue_requested_ = false;
    back_requested_ = false;
    rerun_coarse_all_requested_ = false;
    rerun_refine_all_requested_ = false;
    rerun_coarse_one_requested_ = false;
    rerun_refine_one_requested_ = false;
    toggle_exclude_requested_ = false;
    roi_no_crop_requested_ = false;
    rerun_stage_one_requested_ = false;
    rerun_stage_all_requested_ = false;
    flip_rotation_requested_ = false;
    redraw_requested_ = false;
    nudge_pending_ = false;
    picked_cloud_id_.clear();

    while (!stopped_ && !viewer_->wasStopped())
    {
        viewer_->spinOnce(50);

        if (continue_requested_ || back_requested_ ||
            rerun_coarse_all_requested_ || rerun_refine_all_requested_ ||
            rerun_stage_all_requested_)
        {
            // Resolve a Shift+Enter into the concrete batch action before
            // leaving the loop, so the return value below sees it.
            if (rerun_stage_all_requested_)
            {
                rerun_stage_all_requested_ = false;
                if (stage == Stage::CoarseReview) rerun_coarse_all_requested_ = true;
                else                              rerun_refine_all_requested_ = true;
            }
            break;
        }

        // --- Selection from a viewport pick ------------------------------
        if (!picked_cloud_id_.empty())
        {
            const int picked = scanIndexFromCloudId(picked_cloud_id_);
            picked_cloud_id_.clear();
            if (picked >= 0 && static_cast<size_t>(picked) < session.size())
            {
                selected_ = picked;
                std::cout << "[viewer] selected [" << picked << "] "
                          << session.scan(static_cast<size_t>(picked)).label() << "\n";
                redraw_requested_ = true;
            }
        }

        // --- Session-mutating actions, all on the main thread -------------
        if (nudge_pending_)
        {
            nudgeSelected(session, nudge_translation_, nudge_rotation_deg_);
            nudge_translation_.setZero();
            nudge_rotation_deg_ = 0.0f;
            nudge_pending_ = false;
            redraw_requested_ = true;
        }

        if (toggle_exclude_requested_)
        {
            toggle_exclude_requested_ = false;
            if (selected_ >= 0 && static_cast<size_t>(selected_) < session.size())
            {
                const auto index = static_cast<size_t>(selected_);
                const bool excluding = session.scan(index).status != ScanStatus::Excluded;
                session.setExcluded(index, excluding);
                std::cout << "[viewer] " << (excluding ? "excluded " : "re-included ")
                          << session.scan(index).label() << "\n";
                redraw_requested_ = true;
            }
        }

        // Enter means "do this stage again", so resolve it against the
        // stage we are reviewing before the handlers below run.
        if (rerun_stage_one_requested_)
        {
            rerun_stage_one_requested_ = false;
            if (stage == Stage::CoarseReview) rerun_coarse_one_requested_ = true;
            else                              rerun_refine_one_requested_ = true;
        }
        if (rerun_stage_all_requested_)
        {
            rerun_stage_all_requested_ = false;
            if (stage == Stage::CoarseReview) rerun_coarse_all_requested_ = true;
            else                              rerun_refine_all_requested_ = true;
        }

        if (flip_rotation_requested_)
        {
            flip_rotation_requested_ = false;
            if (selected_ >= 0 && static_cast<size_t>(selected_) < session.size())
            {
                const int group = session.rotationGroupOf(static_cast<size_t>(selected_));
                if (group < 0)
                {
                    std::cout << "[viewer] " << session.scan(static_cast<size_t>(selected_)).label()
                              << " has no rotation metadata - nothing to flip\n";
                }
                else
                {
                    setStatus("flipping rotation direction and re-aligning ...");
                    viewer_->spinOnce(1);

                    session.flipRotationGroup(static_cast<size_t>(group));

                    // Re-place every scan the flip moved. Flipping step k
                    // shifts group k and all later groups, so this walks
                    // from k to the end rather than just the one group.
                    for (size_t g = static_cast<size_t>(group);
                          g < session.rotationGroups().size(); ++g)
                    {
                        for (size_t index : session.rotationGroups()[g].scans)
                        {
                            if (index == session.referenceIndex()) continue;
                            if (session.scan(index).status == ScanStatus::Excluded) continue;
                            session.coarseAlignScan(index);
                        }
                    }
                }
                redraw_requested_ = true;
            }
        }

        if (rerun_coarse_one_requested_ || rerun_refine_one_requested_)
        {
            const bool coarse = rerun_coarse_one_requested_;
            rerun_coarse_one_requested_ = false;
            rerun_refine_one_requested_ = false;

            if (selected_ >= 0 && static_cast<size_t>(selected_) < session.size())
            {
                const auto index = static_cast<size_t>(selected_);
                setStatus(std::string(coarse ? "re-running coarse alignment for "
                                              : "re-running fine registration for ") +
                           session.scan(index).label() + " ...");
                viewer_->spinOnce(1);

                if (coarse) session.coarseAlignScan(index);
                else        session.refineScan(index);
                redraw_requested_ = true;
            }
        }

        if (redraw_requested_)
        {
            redraw_requested_ = false;
            showSession(session, reviewStatusLine(session, stage));
        }

        std::this_thread::sleep_for(20ms);
    }

    if (viewer_->wasStopped()) stopped_ = true;
    if (stopped_) return ReviewAction::Quit;
    if (rerun_coarse_all_requested_) return ReviewAction::RerunCoarse;
    if (rerun_refine_all_requested_) return ReviewAction::RerunRefine;
    if (back_requested_)             return ReviewAction::Back;
    return ReviewAction::Continue;
}

// -------------------------------------------------------------------------
bool FusionVisualizer::selectRoi(const ScanInput& input,
                                  const PointCloudT::ConstPtr& raw_cloud,
                                  Eigen::Vector4f& out_min,
                                  Eigen::Vector4f& out_max)
{
    if (stopped_) return false;

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

    setHelp({ "CROP",
               "  drag the box handles to",
               "  enclose the region to keep",
               "",
               "  space   confirm the crop",
               "  d       use the full cloud" });
    setStatus(input.label() + "  -  drag the box to crop  -  space=confirm  d=use full cloud");

    roi_no_crop_requested_ = false;
    toggle_exclude_requested_ = false;
    continue_requested_ = false;
    while (!stopped_ && !viewer_->wasStopped())
    {
        viewer_->spinOnce(50);
        if (continue_requested_ || roi_no_crop_requested_) break;
        std::this_thread::sleep_for(20ms);
    }
    if (viewer_->wasStopped()) stopped_ = true;

    const bool no_crop = stopped_ || roi_no_crop_requested_;

    // Read the box's world-space extent before tearing it down. The widget
    // reports a transform on a unit cube centred at the origin, so the crop
    // bounds are the transformed corners' axis-aligned envelope.
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

    if (no_crop) return false;

    out_min = Eigen::Vector4f(static_cast<float>(corner_min[0]), static_cast<float>(corner_min[1]),
                               static_cast<float>(corner_min[2]), 1.0f);
    out_max = Eigen::Vector4f(static_cast<float>(corner_max[0]), static_cast<float>(corner_max[1]),
                               static_cast<float>(corner_max[2]), 1.0f);
    return true;
}

// -------------------------------------------------------------------------
void FusionVisualizer::showFinal(const PointCloudNT::ConstPtr& final_cloud)
{
    if (stopped_) return;

    clearScanClouds();
    legend_.clear();

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
