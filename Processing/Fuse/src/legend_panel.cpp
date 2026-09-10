#include "legend_panel.h"

#include <algorithm>
#include <iomanip>
#include <sstream>

#include <vtkRenderWindow.h>

#include "scan_colors.h"

namespace
{
constexpr const char* kHeaderId = "legend_header";

std::string legendRowId(int row) { return "legend_row_" + std::to_string(row); }

// Right-pads to a fixed width. The overlay font is proportional, so this
// only approximates aligned columns - it is still far easier to scan than
// unpadded text.
std::string pad(const std::string& s, size_t width)
{
    if (s.size() >= width) return s.substr(0, width);
    return s + std::string(width - s.size(), ' ');
}

std::string formatMillimetres(double metres)
{
    if (metres >= 1e6) return "-";
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << metres * 1000.0;
    return ss.str();
}

std::string formatPercent(double fraction)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(0) << fraction * 100.0 << "%";
    return ss.str();
}

// One-character status glyph, so the status column stays narrow enough to
// leave room for the scan name.
const char* statusGlyph(ScanStatus status)
{
    switch (status)
    {
        case ScanStatus::Pending:       return ".";
        case ScanStatus::Reference:     return "R";
        case ScanStatus::CoarseAligned: return "c";
        case ScanStatus::CoarseFailed:  return "!";
        case ScanStatus::Refined:       return "F";
        case ScanStatus::RefineFailed:  return "!";
        case ScanStatus::Excluded:      return "X";
    }
    return "?";
}
} // namespace

void LegendPanel::attach(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    viewer_ = std::move(viewer);
}

void LegendPanel::clear()
{
    if (!viewer_) return;
    for (const auto& id : drawn_ids_)
        viewer_->removeShape(id);
    drawn_ids_.clear();
    rows_.clear();
}

std::string LegendPanel::formatRow(const FusionSession& session,
                                    size_t index,
                                    bool is_selected,
                                    bool is_visible) const
{
    const auto& scan = session.scan(index);
    // Show the freshest measurement: fine once ICP has run, else coarse.
    const AlignmentQuality& q =
        scan.fine_quality.valid() ? scan.fine_quality : scan.coarse_quality;

    std::ostringstream row;
    row << (is_selected ? ">" : " ")
        << (is_visible ? "[o] " : "[ ] ")
        << pad(std::to_string(index), 3)
        << pad(statusGlyph(scan.status), 2)
        << pad(scan.input.shortLabel(), 16)
        << pad(q.valid() ? formatMillimetres(q.rmse) + "mm" : "-", 9)
        << pad(q.valid() ? formatPercent(q.inlier_fraction) : "-", 5);

    if (scan.flagged) row << " !";
    return row.str();
}

void LegendPanel::draw(const FusionSession& session,
                        int selected,
                        const std::vector<bool>& visible)
{
    if (!viewer_) return;
    clear();

    int window_height = 800;
    int window_width  = 800;
    // getRenderWindow() hands back a vtkSmartPointer, so bind it as a value.
    if (auto window = viewer_->getRenderWindow())
    {
        const int* size = window->GetSize();
        if (size && size[0] > 0) window_width  = size[0];
        if (size && size[1] > 0) window_height = size[1];
    }

    // Never let the click target swallow more than a third of the window,
    // or dragging to rotate the view near the left edge stops working.
    hit_width_ = std::min(style_.max_panel_width, window_width / 3);

    // Rows are laid out downward from the top margin; VTK's text origin is
    // the window's bottom-left, so a row's y is measured up from the bottom.
    const int usable_height = window_height - style_.top_margin - style_.bottom_margin;
    const int max_rows = std::max(1, usable_height / style_.row_height - 1); // -1 for the header
    const int scan_count = static_cast<int>(session.size());

    // Scroll only as far as needed to bring the selection into view.
    if (selected >= 0)
    {
        if (selected < scroll_offset_)
            scroll_offset_ = selected;
        else if (selected >= scroll_offset_ + max_rows)
            scroll_offset_ = selected - max_rows + 1;
    }
    scroll_offset_ = std::clamp(scroll_offset_, 0, std::max(0, scan_count - max_rows));

    const int first = scroll_offset_;
    const int last  = std::min(scan_count, first + max_rows); // exclusive

    // Header: column key plus a scroll indicator when the list is clipped.
    std::ostringstream header;
    header << "  #  st  scan            rmse     ovl";
    if (scan_count > max_rows)
        header << "   (" << (first + 1) << "-" << last << " of " << scan_count << ")";

    int y = window_height - style_.top_margin;
    viewer_->addText(header.str(), style_.left_margin, y, style_.font_size,
                      0.75, 0.75, 0.8, kHeaderId);
    drawn_ids_.push_back(kHeaderId);

    for (int i = first; i < last; ++i)
    {
        y -= style_.row_height;

        const auto& scan = session.scan(static_cast<size_t>(i));
        const bool is_selected = (i == selected);
        const bool is_visible =
            (static_cast<size_t>(i) < visible.size()) ? visible[static_cast<size_t>(i)] : true;

        double r, g, b;
        ScanColors::forIndex(i, r, g, b);
        if (scan.status == ScanStatus::Excluded)
        {
            // Excluded scans stay listed (you need to be able to bring them
            // back) but dim right down so they read as inactive.
            r *= 0.35; g *= 0.35; b *= 0.35;
        }
        else if (is_selected)
        {
            // Lift the selected row toward white so it reads as selected
            // even against its own saturated hue.
            r = 0.4 + 0.6 * r; g = 0.4 + 0.6 * g; b = 0.4 + 0.6 * b;
        }

        const std::string id = legendRowId(i);
        viewer_->addText(formatRow(session, static_cast<size_t>(i), is_selected, is_visible),
                          style_.left_margin, y, style_.font_size, r, g, b, id);
        drawn_ids_.push_back(id);

        // The clickable band for this row. Text is drawn from its baseline
        // upward, so the band spans one row height above the baseline.
        rows_.push_back(DrawnRow{ i, y, y + style_.row_height });
    }
}

int LegendPanel::hitTest(int x, int y) const
{
    if (x < style_.left_margin - 4 || x > style_.left_margin + hit_width_)
        return -1;

    for (const auto& row : rows_)
    {
        if (y >= row.y_bottom && y < row.y_top)
            return row.scan_index;
    }
    return -1;
}
