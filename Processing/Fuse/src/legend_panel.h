#pragma once

#include <memory>
#include <string>
#include <vector>

#include <pcl/visualization/pcl_visualizer.h>

#include "fusion_session.h"

// The scan list rendered alongside the 3D view: one row per scan, showing
// its color swatch, name, status and alignment error, with the selected
// row marked. Replaces the old "press 1-9 and hope" scheme - rows are
// labeled, and clicking one selects that scan.
//
// The panel draws itself as 2D text overlays at pixel positions it chooses,
// so it also knows where every row is on screen and can hit-test a mouse
// click back to a scan index. Both the text placement and the mouse
// coordinates use VTK's display convention (origin at the window's
// bottom-left), which is what keeps hitTest consistent with draw.
//
// More scans than fit on screen scroll: the panel keeps the selected row
// within view and marks the rows above/below that are scrolled out of it.
class LegendPanel
{
public:
    struct Style
    {
        int left_margin   = 12;   // px from the window's left edge
        int top_margin    = 30;   // px below the window's top edge
        int bottom_margin = 60;   // px kept clear above the status line
        int row_height    = 15;
        int font_size     = 12;
        int max_panel_width = 430; // click-target width cap; narrowed to fit small windows
    };

    void attach(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer);

    // Redraws every row. `selected` is a scan index or -1 for no selection;
    // `visible` is indexed by scan and may be shorter than the session.
    void draw(const FusionSession& session,
               int selected,
               const std::vector<bool>& visible);

    // Maps a click at window coordinates to a scan index, or -1 if the
    // click missed the panel.
    int hitTest(int x, int y) const;

    void clear();

    const Style& style() const { return style_; }
    Style&       style()       { return style_; }

private:
    // A row as actually drawn, retained so clicks can be resolved against
    // the same geometry that was rendered.
    struct DrawnRow
    {
        int scan_index;
        int y_bottom;
        int y_top;
    };

    std::string formatRow(const FusionSession& session,
                           size_t index,
                           bool is_selected,
                           bool is_visible) const;

    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    Style style_;
    std::vector<std::string> drawn_ids_;
    std::vector<DrawnRow>    rows_;

    // Index of the first scan row currently shown; moves only as far as it
    // must to keep the selection on screen, so the list doesn't jump around
    // while you arrow through it.
    int scroll_offset_ = 0;

    // Click-target width actually used at the last draw(), which depends on
    // the window size - kept so hitTest matches what was drawn.
    int hit_width_ = 430;
};
