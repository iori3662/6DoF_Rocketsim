#ifdef _WIN32

#include "hrocket/dispersion.hpp"
#include "hrocket/io.hpp"
#include "hrocket/simulator.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>
#include <windows.h>
#include <commdlg.h>

namespace {

constexpr int id_vehicle = 101;
constexpr int id_thrust = 102;
constexpr int id_wind = 103;
constexpr int id_run = 105;
constexpr int id_descent_freefall = 106;
constexpr int id_descent_parachute = 107;
constexpr int id_wind_nominal = 108;
constexpr int id_wind_calm = 109;
constexpr int id_graph_combo = 110;
constexpr int id_dispersion_combo = 111;

HWND vehicle_edit{};
HWND thrust_edit{};
HWND wind_edit{};
HWND out_edit{};
HWND dispersion_edit{};
HWND wind_sigma_edit{};
HWND seed_edit{};
HWND sweep_max_wind_edit{};
HWND sweep_directions_edit{};
HWND status_text{};
HWND descent_freefall_radio{};
HWND descent_parachute_radio{};
HWND wind_nominal_radio{};
HWND wind_calm_radio{};
HWND graph_combo{};
HWND dispersion_combo{};
HFONT ui_font{};
HFONT title_font{};
HBRUSH bg_brush{};
HBRUSH panel_brush{};
HBRUSH edit_brush{};
hrocket::SimulationResult last_result;
std::vector<hrocket::DispersionPoint> last_dispersion;
bool has_result = false;

COLORREF bg_color = RGB(16, 20, 24);
COLORREF panel_color = RGB(23, 30, 38);
COLORREF border_color = RGB(50, 65, 80);
COLORREF text_color = RGB(230, 238, 245);
COLORREF muted_color = RGB(158, 176, 192);
COLORREF accent_blue = RGB(79, 195, 247);
COLORREF accent_yellow = RGB(255, 202, 40);
COLORREF accent_red = RGB(239, 83, 80);

std::wstring widen(const std::string& s) {
    if (s.empty()) {
        return {};
    }
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
    std::wstring out(n, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), out.data(), n);
    return out;
}

std::string narrow(const std::wstring& s) {
    if (s.empty()) {
        return {};
    }
    const int n = WideCharToMultiByte(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0, nullptr, nullptr);
    std::string out(n, '\0');
    WideCharToMultiByte(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), out.data(), n, nullptr, nullptr);
    return out;
}

std::wstring get_text(HWND hwnd) {
    const int n = GetWindowTextLengthW(hwnd);
    std::wstring s(n, L'\0');
    GetWindowTextW(hwnd, s.data(), n + 1);
    return s;
}

double get_double(HWND hwnd, double fallback) {
    const auto s = narrow(get_text(hwnd));
    return s.empty() ? fallback : std::stod(s);
}

int get_int(HWND hwnd, int fallback) {
    const auto s = narrow(get_text(hwnd));
    return s.empty() ? fallback : std::stoi(s);
}

void set_status(const std::wstring& text) {
    SetWindowTextW(status_text, text.c_str());
}

void choose_file(HWND owner, HWND edit) {
    wchar_t file[MAX_PATH]{};
    OPENFILENAMEW ofn{};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = owner;
    ofn.lpstrFilter = L"CSV Files\0*.csv\0All Files\0*.*\0";
    ofn.lpstrFile = file;
    ofn.nMaxFile = MAX_PATH;
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST;
    if (GetOpenFileNameW(&ofn)) {
        SetWindowTextW(edit, file);
    }
}

HWND add_label(HWND parent, const wchar_t* text, int x, int y, int w = 92) {
    HWND hwnd = CreateWindowW(L"STATIC", text, WS_CHILD | WS_VISIBLE, x, y, w, 22, parent, nullptr, nullptr, nullptr);
    SendMessageW(hwnd, WM_SETFONT, reinterpret_cast<WPARAM>(ui_font), TRUE);
    return hwnd;
}

HWND add_edit(HWND parent, int x, int y, int w, const wchar_t* text) {
    HWND hwnd = CreateWindowW(L"EDIT", text, WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL, x, y, w, 25, parent, nullptr, nullptr, nullptr);
    SendMessageW(hwnd, WM_SETFONT, reinterpret_cast<WPARAM>(ui_font), TRUE);
    return hwnd;
}

HWND add_button(HWND parent, const wchar_t* text, int x, int y, int w, int h, int id) {
    HWND hwnd = CreateWindowW(L"BUTTON", text, WS_CHILD | WS_VISIBLE, x, y, w, h, parent, reinterpret_cast<HMENU>(id), nullptr, nullptr);
    SendMessageW(hwnd, WM_SETFONT, reinterpret_cast<WPARAM>(ui_font), TRUE);
    return hwnd;
}

HWND add_radio(HWND parent, const wchar_t* text, int x, int y, int w, int h, int id, bool group) {
    DWORD style = WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON;
    if (group) {
        style |= WS_GROUP;
    }
    HWND hwnd = CreateWindowW(L"BUTTON", text, style, x, y, w, h, parent, reinterpret_cast<HMENU>(id), nullptr, nullptr);
    SendMessageW(hwnd, WM_SETFONT, reinterpret_cast<WPARAM>(ui_font), TRUE);
    return hwnd;
}

void round_rect(HDC dc, RECT r, int radius, COLORREF fill, COLORREF stroke) {
    HBRUSH brush = CreateSolidBrush(fill);
    HPEN pen = CreatePen(PS_SOLID, 1, stroke);
    HGDIOBJ old_brush = SelectObject(dc, brush);
    HGDIOBJ old_pen = SelectObject(dc, pen);
    RoundRect(dc, r.left, r.top, r.right, r.bottom, radius, radius);
    SelectObject(dc, old_brush);
    SelectObject(dc, old_pen);
    DeleteObject(brush);
    DeleteObject(pen);
}

void draw_text(HDC dc, const wchar_t* text, RECT r, COLORREF color, HFONT font, UINT format = DT_LEFT | DT_VCENTER | DT_SINGLELINE) {
    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, color);
    HGDIOBJ old_font = SelectObject(dc, font);
    DrawTextW(dc, text, -1, &r, format);
    SelectObject(dc, old_font);
}

struct PlotSeries {
    COLORREF color;
    std::vector<std::pair<double, double>> points;
};

std::pair<double, double> range_for(const std::vector<PlotSeries>& series, bool x_axis) {
    bool has = false;
    double lo = 0.0;
    double hi = 1.0;
    for (const auto& s : series) {
        for (const auto& p : s.points) {
            const double v = x_axis ? p.first : p.second;
            if (!has) {
                lo = hi = v;
                has = true;
            } else {
                lo = std::min(lo, v);
                hi = std::max(hi, v);
            }
        }
    }
    if (std::abs(hi - lo) < 1.0e-9) {
        hi = lo + 1.0;
    }
    const double pad = 0.06 * (hi - lo);
    return {lo - pad, hi + pad};
}

double speed(const hrocket::TrajectoryPoint& p) {
    return hrocket::norm(p.state.velocity_ned_mps);
}

std::wstring format_axis(double value) {
    wchar_t buffer[64]{};
    const double a = std::abs(value);
    if (a >= 100.0) {
        swprintf(buffer, 64, L"%.0f", value);
    } else if (a >= 10.0) {
        swprintf(buffer, 64, L"%.1f", value);
    } else {
        swprintf(buffer, 64, L"%.2f", value);
    }
    return buffer;
}

double roll_deg(const hrocket::TrajectoryPoint& p) {
    const auto q = hrocket::normalize(p.state.attitude_body_to_ned);
    return std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y)) * 57.29577951308232;
}

double pitch_deg(const hrocket::TrajectoryPoint& p) {
    const auto q = hrocket::normalize(p.state.attitude_body_to_ned);
    return std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0)) * 57.29577951308232;
}

double yaw_deg(const hrocket::TrajectoryPoint& p) {
    const auto q = hrocket::normalize(p.state.attitude_body_to_ned);
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)) * 57.29577951308232;
}

void draw_line_plot(HDC dc, RECT area, const std::vector<PlotSeries>& series) {
    round_rect(dc, area, 18, RGB(21, 27, 34), border_color);
    RECT plot{area.left + 62, area.top + 32, area.right - 24, area.bottom - 44};
    const auto [xmin, xmax] = range_for(series, true);
    const auto [ymin, ymax] = range_for(series, false);
    const auto sx = [&](double x) { return plot.left + static_cast<int>((x - xmin) / (xmax - xmin) * (plot.right - plot.left)); };
    const auto sy = [&](double y) { return plot.bottom - static_cast<int>((y - ymin) / (ymax - ymin) * (plot.bottom - plot.top)); };

    HPEN grid_pen = CreatePen(PS_SOLID, 1, RGB(38, 49, 60));
    HGDIOBJ old_pen = SelectObject(dc, grid_pen);
    for (int i = 0; i <= 5; ++i) {
        const int x = plot.left + i * (plot.right - plot.left) / 5;
        const int y = plot.top + i * (plot.bottom - plot.top) / 5;
        const double xv = xmin + i * (xmax - xmin) / 5.0;
        const double yv = ymax - i * (ymax - ymin) / 5.0;
        MoveToEx(dc, x, plot.top, nullptr);
        LineTo(dc, x, plot.bottom);
        MoveToEx(dc, plot.left, y, nullptr);
        LineTo(dc, plot.right, y);
        RECT x_label{x - 32, plot.bottom + 8, x + 32, plot.bottom + 28};
        const auto xs = format_axis(xv);
        draw_text(dc, xs.c_str(), x_label, muted_color, ui_font, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
        RECT y_label{area.left + 6, y - 10, plot.left - 8, y + 10};
        const auto ys = format_axis(yv);
        draw_text(dc, ys.c_str(), y_label, muted_color, ui_font, DT_RIGHT | DT_VCENTER | DT_SINGLELINE);
    }
    SelectObject(dc, old_pen);
    DeleteObject(grid_pen);

    for (const auto& s : series) {
        if (s.points.size() < 2) {
            continue;
        }
        HPEN pen = CreatePen(PS_SOLID, 2, s.color);
        old_pen = SelectObject(dc, pen);
        MoveToEx(dc, sx(s.points.front().first), sy(s.points.front().second), nullptr);
        for (const auto& p : s.points) {
            LineTo(dc, sx(p.first), sy(p.second));
        }
        SelectObject(dc, old_pen);
        DeleteObject(pen);
    }
}

void draw_dispersion_plot(HDC dc, RECT area) {
    round_rect(dc, area, 18, RGB(21, 27, 34), border_color);
    std::vector<PlotSeries> series{{accent_blue, {}}};
    for (const auto& p : last_dispersion) {
        series[0].points.push_back({p.impact_east_m, p.impact_north_m});
    }
    RECT plot{area.left + 62, area.top + 32, area.right - 24, area.bottom - 44};
    const auto [xmin, xmax] = range_for(series, true);
    const auto [ymin, ymax] = range_for(series, false);
    const auto sx = [&](double x) { return plot.left + static_cast<int>((x - xmin) / (xmax - xmin) * (plot.right - plot.left)); };
    const auto sy = [&](double y) { return plot.bottom - static_cast<int>((y - ymin) / (ymax - ymin) * (plot.bottom - plot.top)); };

    HPEN grid_pen = CreatePen(PS_SOLID, 1, RGB(38, 49, 60));
    HGDIOBJ old_pen = SelectObject(dc, grid_pen);
    for (int i = 0; i <= 5; ++i) {
        const int x = plot.left + i * (plot.right - plot.left) / 5;
        const int y = plot.top + i * (plot.bottom - plot.top) / 5;
        const double xv = xmin + i * (xmax - xmin) / 5.0;
        const double yv = ymax - i * (ymax - ymin) / 5.0;
        MoveToEx(dc, x, plot.top, nullptr);
        LineTo(dc, x, plot.bottom);
        MoveToEx(dc, plot.left, y, nullptr);
        LineTo(dc, plot.right, y);
        RECT x_label{x - 32, plot.bottom + 8, x + 32, plot.bottom + 28};
        const auto xs = format_axis(xv);
        draw_text(dc, xs.c_str(), x_label, muted_color, ui_font, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
        RECT y_label{area.left + 6, y - 10, plot.left - 8, y + 10};
        const auto ys = format_axis(yv);
        draw_text(dc, ys.c_str(), y_label, muted_color, ui_font, DT_RIGHT | DT_VCENTER | DT_SINGLELINE);
    }
    SelectObject(dc, old_pen);
    DeleteObject(grid_pen);

    HBRUSH brush = CreateSolidBrush(accent_blue);
    HGDIOBJ old_brush = SelectObject(dc, brush);
    for (const auto& p : last_dispersion) {
        const int x = sx(p.impact_east_m);
        const int y = sy(p.impact_north_m);
        Ellipse(dc, x - 4, y - 4, x + 4, y + 4);
    }
    SelectObject(dc, old_brush);
    DeleteObject(brush);
}

std::vector<PlotSeries> build_series(int graph_index) {
    std::vector<PlotSeries> series;
    if (!has_result) {
        return series;
    }
    if (graph_index == 0) {
        series.push_back({accent_blue, {}});
        for (const auto& p : last_result.points) {
            const double downrange = std::sqrt(p.state.position_ned_m.x * p.state.position_ned_m.x + p.state.position_ned_m.y * p.state.position_ned_m.y);
            series[0].points.push_back({downrange, p.altitude_m});
        }
    } else if (graph_index == 1) {
        series = {{accent_blue, {}}, {accent_yellow, {}}, {accent_red, {}}};
        for (const auto& p : last_result.points) {
            series[0].points.push_back({p.state.t_s, p.altitude_m});
            series[1].points.push_back({p.state.t_s, speed(p)});
            series[2].points.push_back({p.state.t_s, p.thrust_n});
        }
    } else if (graph_index == 2) {
        series = {{accent_yellow, {}}, {accent_blue, {}}, {accent_red, {}}};
        for (const auto& p : last_result.points) {
            series[0].points.push_back({p.state.t_s, roll_deg(p)});
            series[1].points.push_back({p.state.t_s, pitch_deg(p)});
            series[2].points.push_back({p.state.t_s, yaw_deg(p)});
        }
    } else if (graph_index == 3) {
        series = {{accent_blue, {}}, {accent_yellow, {}}, {accent_red, {}}};
        for (const auto& p : last_result.points) {
            series[0].points.push_back({p.state.t_s, p.state.velocity_ned_mps.x});
            series[1].points.push_back({p.state.t_s, p.state.velocity_ned_mps.y});
            series[2].points.push_back({p.state.t_s, p.state.velocity_ned_mps.z});
        }
    } else if (graph_index == 4) {
        series = {{accent_blue, {}}, {accent_red, {}}};
        for (const auto& p : last_result.points) {
            series[0].points.push_back({p.state.t_s, p.control_deflection_deg});
            series[1].points.push_back({p.state.t_s, hrocket::norm(p.control_moment_body_nm)});
        }
    }
    return series;
}

void draw_graph(HDC dc, RECT area) {
    RECT title{area.left, area.top, area.right, area.top + 28};
    const int graph_index = static_cast<int>(SendMessageW(graph_combo, CB_GETCURSEL, 0, 0));
    const wchar_t* titles[] = {L"Trajectory", L"Altitude / Speed / Thrust", L"Attitude", L"Velocity", L"Twin Tail Control", L"Impact Dispersion"};
    draw_text(dc, titles[std::max(0, graph_index)], title, text_color, title_font);
    RECT plot{area.left, area.top + 42, area.right, area.bottom};
    if (!has_result) {
        round_rect(dc, plot, 18, RGB(21, 27, 34), border_color);
        RECT msg{plot.left + 24, plot.top + 20, plot.right - 24, plot.top + 60};
        draw_text(dc, L"Run a simulation to draw graphs.", msg, muted_color, ui_font);
        return;
    }
    if (graph_index == 5) {
        draw_dispersion_plot(dc, plot);
    } else {
        draw_line_plot(dc, plot, build_series(graph_index));
    }
}

void run(HWND owner) {
    try {
        hrocket::SimulationInputs inputs;
        inputs.vehicle = hrocket::load_vehicle_csv(narrow(get_text(vehicle_edit)));
        inputs.thrust = hrocket::load_thrust_csv(narrow(get_text(thrust_edit)));
        inputs.wind = hrocket::load_wind_csv(narrow(get_text(wind_edit)));
        inputs.config.descent_mode = SendMessageW(descent_parachute_radio, BM_GETCHECK, 0, 0) == BST_CHECKED
            ? hrocket::DescentMode::Parachute
            : hrocket::DescentMode::FreeFall;
        inputs.config.wind_mode = SendMessageW(wind_calm_radio, BM_GETCHECK, 0, 0) == BST_CHECKED
            ? hrocket::WindMode::Calm
            : hrocket::WindMode::Nominal;

        const int dispersion_runs = std::max(1, get_int(dispersion_edit, 1));
        hrocket::DispersionConfig dispersion_config;
        dispersion_config.mode = SendMessageW(dispersion_combo, CB_GETCURSEL, 0, 0) == 1
            ? hrocket::DispersionMode::WindSweep
            : hrocket::DispersionMode::MonteCarlo;
        dispersion_config.runs = dispersion_runs;
        dispersion_config.wind_sigma_mps = get_double(wind_sigma_edit, 0.0);
        dispersion_config.seed = static_cast<unsigned>(std::max(0, get_int(seed_edit, 1)));
        dispersion_config.sweep_max_wind_mps = get_double(sweep_max_wind_edit, 7.0);
        dispersion_config.sweep_step_mps = 1.0;
        dispersion_config.sweep_directions = std::max(1, get_int(sweep_directions_edit, 16));
        const auto out_dir = std::filesystem::path(narrow(get_text(out_edit)));

        std::filesystem::create_directories(out_dir);
        last_result = hrocket::run_simulation(inputs);
        last_dispersion = hrocket::run_dispersion(inputs, last_result, dispersion_config);

        hrocket::write_trajectory_csv(out_dir / "trajectory.csv", last_result, inputs.vehicle);
        hrocket::write_kml(out_dir / "trajectory.kml", last_result, inputs.vehicle, last_dispersion);
        hrocket::write_summary_csv(out_dir / "summary.csv", last_result);
        hrocket::write_dispersion_csv(out_dir / "dispersion.csv", last_dispersion);
        hrocket::write_graph_svgs(out_dir, last_result, last_dispersion);
        has_result = true;
        set_status(L"Simulation complete. CSV, KML, dispersion, and SVG graphs were saved.");
        InvalidateRect(owner, nullptr, FALSE);
    } catch (const std::exception& e) {
        MessageBoxW(owner, widen(e.what()).c_str(), L"Simulation error", MB_ICONERROR);
    }
}

void create_fonts() {
    ui_font = CreateFontW(-16, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
        CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, L"Segoe UI");
    title_font = CreateFontW(-24, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
        CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, L"Segoe UI");
    bg_brush = CreateSolidBrush(bg_color);
    panel_brush = CreateSolidBrush(panel_color);
    edit_brush = CreateSolidBrush(RGB(28, 36, 45));
}

LRESULT CALLBACK wnd_proc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
    case WM_CREATE:
        create_fonts();
        add_label(hwnd, L"Vehicle", 36, 78);
        vehicle_edit = add_edit(hwnd, 128, 75, 360, L"samples\\vehicle.csv");
        add_button(hwnd, L"...", 498, 75, 36, 25, id_vehicle);

        add_label(hwnd, L"Thrust", 36, 114);
        thrust_edit = add_edit(hwnd, 128, 111, 360, L"samples\\thrust.csv");
        add_button(hwnd, L"...", 498, 111, 36, 25, id_thrust);

        add_label(hwnd, L"Wind CSV", 36, 150);
        wind_edit = add_edit(hwnd, 128, 147, 360, L"samples\\wind.csv");
        add_button(hwnd, L"...", 498, 147, 36, 25, id_wind);

        add_label(hwnd, L"Output", 36, 186);
        out_edit = add_edit(hwnd, 128, 183, 406, L"out");

        add_label(hwnd, L"Descent", 36, 226);
        descent_freefall_radio = add_radio(hwnd, L"Free fall", 128, 224, 100, 24, id_descent_freefall, true);
        descent_parachute_radio = add_radio(hwnd, L"Parachute", 236, 224, 120, 24, id_descent_parachute, false);
        SendMessageW(descent_freefall_radio, BM_SETCHECK, BST_CHECKED, 0);

        add_label(hwnd, L"Wind", 36, 262);
        wind_nominal_radio = add_radio(hwnd, L"Nominal CSV", 128, 260, 120, 24, id_wind_nominal, true);
        wind_calm_radio = add_radio(hwnd, L"No wind", 258, 260, 90, 24, id_wind_calm, false);
        SendMessageW(wind_nominal_radio, BM_SETCHECK, BST_CHECKED, 0);

        add_label(hwnd, L"Runs", 36, 302);
        dispersion_edit = add_edit(hwnd, 128, 299, 70, L"30");
        add_label(hwnd, L"Wind sigma", 214, 302, 92);
        wind_sigma_edit = add_edit(hwnd, 310, 299, 70, L"2.0");
        add_label(hwnd, L"Seed", 398, 302, 42);
        seed_edit = add_edit(hwnd, 444, 299, 90, L"1");

        add_label(hwnd, L"Dispersion", 36, 342);
        dispersion_combo = CreateWindowW(L"COMBOBOX", L"", WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST, 128, 339, 152, 200, hwnd, reinterpret_cast<HMENU>(id_dispersion_combo), nullptr, nullptr);
        SendMessageW(dispersion_combo, WM_SETFONT, reinterpret_cast<WPARAM>(ui_font), TRUE);
        SendMessageW(dispersion_combo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Monte Carlo"));
        SendMessageW(dispersion_combo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Wind sweep"));
        SendMessageW(dispersion_combo, CB_SETCURSEL, 0, 0);
        add_label(hwnd, L"Max wind", 296, 342, 70);
        sweep_max_wind_edit = add_edit(hwnd, 370, 339, 54, L"7");
        add_label(hwnd, L"Dirs", 438, 342, 38);
        sweep_directions_edit = add_edit(hwnd, 480, 339, 54, L"16");

        add_button(hwnd, L"Run Simulation", 36, 388, 498, 36, id_run);

        graph_combo = CreateWindowW(L"COMBOBOX", L"", WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST, 592, 75, 260, 200, hwnd, reinterpret_cast<HMENU>(id_graph_combo), nullptr, nullptr);
        SendMessageW(graph_combo, WM_SETFONT, reinterpret_cast<WPARAM>(ui_font), TRUE);
        SendMessageW(graph_combo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Trajectory"));
        SendMessageW(graph_combo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Altitude / Speed / Thrust"));
        SendMessageW(graph_combo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Attitude"));
        SendMessageW(graph_combo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Velocity"));
        SendMessageW(graph_combo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Twin Tail Control"));
        SendMessageW(graph_combo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Impact Dispersion"));
        SendMessageW(graph_combo, CB_SETCURSEL, 0, 0);

        status_text = CreateWindowW(L"STATIC", L"Ready", WS_CHILD | WS_VISIBLE, 36, 656, 1110, 26, hwnd, nullptr, nullptr, nullptr);
        SendMessageW(status_text, WM_SETFONT, reinterpret_cast<WPARAM>(ui_font), TRUE);
        return 0;
    case WM_COMMAND:
        switch (LOWORD(wp)) {
        case id_vehicle:
            choose_file(hwnd, vehicle_edit);
            break;
        case id_thrust:
            choose_file(hwnd, thrust_edit);
            break;
        case id_wind:
            choose_file(hwnd, wind_edit);
            break;
        case id_run:
            run(hwnd);
            break;
        case id_graph_combo:
            if (HIWORD(wp) == CBN_SELCHANGE) {
                InvalidateRect(hwnd, nullptr, FALSE);
            }
            break;
        }
        return 0;
    case WM_CTLCOLORSTATIC: {
        HDC dc = reinterpret_cast<HDC>(wp);
        SetTextColor(dc, text_color);
        SetBkMode(dc, TRANSPARENT);
        return reinterpret_cast<LRESULT>(bg_brush);
    }
    case WM_CTLCOLOREDIT:
    case WM_CTLCOLORLISTBOX: {
        HDC dc = reinterpret_cast<HDC>(wp);
        SetTextColor(dc, text_color);
        SetBkColor(dc, RGB(28, 36, 45));
        return reinterpret_cast<LRESULT>(edit_brush);
    }
    case WM_PAINT: {
        PAINTSTRUCT ps{};
        HDC dc = BeginPaint(hwnd, &ps);
        RECT client{};
        GetClientRect(hwnd, &client);
        FillRect(dc, &client, bg_brush);

        RECT header{32, 18, 1120, 54};
        draw_text(dc, L"Hybrid Rocket 6DoF Simulator", header, text_color, title_font);
        RECT sub{32, 48, 1120, 72};
        draw_text(dc, L"Local C++ solver with CSV/KML output, SVG graphs, and Monte Carlo impact dispersion.", sub, muted_color, ui_font);

        round_rect(dc, {24, 64, 552, 444}, 20, panel_color, border_color);
        round_rect(dc, {576, 64, 1156, 640}, 20, panel_color, border_color);
        draw_graph(dc, {600, 116, 1132, 618});
        EndPaint(hwnd, &ps);
        return 0;
    }
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    default:
        return DefWindowProcW(hwnd, msg, wp, lp);
    }
}

} // namespace

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR, int show) {
    const wchar_t class_name[] = L"Hrocket6DoFWindow";
    WNDCLASSW wc{};
    wc.lpfnWndProc = wnd_proc;
    wc.hInstance = instance;
    wc.lpszClassName = class_name;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = bg_brush;
    RegisterClassW(&wc);

    HWND hwnd = CreateWindowExW(0, class_name, L"Hybrid Rocket 6DoF Simulator", WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT, 1200, 740, nullptr, nullptr, instance, nullptr);
    ShowWindow(hwnd, show);

    MSG msg{};
    while (GetMessageW(&msg, nullptr, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    return 0;
}

#endif
