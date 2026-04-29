#ifdef _WIN32

#include "hrocket/io.hpp"
#include "hrocket/simulator.hpp"

#include <algorithm>
#include <filesystem>
#include <string>
#include <windows.h>
#include <commdlg.h>

namespace {

constexpr int id_vehicle = 101;
constexpr int id_thrust = 102;
constexpr int id_wind = 103;
constexpr int id_out = 104;
constexpr int id_run = 105;
constexpr int id_descent_freefall = 106;
constexpr int id_descent_parachute = 107;
constexpr int id_wind_nominal = 108;
constexpr int id_wind_calm = 109;

HWND vehicle_edit{};
HWND thrust_edit{};
HWND wind_edit{};
HWND out_edit{};
HWND status_text{};
HWND descent_freefall_radio{};
HWND descent_parachute_radio{};
HWND wind_nominal_radio{};
HWND wind_calm_radio{};

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

void set_status(const std::wstring& text) {
    SetWindowTextW(status_text, text.c_str());
}

hrocket::DispersionPoint summarize(int run_index, const hrocket::SimulationResult& result) {
    hrocket::DispersionPoint point;
    point.run_index = run_index;
    point.impacted = result.impacted;
    for (const auto& p : result.points) {
        point.apogee_m = std::max(point.apogee_m, p.altitude_m);
    }
    if (!result.points.empty()) {
        const auto& last = result.points.back();
        point.impact_north_m = last.state.position_ned_m.x;
        point.impact_east_m = last.state.position_ned_m.y;
        point.flight_time_s = last.state.t_s;
    }
    return point;
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
        const auto out_dir = std::filesystem::path(narrow(get_text(out_edit)));
        std::filesystem::create_directories(out_dir);
        const auto result = hrocket::run_simulation(inputs);
        hrocket::write_trajectory_csv(out_dir / "trajectory.csv", result, inputs.vehicle);
        hrocket::write_kml(out_dir / "trajectory.kml", result, inputs.vehicle);
        hrocket::write_summary_csv(out_dir / "summary.csv", result);
        hrocket::write_dispersion_csv(out_dir / "dispersion.csv", {summarize(0, result)});
        set_status(L"Simulation complete. trajectory.csv, trajectory.kml, summary.csv を出力しました。");
    } catch (const std::exception& e) {
        MessageBoxW(owner, widen(e.what()).c_str(), L"Simulation error", MB_ICONERROR);
    }
}

HWND add_label(HWND parent, const wchar_t* text, int x, int y) {
    return CreateWindowW(L"STATIC", text, WS_CHILD | WS_VISIBLE, x, y, 90, 24, parent, nullptr, nullptr, nullptr);
}

HWND add_edit(HWND parent, int x, int y, const wchar_t* text) {
    return CreateWindowW(L"EDIT", text, WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL, x, y, 470, 24, parent, nullptr, nullptr, nullptr);
}

LRESULT CALLBACK wnd_proc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
    case WM_CREATE:
        add_label(hwnd, L"Vehicle", 16, 20);
        vehicle_edit = add_edit(hwnd, 110, 18, L"samples\\vehicle.csv");
        CreateWindowW(L"BUTTON", L"...", WS_CHILD | WS_VISIBLE, 590, 18, 34, 24, hwnd, reinterpret_cast<HMENU>(id_vehicle), nullptr, nullptr);

        add_label(hwnd, L"Thrust", 16, 56);
        thrust_edit = add_edit(hwnd, 110, 54, L"samples\\thrust.csv");
        CreateWindowW(L"BUTTON", L"...", WS_CHILD | WS_VISIBLE, 590, 54, 34, 24, hwnd, reinterpret_cast<HMENU>(id_thrust), nullptr, nullptr);

        add_label(hwnd, L"Wind", 16, 92);
        wind_edit = add_edit(hwnd, 110, 90, L"samples\\wind.csv");
        CreateWindowW(L"BUTTON", L"...", WS_CHILD | WS_VISIBLE, 590, 90, 34, 24, hwnd, reinterpret_cast<HMENU>(id_wind), nullptr, nullptr);

        add_label(hwnd, L"Output", 16, 128);
        out_edit = add_edit(hwnd, 110, 126, L"out");
        add_label(hwnd, L"Descent", 16, 166);
        descent_freefall_radio = CreateWindowW(L"BUTTON", L"Free fall", WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON | WS_GROUP, 110, 164, 100, 24, hwnd, reinterpret_cast<HMENU>(id_descent_freefall), nullptr, nullptr);
        descent_parachute_radio = CreateWindowW(L"BUTTON", L"Parachute", WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON, 220, 164, 120, 24, hwnd, reinterpret_cast<HMENU>(id_descent_parachute), nullptr, nullptr);
        SendMessageW(descent_freefall_radio, BM_SETCHECK, BST_CHECKED, 0);

        add_label(hwnd, L"Wind", 16, 202);
        wind_nominal_radio = CreateWindowW(L"BUTTON", L"Nominal CSV", WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON | WS_GROUP, 110, 200, 120, 24, hwnd, reinterpret_cast<HMENU>(id_wind_nominal), nullptr, nullptr);
        wind_calm_radio = CreateWindowW(L"BUTTON", L"No wind", WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON, 240, 200, 100, 24, hwnd, reinterpret_cast<HMENU>(id_wind_calm), nullptr, nullptr);
        SendMessageW(wind_nominal_radio, BM_SETCHECK, BST_CHECKED, 0);

        CreateWindowW(L"BUTTON", L"Run", WS_CHILD | WS_VISIBLE, 110, 240, 90, 30, hwnd, reinterpret_cast<HMENU>(id_run), nullptr, nullptr);
        status_text = CreateWindowW(L"STATIC", L"Ready", WS_CHILD | WS_VISIBLE, 16, 288, 610, 24, hwnd, nullptr, nullptr, nullptr);
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
        }
        return 0;
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
    RegisterClassW(&wc);

    HWND hwnd = CreateWindowExW(0, class_name, L"Hybrid Rocket 6DoF Simulator", WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT, 660, 380, nullptr, nullptr, instance, nullptr);
    ShowWindow(hwnd, show);

    MSG msg{};
    while (GetMessageW(&msg, nullptr, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    return 0;
}

#endif
