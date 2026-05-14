#include "ui_app/app_window.hpp"
#include "ui_app/i18n_manager.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QComboBox>
#include <QPushButton>
#include <QMessageBox>
#include <QFileDialog>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

// ---------- loadCalibYaml -----------------------------------------------
// Reads hand_eye.yaml and populates all spinboxes.
// Converts quaternion → RPY (degrees) for the TF spinboxes.
void AppWindow::loadCalibYaml(const QString& path)
{
    if (path.isEmpty()) return;
    try {
        YAML::Node cfg = YAML::LoadFile(path.toStdString());

        // --- hand_eye → TF spinboxes ---
        if (cfg["hand_eye"]) {
            auto he = cfg["hand_eye"];
            spin_calib_tf_[0]->setValue(he["x"]  ? he["x"].as<double>()  : 0.05);
            spin_calib_tf_[1]->setValue(he["y"]  ? he["y"].as<double>()  : 0.00);
            spin_calib_tf_[2]->setValue(he["z"]  ? he["z"].as<double>()  : 0.08);
            // Convert quaternion → RPY (ZYX extrinsic, degrees)
            double qx = he["qx"] ? he["qx"].as<double>() : 0.0;
            double qy = he["qy"] ? he["qy"].as<double>() : 0.0;
            double qz = he["qz"] ? he["qz"].as<double>() : 0.7071;
            double qw = he["qw"] ? he["qw"].as<double>() : 0.7071;
            double n = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
            if (n > 1e-6) { qx/=n; qy/=n; qz/=n; qw/=n; }
            double sinr = 2*(qw*qx + qy*qz), cosr = 1-2*(qx*qx+qy*qy);
            double sinp = 2*(qw*qy - qz*qx);
            double siny = 2*(qw*qz + qx*qy), cosy = 1-2*(qy*qy+qz*qz);
            double rx_deg = std::atan2(sinr, cosr) * 180.0 / M_PI;
            double ry_deg = (std::abs(sinp) >= 1.0
                ? std::copysign(90.0, sinp)
                : std::asin(sinp) * 180.0 / M_PI);
            double rz_deg = std::atan2(siny, cosy) * 180.0 / M_PI;
            spin_calib_tf_[3]->setValue(rx_deg);
            spin_calib_tf_[4]->setValue(ry_deg);
            spin_calib_tf_[5]->setValue(rz_deg);
        }

        // --- hand.close → hand spinboxes ---
        if (cfg["hand"] && cfg["hand"]["close"]) {
            auto cl = cfg["hand"]["close"];
            for (int i = 0; i < 6 && i < (int)cl.size(); i++)
                spin_calib_hand_[i]->setValue(cl[i].as<int>());
        }

        // --- grasp_offset → offset spinboxes ---
        if (cfg["grasp_offset"]) {
            auto go = cfg["grasp_offset"];
            spin_grasp_offset_[0]->setValue(go["x"] ? go["x"].as<double>() : 0.0);
            spin_grasp_offset_[1]->setValue(go["y"] ? go["y"].as<double>() : 0.0);
            spin_grasp_offset_[2]->setValue(go["z"] ? go["z"].as<double>() : 0.0);
        }

        calib_yaml_path_ = path;
        if (label_calib_status_)
            label_calib_status_->setText(
                tr_ui("已加载配置: ", "Loaded: ") + path);
    } catch (const YAML::Exception& e) {
        if (label_calib_status_)
            label_calib_status_->setText(
                tr_ui("加载失败: ", "Load failed: ") +
                QString::fromStdString(e.what()));
    }
}
// ------------------------------------------------------------------------

QWidget* AppWindow::createCalibrationTab() {
    auto* widget = new QWidget();
    auto* main_layout = new QVBoxLayout(widget);
    main_layout->setSpacing(8);

    // ================================================================
    // Section -1: Config file path (load/save sync)
    // ================================================================
    auto* file_group = new QGroupBox(tr_ui("配置文件", "Config File"));
    auto* file_layout = new QHBoxLayout(file_group);

    // Resolve default path: installed share dir
    QString default_yaml_path;
    try {
        std::string share = ament_index_cpp::get_package_share_directory("vision_grasp");
        default_yaml_path = QString::fromStdString(share + "/config/hand_eye.yaml");
    } catch (...) {
        default_yaml_path = "";
    }

    auto* edit_yaml_path = new QLineEdit(default_yaml_path);
    edit_yaml_path->setPlaceholderText(tr_ui("hand_eye.yaml 路径", "Path to hand_eye.yaml"));
    auto* btn_browse = new QPushButton(tr_ui("浏览", "Browse"));
    btn_browse->setFixedWidth(55);
    auto* btn_load_yaml = new QPushButton(tr_ui("载入", "Load"));
    btn_load_yaml->setFixedWidth(55);
    file_layout->addWidget(edit_yaml_path, 1);
    file_layout->addWidget(btn_browse);
    file_layout->addWidget(btn_load_yaml);
    main_layout->addWidget(file_group);

    // ================================================================
    // Section 0: Gemini 305 Camera Selection
    // ================================================================
    auto* cam_group = new QGroupBox(
        tr_ui("视觉相机选择 (Gemini 305)", "Vision Camera (Gemini 305)"));
    auto* cam_layout = new QHBoxLayout(cam_group);

    combo_calib_camera_ = new QComboBox();
    combo_calib_camera_->setMinimumWidth(220);
    auto* btn_scan_cameras = new QPushButton(tr_ui("扫描", "Scan"));
    btn_scan_cameras->setFixedWidth(60);
    cam_layout->addWidget(new QLabel(tr_ui("相机:", "Camera:")));
    cam_layout->addWidget(combo_calib_camera_, 1);
    cam_layout->addWidget(btn_scan_cameras);
    main_layout->addWidget(cam_group);

    // Helper: populate combo with only Gemini 305 cameras (ns contains "305" or "cam_305")
    // Falls back to all cameras if none match the filter.
    auto populate_cameras = [this]() {
        auto all = node_->scan_cameras();
        combo_calib_camera_->clear();
        // Filter: keep only namespaces that look like Gemini 305
        // Orbbec driver uses camera_name as ns, so "305" in the name is the signal.
        std::vector<std::string> filtered;
        for (const auto& ns : all) {
            std::string lower = ns;
            std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
            if (lower.find("305") != std::string::npos ||
                lower.find("cam_305") != std::string::npos ||
                lower.find("gemini305") != std::string::npos) {
                filtered.push_back(ns);
            }
        }
        // If no 305 found, show all (better than empty list)
        const auto& show = filtered.empty() ? all : filtered;
        for (const auto& ns : show)
            combo_calib_camera_->addItem(QString::fromStdString(ns));
        // Default: first entry
        if (combo_calib_camera_->count() > 0)
            combo_calib_camera_->setCurrentIndex(0);
    };

    // Scan on button click
    connect(btn_scan_cameras, &QPushButton::clicked, this, [this, populate_cameras]() {
        populate_cameras();
        label_calib_status_->setText(
            tr_ui("已扫描到 ", "Found ") +
            QString::number(combo_calib_camera_->count()) +
            tr_ui(" 台相机", " camera(s)"));
    });

    // Auto-scan once at construction
    populate_cameras();

    // ================================================================
    // Section 1: Hand-Eye TF Calibration (link_6 → camera_link)
    // ================================================================
    auto* tf_group = new QGroupBox(
        tr_ui("手眼标定 TF (L_base_link → cam_305_link)",
              "Hand-Eye TF (L_base_link → cam_305_link)"));
    auto* tf_layout = new QGridLayout(tf_group);

    const char* tf_labels[] = {"X (m)", "Y (m)", "Z (m)", "RX (°)", "RY (°)", "RZ (°)"};
    double tf_defaults[] = {0.05, 0.0, 0.08, 0.0, 0.0, 0.0};

    for (int i = 0; i < 6; i++) {
        int row = i / 3;
        int col = (i % 3) * 2;
        tf_layout->addWidget(new QLabel(tf_labels[i]), row, col);
        spin_calib_tf_[i] = new QDoubleSpinBox();
        if (i < 3) {
            spin_calib_tf_[i]->setRange(-0.50, 0.50);
            spin_calib_tf_[i]->setSingleStep(0.001);
            spin_calib_tf_[i]->setDecimals(4);
        } else {
            spin_calib_tf_[i]->setRange(-180.0, 180.0);
            spin_calib_tf_[i]->setSingleStep(0.5);
            spin_calib_tf_[i]->setDecimals(2);
        }
        spin_calib_tf_[i]->setValue(tf_defaults[i]);
        tf_layout->addWidget(spin_calib_tf_[i], row, col + 1);
    }

    auto* tf_btn_layout = new QHBoxLayout();
    auto* btn_apply_tf = new QPushButton(tr_ui("发布 TF", "Apply TF"));
    auto* btn_save_tf = new QPushButton(tr_ui("保存配置", "Save Config"));
    tf_btn_layout->addWidget(btn_apply_tf);
    tf_btn_layout->addWidget(btn_save_tf);
    tf_layout->addLayout(tf_btn_layout, 2, 0, 1, 6);

    main_layout->addWidget(tf_group);

    // ================================================================
    // Section 2: Hand Position Testing (DH116)
    // ================================================================
    auto* hand_group = new QGroupBox(
        tr_ui("灵巧手抓取调试 (DH116)", "Hand Grasp Testing (DH116)"));
    auto* hand_layout = new QGridLayout(hand_group);

    for (int i = 0; i < 6; i++) {
        hand_layout->addWidget(new QLabel(QString("M%1").arg(i + 1)), 0, i);
        spin_calib_hand_[i] = new QSpinBox();
        spin_calib_hand_[i]->setRange(0, 1000);
        spin_calib_hand_[i]->setSingleStep(50);
        spin_calib_hand_[i]->setValue(0);
        hand_layout->addWidget(spin_calib_hand_[i], 1, i);
    }

    auto* hand_btn_row1 = new QHBoxLayout();
    auto* btn_hand_send = new QPushButton(tr_ui("发送位置", "Send Position"));
    auto* btn_hand_open = new QPushButton(tr_ui("全开 (0)", "Open All (0)"));
    auto* btn_hand_close = new QPushButton(tr_ui("抓取测试", "Grasp Test"));
    auto* btn_hand_read = new QPushButton(tr_ui("读取当前位置", "Read Current"));
    hand_btn_row1->addWidget(btn_hand_send);
    hand_btn_row1->addWidget(btn_hand_open);
    hand_btn_row1->addWidget(btn_hand_close);
    hand_btn_row1->addWidget(btn_hand_read);
    hand_layout->addLayout(hand_btn_row1, 2, 0, 1, 6);

    // Preset row
    auto* hand_btn_row2 = new QHBoxLayout();
    auto* btn_preset_light = new QPushButton(tr_ui("轻握 (400)", "Light (400)"));
    auto* btn_preset_medium = new QPushButton(tr_ui("中握 (600)", "Medium (600)"));
    auto* btn_preset_firm = new QPushButton(tr_ui("紧握 (800)", "Firm (800)"));
    auto* btn_preset_max = new QPushButton(tr_ui("最大 (1000)", "Max (1000)"));
    hand_btn_row2->addWidget(btn_preset_light);
    hand_btn_row2->addWidget(btn_preset_medium);
    hand_btn_row2->addWidget(btn_preset_firm);
    hand_btn_row2->addWidget(btn_preset_max);
    hand_layout->addLayout(hand_btn_row2, 3, 0, 1, 6);

    main_layout->addWidget(hand_group);

    // ================================================================
    // Section 3: Vision Grasp Trigger
    // ================================================================
    auto* grasp_group = new QGroupBox(
        tr_ui("视觉抓取 — 在彩色流画面上点击选择目标",
              "Vision Grasp — click on Color Stream to select target"));
    auto* grasp_layout = new QHBoxLayout(grasp_group);

    label_grasp_target_ = new QLabel(tr_ui("未选择目标", "No target selected"));
    label_grasp_target_->setStyleSheet("font-size: 12px; color: #aaa;");
    btn_grasp_ = new QPushButton(tr_ui("抓取选中目标", "Grasp Selected"));
    btn_grasp_->setEnabled(false);
    btn_grasp_->setMinimumHeight(36);
    btn_grasp_->setStyleSheet("font-size: 13px; font-weight: bold;");

    grasp_layout->addWidget(label_grasp_target_, 1);
    grasp_layout->addWidget(btn_grasp_);
    main_layout->addWidget(grasp_group);

    // ================================================================
    // Section 4: Grasp Offset Compensation
    // ================================================================
    auto* offset_group = new QGroupBox(
        tr_ui("抓取位置偏移补偿 (base frame, 米)",
              "Grasp Position Offset (base frame, meters)"));
    auto* offset_layout = new QGridLayout(offset_group);
    offset_layout->setColumnStretch(1, 1);
    offset_layout->setColumnStretch(3, 1);
    offset_layout->setColumnStretch(5, 1);

    const char* offset_labels[] = {"ΔX (m)", "ΔY (m)", "ΔZ (m)"};
    const char* offset_tips[] = {
        "正值→右偏, 负值→左偏\n(base +X direction)",
        "正值→前偏, 负值→后偏\n(base +Y direction)",
        "正值→上偏, 负值→下偏\n(base +Z direction)"
    };
    for (int i = 0; i < 3; i++) {
        auto* lbl = new QLabel(offset_labels[i]);
        lbl->setToolTip(QString::fromUtf8(offset_tips[i]));
        offset_layout->addWidget(lbl, 0, i * 2);
        spin_grasp_offset_[i] = new QDoubleSpinBox();
        spin_grasp_offset_[i]->setRange(-2.0, 2.0);
        spin_grasp_offset_[i]->setSingleStep(0.01);
        spin_grasp_offset_[i]->setDecimals(3);
        spin_grasp_offset_[i]->setValue(0.0);
        spin_grasp_offset_[i]->setToolTip(QString::fromUtf8(offset_tips[i]));
        offset_layout->addWidget(spin_grasp_offset_[i], 0, i * 2 + 1);
    }

    auto* offset_btn_row = new QHBoxLayout();
    auto* btn_apply_offset = new QPushButton(tr_ui("应用偏移 (实时生效)", "Apply Offset (live)"));
    btn_apply_offset->setStyleSheet("font-weight: bold; color: #00cc66;");
    auto* btn_reset_offset = new QPushButton(tr_ui("清零", "Reset"));
    btn_reset_offset->setFixedWidth(60);
    offset_btn_row->addWidget(btn_apply_offset, 1);
    offset_btn_row->addWidget(btn_reset_offset);
    offset_layout->addLayout(offset_btn_row, 1, 0, 1, 6);

    auto* offset_hint = new QLabel(tr_ui(
        "💡 机器人偏右→ΔX填负值  偏左→ΔX填正值  偏高→ΔZ填负值  每次调5mm",
        "💡 Robot goes right→ΔX negative  left→ΔX positive  high→ΔZ negative  adjust 5mm each time"));
    offset_hint->setStyleSheet("color: #888; font-size: 11px;");
    offset_hint->setWordWrap(true);
    offset_layout->addWidget(offset_hint, 2, 0, 1, 6);

    main_layout->addWidget(offset_group);

    // ================================================================
    // Hand-Eye Calibration Workflow
    // ================================================================
    auto* calib_wf_group = new QGroupBox(
        tr_ui("手眼标定", "Hand-Eye Calibration"));
    auto* calib_wf_layout = new QVBoxLayout(calib_wf_group);

    auto* calib_wf_hint = new QLabel(tr_ui(
        "1) 工作面上画标记点, 保持固定\n"
        "2) 机器人指尖对准标记点 → 点\"触碰标记\"\n"
        "3) 移动机器人使相机能看到该点 → 在画面上点击标记位置\n"
        "4) 点\"拍照采点\" → 重复3-4至少4次不同视角\n"
        "5) 点\"求解\" → 点\"保存\"",
        "1) Draw a dot on workspace, keep it FIXED\n"
        "2) Touch gripper tip to dot → click \"Touch\"\n"
        "3) Move robot so camera sees dot → click dot on video\n"
        "4) Click \"Collect\" → repeat 3-4 at least 4 times\n"
        "5) Click \"Solve\" → Click \"Save\""));
    calib_wf_hint->setStyleSheet("color: #888; font-size: 11px;");
    calib_wf_hint->setWordWrap(true);
    calib_wf_layout->addWidget(calib_wf_hint);

    auto* calib_btn_row = new QHBoxLayout();
    auto* btn_touch = new QPushButton(tr_ui("触碰标记", "Touch Dot"));
    btn_touch->setStyleSheet("font-weight: bold; color: #ff6600;");
    auto* btn_collect = new QPushButton(tr_ui("拍照采点", "Collect"));
    btn_collect->setStyleSheet("font-weight: bold;");
    auto* btn_solve = new QPushButton(tr_ui("求解", "Solve"));
    auto* btn_save_calib = new QPushButton(tr_ui("保存标定", "Save Calibration"));
    btn_save_calib->setStyleSheet("font-weight: bold; color: #00cc66;");
    auto* btn_clear = new QPushButton(tr_ui("清除", "Clear"));
    btn_clear->setFixedWidth(60);

    calib_btn_row->addWidget(btn_touch, 1);
    calib_btn_row->addWidget(btn_collect, 1);
    calib_btn_row->addWidget(btn_solve, 1);
    calib_btn_row->addWidget(btn_save_calib, 1);
    calib_btn_row->addWidget(btn_clear);
    calib_wf_layout->addLayout(calib_btn_row);

    auto* label_calib_wf_status = new QLabel(
        tr_ui("样本: 0 | 请先触碰标记", "Samples: 0 | Touch dot first"));
    label_calib_wf_status->setStyleSheet("font-size: 12px;");
    calib_wf_layout->addWidget(label_calib_wf_status);

    main_layout->addWidget(calib_wf_group);

    // --- Calibration workflow connections ---
    connect(btn_touch, &QPushButton::clicked, this, [=]() {
        btn_touch->setEnabled(false);
        label_calib_wf_status->setText(tr_ui("触碰中...", "Touching..."));
        node_->call_hand_eye_calibrate("touch", -1, -1,
            [=](bool ok, const std::string& msg, int, double, double) {
                QMetaObject::invokeMethod(this, [=]() {
                    btn_touch->setEnabled(true);
                    label_calib_wf_status->setText(QString::fromStdString(msg));
                    label_calib_status_->setText(QString::fromStdString(msg));
                });
            });
    });

    connect(btn_collect, &QPushButton::clicked, this, [=]() {
        if (!has_video_selection_) {
            label_calib_wf_status->setText(
                tr_ui("请先在画面中点击标记点!", "Click the dot on the video stream first!"));
            return;
        }
        QPointF pt = video_selected_point_;
        btn_collect->setEnabled(false);
        label_calib_wf_status->setText(
            tr_ui("采集中...", "Collecting...") +
            QString(" (%1,%2)").arg((int)pt.x()).arg((int)pt.y()));
        node_->call_hand_eye_calibrate("collect", pt.x(), pt.y(),
            [=](bool ok, const std::string& msg, int count, double, double) {
                QMetaObject::invokeMethod(this, [=]() {
                    btn_collect->setEnabled(true);
                    QString st = QString(tr_ui("样本: %1 | ", "Samples: %1 | ")).arg(count) +
                                 QString::fromStdString(msg);
                    label_calib_wf_status->setText(st);
                    label_calib_status_->setText(QString::fromStdString(msg));
                });
            });
    });

    connect(btn_solve, &QPushButton::clicked, this, [=]() {
        btn_solve->setEnabled(false);
        label_calib_wf_status->setText(tr_ui("求解中...", "Solving..."));
        node_->call_hand_eye_calibrate("solve", -1, -1,
            [=](bool ok, const std::string& msg, int count,
                double rot_err, double pos_err) {
                QMetaObject::invokeMethod(this, [=]() {
                    btn_solve->setEnabled(true);
                    (void)rot_err;
                    QString st = QString(tr_ui("样本: %1 | err=%2mm | ", "Samples: %1 | err=%2mm | "))
                                     .arg(count).arg(pos_err, 0, 'f', 1) +
                                 (ok ? tr_ui("通过", "PASS") : tr_ui("失败", "FAIL"));
                    label_calib_wf_status->setText(st);
                    label_calib_status_->setText(QString::fromStdString(msg));
                });
            });
    });

    connect(btn_save_calib, &QPushButton::clicked, this, [=]() {
        btn_save_calib->setEnabled(false);
        node_->call_hand_eye_calibrate("save", -1, -1,
            [=](bool ok, const std::string& msg, int, double, double) {
                QMetaObject::invokeMethod(this, [=]() {
                    btn_save_calib->setEnabled(true);
                    label_calib_wf_status->setText(QString::fromStdString(msg));
                    label_calib_status_->setText(QString::fromStdString(msg));
                });
            });
    });

    connect(btn_clear, &QPushButton::clicked, this, [=]() {
        node_->call_hand_eye_calibrate("clear", -1, -1,
            [=](bool, const std::string& msg, int count, double, double) {
                QMetaObject::invokeMethod(this, [=]() {
                    QString st = QString(tr_ui("样本: %1 | ", "Samples: %1 | ")).arg(count) +
                                 QString::fromStdString(msg);
                    label_calib_wf_status->setText(st);
                });
            });
    });

    // Wire up: grasp button → call trigger service using stored selection
    connect(btn_grasp_, &QPushButton::clicked, this, [this]() {
        if (!has_video_selection_) {
            label_calib_status_->setText(
                tr_ui("请先在彩色流画面上点击选择目标", "Click on Color Stream to select a target first"));
            return;
        }
        QPointF pt = video_selected_point_;
        btn_grasp_->setEnabled(false);
        label_calib_status_->setText(
            tr_ui("抓取执行中...", "Grasping...") +
            QString(" (%1, %2)").arg((int)pt.x()).arg((int)pt.y()));

        node_->call_trigger_grasp(pt.x(), pt.y(),
            [this](bool success, const std::string& msg) {
                QMetaObject::invokeMethod(this, [this, success, msg]() {
                    btn_grasp_->setEnabled(true);
                    QString qmsg = QString::fromStdString(msg);
                    if (success) {
                        label_calib_status_->setText(tr_ui("抓取成功: ", "Grasp OK: ") + qmsg);
                    } else {
                        label_calib_status_->setText(tr_ui("抓取失败: ", "Grasp failed: ") + qmsg);
                    }
                });
            });
    });

    // Status label
    label_calib_status_ = new QLabel(
        tr_ui("就绪 — 调整参数后点击对应按钮生效",
              "Ready — adjust values and click buttons to apply"));
    main_layout->addWidget(label_calib_status_);
    main_layout->addStretch();

    // ================================================================
    // Connections
    // ================================================================

    // --- Apply Offset (live, via ROS set_parameters) ---
    connect(btn_apply_offset, &QPushButton::clicked, this, [this]() {
        double ox = spin_grasp_offset_[0]->value();
        double oy = spin_grasp_offset_[1]->value();
        double oz = spin_grasp_offset_[2]->value();
        node_->set_grasp_offset(ox, oy, oz);
        label_calib_status_->setText(
            tr_ui("偏移已应用: ", "Offset applied: ") +
            QString("ΔX=%1 ΔY=%2 ΔZ=%3 m")
                .arg(ox, 0, 'f', 3).arg(oy, 0, 'f', 3).arg(oz, 0, 'f', 3));
    });

    // --- Reset Offset ---
    connect(btn_reset_offset, &QPushButton::clicked, this, [this]() {
        for (int i = 0; i < 3; i++) spin_grasp_offset_[i]->setValue(0.0);
        node_->set_grasp_offset(0.0, 0.0, 0.0);
        label_calib_status_->setText(tr_ui("偏移已清零", "Offset reset to zero"));
    });

    // --- Apply TF ---
    connect(btn_apply_tf, &QPushButton::clicked, this, [this]() {
        std::string ns = combo_calib_camera_->currentText().toStdString();
        std::string sn = ns;
        if (!sn.empty() && sn[0] == '/') sn = sn.substr(1);
        node_->publish_hand_eye_tf(
            spin_calib_tf_[0]->value(), spin_calib_tf_[1]->value(),
            spin_calib_tf_[2]->value(), spin_calib_tf_[3]->value(),
            spin_calib_tf_[4]->value(), spin_calib_tf_[5]->value(),
            sn + "_link");
        label_calib_status_->setText(
            tr_ui("TF 已发布 → ", "TF published → ") + QString::fromStdString(sn + "_link"));
    });

    // --- Save Config ---
    connect(btn_save_tf, &QPushButton::clicked, this, [this]() {
        QString default_path = calib_yaml_path_.isEmpty() ? "hand_eye.yaml" : calib_yaml_path_;
        QString path = QFileDialog::getSaveFileName(
            this, tr_ui("保存标定配置", "Save Calibration"),
            default_path, "YAML (*.yaml)");
        if (path.isEmpty()) return;

        // Convert RPY (degrees) → quaternion for hand_eye.yaml format
        double rx = spin_calib_tf_[3]->value() * M_PI / 180.0;
        double ry = spin_calib_tf_[4]->value() * M_PI / 180.0;
        double rz = spin_calib_tf_[5]->value() * M_PI / 180.0;
        double cy = std::cos(rz * 0.5), sy = std::sin(rz * 0.5);
        double cp = std::cos(ry * 0.5), sp = std::sin(ry * 0.5);
        double cr = std::cos(rx * 0.5), sr = std::sin(rx * 0.5);
        double qw = cr*cp*cy + sr*sp*sy;
        double qx = sr*cp*cy - cr*sp*sy;
        double qy = cr*sp*cy + sr*cp*sy;
        double qz = cr*cp*sy - sr*sp*cy;

        std::ofstream f(path.toStdString());
        f << "# Hand-eye calibration: T_end_camera (L_base_link → cam_305_link)\n";
        f << "# Generated by UI calibration tool\n";
        f << "hand_eye:\n";
        f << "  x: "  << spin_calib_tf_[0]->value() << "\n";
        f << "  y: "  << spin_calib_tf_[1]->value() << "\n";
        f << "  z: "  << spin_calib_tf_[2]->value() << "\n";
        f << "  qx: " << qx << "\n";
        f << "  qy: " << qy << "\n";
        f << "  qz: " << qz << "\n";
        f << "  qw: " << qw << "\n";
        f << "\ngrasp:\n";
        f << "  pre_grasp_height: 0.15\n";
        f << "  grasp_z_offset: 0.02\n";
        f << "  lift_height: 0.15\n";
        f << "  grasp_rx: 3.14159\n";
        f << "  grasp_ry: 0.0\n";
        f << "  grasp_rz: 0.0\n";
        f << "  move_speed: 0.20\n";
        f << "  approach_speed: 0.05\n";
        f << "  lift_speed: 0.10\n";
        f << "  move_accel: 0.5\n";
        f << "\nhand:\n";
        f << "  open:  [0, 0, 0, 0, 0, 0]\n";
        f << "  close: [";
        for (int i = 0; i < 6; i++) { if (i) f << ", "; f << spin_calib_hand_[i]->value(); }
        f << "]\n";
        f << "\nplace:\n";
        f << "  x: 0.30\n  y: -0.30\n  z: 0.25\n";
        f << "\ngrasp_offset:\n";
        f << "  x: " << spin_grasp_offset_[0]->value() << "\n";
        f << "  y: " << spin_grasp_offset_[1]->value() << "\n";
        f << "  z: " << spin_grasp_offset_[2]->value() << "\n";
        f << "\ndetection:\n";
        f << "  model_path: \"yolov8n.pt\"\n";
        f << "  confidence_threshold: 0.5\n";
        f << "  depth_roi_half: 10\n";
        f << "  min_depth_mm: 100.0\n";
        f << "  max_depth_mm: 2000.0\n";
        f.close();
        calib_yaml_path_ = path;
        label_calib_status_->setText(tr_ui("配置已保存: ", "Config saved: ") + path);
    });

    // --- Send hand position ---
    auto send_hand = [this]() {
        std::array<int, 6> pos;
        for (int i = 0; i < 6; i++) pos[i] = spin_calib_hand_[i]->value();
        node_->call_lhand_set_all_position(pos);
        node_->call_lhand_move(0);
        std::ostringstream ss;
        ss << "Hand → [";
        for (int i = 0; i < 6; i++) { if (i) ss << ","; ss << pos[i]; }
        ss << "]";
        label_calib_status_->setText(QString::fromStdString(ss.str()));
    };

    connect(btn_hand_send, &QPushButton::clicked, this, send_hand);

    // --- Open all ---
    connect(btn_hand_open, &QPushButton::clicked, this, [this, send_hand]() {
        for (int i = 0; i < 6; i++) spin_calib_hand_[i]->setValue(0);
        send_hand();
    });

    // --- Grasp test (use current spinbox values) ---
    connect(btn_hand_close, &QPushButton::clicked, this, send_hand);

    // --- Presets ---
    auto set_preset = [this, send_hand](int val) {
        for (int i = 0; i < 6; i++) spin_calib_hand_[i]->setValue(val);
        send_hand();
    };
    connect(btn_preset_light,  &QPushButton::clicked, this, [set_preset]() { set_preset(400); });
    connect(btn_preset_medium, &QPushButton::clicked, this, [set_preset]() { set_preset(600); });
    connect(btn_preset_firm,   &QPushButton::clicked, this, [set_preset]() { set_preset(800); });
    connect(btn_preset_max,    &QPushButton::clicked, this, [set_preset]() { set_preset(1000); });

    // --- Read current positions ---
    connect(btn_hand_read, &QPushButton::clicked, this, [this]() {
        label_calib_status_->setText(tr_ui("读取中...", "Reading..."));
        for (int i = 0; i < 6; i++) {
            int joint_id = i + 1;
            node_->call_lhand_get_position(joint_id, [this, i](int pos) {
                QMetaObject::invokeMethod(this, [this, i, pos]() {
                    spin_calib_hand_[i]->setValue(pos);
                    if (i == 5) {
                        label_calib_status_->setText(
                            tr_ui("已读取当前位置", "Current positions loaded"));
                    }
                });
            });
        }
    });

    // --- File path: Browse ---
    connect(btn_browse, &QPushButton::clicked, this, [this, edit_yaml_path]() {
        QString f = QFileDialog::getOpenFileName(
            this, tr_ui("选择标定配置文件", "Select Calibration YAML"),
            edit_yaml_path->text(), "YAML (*.yaml *.yml)");
        if (!f.isEmpty()) edit_yaml_path->setText(f);
    });

    // --- File path: Load ---
    connect(btn_load_yaml, &QPushButton::clicked, this, [this, edit_yaml_path]() {
        loadCalibYaml(edit_yaml_path->text());
    });

    // --- Auto-load on startup ---
    if (!default_yaml_path.isEmpty())
        QMetaObject::invokeMethod(this, [this, default_yaml_path]() {
            loadCalibYaml(default_yaml_path);
        }, Qt::QueuedConnection);

    return widget;
}
