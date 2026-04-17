#include "ui_app/app_window.hpp"
#include "ui_app/i18n_manager.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QPushButton>
#include <QMessageBox>
#include <QFileDialog>
#include <fstream>
#include <sstream>

QWidget* AppWindow::createCalibrationTab() {
    auto* widget = new QWidget();
    auto* main_layout = new QVBoxLayout(widget);
    main_layout->setSpacing(8);

    // ================================================================
    // Section 1: Hand-Eye TF Calibration (link_6 → camera_link)
    // ================================================================
    auto* tf_group = new QGroupBox(
        tr_ui("手眼标定 TF (link_6 → camera_link)",
              "Hand-Eye TF (link_6 → camera_link)"));
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

    // --- Apply TF ---
    connect(btn_apply_tf, &QPushButton::clicked, this, [this]() {
        node_->publish_hand_eye_tf(
            spin_calib_tf_[0]->value(), spin_calib_tf_[1]->value(),
            spin_calib_tf_[2]->value(), spin_calib_tf_[3]->value(),
            spin_calib_tf_[4]->value(), spin_calib_tf_[5]->value());
        label_calib_status_->setText(tr_ui("TF 已发布", "TF published"));
    });

    // --- Save Config ---
    connect(btn_save_tf, &QPushButton::clicked, this, [this]() {
        QString path = QFileDialog::getSaveFileName(
            this, tr_ui("保存标定配置", "Save Calibration"),
            "hand_eye_calibration.yaml", "YAML (*.yaml)");
        if (path.isEmpty()) return;
        std::ofstream f(path.toStdString());
        f << "# Hand-eye calibration config (auto-generated)\n";
        f << "hand_eye:\n";
        f << "  x: " << spin_calib_tf_[0]->value() << "\n";
        f << "  y: " << spin_calib_tf_[1]->value() << "\n";
        f << "  z: " << spin_calib_tf_[2]->value() << "\n";
        f << "  rx_deg: " << spin_calib_tf_[3]->value() << "\n";
        f << "  ry_deg: " << spin_calib_tf_[4]->value() << "\n";
        f << "  rz_deg: " << spin_calib_tf_[5]->value() << "\n";
        f << "\nhand:\n";
        f << "  close: [";
        for (int i = 0; i < 6; i++) {
            if (i) f << ", ";
            f << spin_calib_hand_[i]->value();
        }
        f << "]\n";
        f.close();
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

    return widget;
}
