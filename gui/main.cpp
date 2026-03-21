/**
 * RADAR — Operator GUI
 * Qt 6 + rclcpp single-file application.
 *
 * Subscribes to:
 *   /radar/camera/image_raw      (sensor_msgs/Image)
 *   /radar/pulseox/vitals        (std_msgs/Float32MultiArray  [hr, spo2])
 *
 * Publishes to:
 *   /cmd_vel                     (geometry_msgs/Twist)
 *   /radar/pan_tilt/cmd          (std_msgs/Float32MultiArray  [pan_deg, tilt_deg])
 *
 * Build:  see CMakeLists.txt in this directory
 * Run:    ./radar_gui
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QLabel>
#include <QSlider>
#include <QGroupBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QTimer>
#include <QThread>
#include <QImage>
#include <QPixmap>
#include <QStatusBar>
#include <QProgressBar>
#include <QPushButton>
#include <QFont>
#include <QColor>
#include <QPalette>
#include <QString>
#include <QMutex>
#include <QMutexLocker>

#include <atomic>
#include <memory>
#include <cstring>

// ---------------------------------------------------------------------------
// ROS 2 node — spun in a background QThread
// ---------------------------------------------------------------------------

class RadarRosNode : public QObject, public rclcpp::Node {
    Q_OBJECT
public:
    RadarRosNode()
      : QObject(), rclcpp::Node("radar_gui")
    {
        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/radar/camera/image_raw", 10,
            [this](sensor_msgs::msg::Image::SharedPtr msg) {
                onImage(msg);
            });

        sub_vitals_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/radar/pulseox/vitals", 10,
            [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() >= 2)
                    emit vitalsReceived(msg->data[0], msg->data[1]);
            });

        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        pub_pan_tilt_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/radar/pan_tilt/cmd", 10);

        RCLCPP_INFO(get_logger(), "RadarRosNode started");
    }

    void publishCmdVel(double linear_x, double angular_z) {
        geometry_msgs::msg::Twist msg;
        msg.linear.x  = linear_x;
        msg.angular.z = angular_z;
        pub_cmd_vel_->publish(msg);
    }

    void publishPanTilt(float pan_deg, float tilt_deg) {
        std_msgs::msg::Float32MultiArray msg;
        msg.data = {pan_deg, tilt_deg};
        pub_pan_tilt_->publish(msg);
    }

signals:
    void imageReceived(QImage img);
    void vitalsReceived(float hr, float spo2);

private:
    void onImage(sensor_msgs::msg::Image::SharedPtr msg) {
        if (msg->encoding != "bgr8" && msg->encoding != "rgb8") {
            return;
        }
        QImage::Format fmt = (msg->encoding == "bgr8")
            ? QImage::Format_BGR888
            : QImage::Format_RGB888;

        QImage img(msg->data.data(),
                   static_cast<int>(msg->width),
                   static_cast<int>(msg->height),
                   static_cast<int>(msg->step),
                   fmt);

        emit imageReceived(img.copy());  // copy before msg goes out of scope
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr           sub_image_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr  sub_vitals_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr            pub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr     pub_pan_tilt_;
};


// ---------------------------------------------------------------------------
// Background spin thread
// ---------------------------------------------------------------------------

class RosThread : public QThread {
    Q_OBJECT
public:
    explicit RosThread(std::shared_ptr<RadarRosNode> node, QObject* parent = nullptr)
      : QThread(parent), node_(std::move(node)), running_(true) {}

    void stop() {
        running_ = false;
    }

protected:
    void run() override {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(node_);
        while (running_ && rclcpp::ok()) {
            exec.spin_some(std::chrono::milliseconds(10));
        }
    }

private:
    std::shared_ptr<RadarRosNode> node_;
    std::atomic<bool> running_;
};


// ---------------------------------------------------------------------------
// Main window
// ---------------------------------------------------------------------------

class RadarWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit RadarWindow(std::shared_ptr<RadarRosNode> ros_node, QWidget* parent = nullptr)
      : QMainWindow(parent), ros_node_(std::move(ros_node))
    {
        setWindowTitle("RADAR — Remote Autonomous Doctor Assistance Robot");
        resize(1200, 700);
        setStyleSheet("background-color: #1e1e2e; color: #cdd6f4;");

        auto* central = new QWidget(this);
        setCentralWidget(central);
        auto* root = new QHBoxLayout(central);
        root->setSpacing(10);
        root->setContentsMargins(10, 10, 10, 10);

        // ---- Left: Video feed ----
        auto* video_group = new QGroupBox("Live Feed", this);
        video_group->setStyleSheet(groupStyle());
        auto* video_layout = new QVBoxLayout(video_group);

        video_label_ = new QLabel("Waiting for camera...", this);
        video_label_->setMinimumSize(640, 480);
        video_label_->setAlignment(Qt::AlignCenter);
        video_label_->setStyleSheet(
            "background-color: #11111b; color: #6c7086; "
            "border: 1px solid #313244;"
        );
        video_layout->addWidget(video_label_);
        root->addWidget(video_group, 3);

        // ---- Right: Controls + Vitals ----
        auto* right = new QVBoxLayout();
        right->setSpacing(10);

        // Vitals panel
        auto* vitals_group = new QGroupBox("Patient Vitals", this);
        vitals_group->setStyleSheet(groupStyle());
        auto* vitals_layout = new QGridLayout(vitals_group);

        auto addVitalsRow = [&](int row, const QString& label,
                                QLabel*& value_label, const QString& unit,
                                QProgressBar*& bar, int lo, int hi)
        {
            auto* lbl = new QLabel(label, this);
            lbl->setStyleSheet("color: #a6adc8; font-size: 13px;");
            vitals_layout->addWidget(lbl, row, 0);

            value_label = new QLabel("--", this);
            value_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            value_label->setStyleSheet(
                "color: #cdd6f4; font-size: 22px; font-weight: bold;"
            );
            vitals_layout->addWidget(value_label, row, 1);

            auto* unit_lbl = new QLabel(unit, this);
            unit_lbl->setStyleSheet("color: #6c7086; font-size: 12px;");
            vitals_layout->addWidget(unit_lbl, row, 2);

            bar = new QProgressBar(this);
            bar->setRange(lo, hi);
            bar->setTextVisible(false);
            bar->setFixedHeight(8);
            bar->setStyleSheet(
                "QProgressBar { background: #313244; border-radius: 4px; }"
                "QProgressBar::chunk { background: #89dceb; border-radius: 4px; }"
            );
            vitals_layout->addWidget(bar, row + 1, 0, 1, 3);
        };

        addVitalsRow(0, "Heart Rate",   hr_label_,   "BPM", hr_bar_,   40, 180);
        addVitalsRow(2, "SpO2",         spo2_label_, "%",   spo2_bar_, 70, 100);

        right->addWidget(vitals_group);

        // Pan-tilt sliders
        auto* pantilt_group = new QGroupBox("Camera Orientation", this);
        pantilt_group->setStyleSheet(groupStyle());
        auto* pt_layout = new QGridLayout(pantilt_group);

        auto makeSlider = [&](int min, int max, int val, Qt::Orientation orient) {
            auto* s = new QSlider(orient, this);
            s->setRange(min, max);
            s->setValue(val);
            s->setStyleSheet(
                "QSlider::groove:horizontal { height: 6px; background: #313244; border-radius: 3px; }"
                "QSlider::handle:horizontal { width: 16px; height: 16px; margin: -5px 0; "
                "  background: #89b4fa; border-radius: 8px; }"
                "QSlider::sub-page:horizontal { background: #89b4fa; border-radius: 3px; }"
            );
            return s;
        };

        pan_label_  = new QLabel("Pan:  0°",  this);
        tilt_label_ = new QLabel("Tilt: 0°", this);
        pan_label_->setStyleSheet("color: #cdd6f4;");
        tilt_label_->setStyleSheet("color: #cdd6f4;");

        pan_slider_  = makeSlider(-80, 80, 0, Qt::Horizontal);
        tilt_slider_ = makeSlider(-45, 45, 0, Qt::Horizontal);

        pt_layout->addWidget(pan_label_,  0, 0);
        pt_layout->addWidget(pan_slider_, 0, 1);
        pt_layout->addWidget(tilt_label_, 1, 0);
        pt_layout->addWidget(tilt_slider_,1, 1);

        right->addWidget(pantilt_group);

        // Emergency stop
        auto* estop_btn = new QPushButton("EMERGENCY STOP", this);
        estop_btn->setStyleSheet(
            "QPushButton { background-color: #f38ba8; color: #1e1e2e; "
            "  font-weight: bold; font-size: 14px; padding: 12px; border-radius: 6px; }"
            "QPushButton:pressed { background-color: #e06c75; }"
        );
        right->addWidget(estop_btn);
        right->addStretch();

        root->addLayout(right, 1);

        // ---- Status bar ----
        status_label_ = new QLabel("Connecting to ROS 2...", this);
        status_label_->setStyleSheet("color: #fab387;");
        statusBar()->addPermanentWidget(status_label_);
        statusBar()->setStyleSheet("background-color: #181825; color: #a6adc8;");

        // ---- Connections ----
        connect(ros_node_.get(), &RadarRosNode::imageReceived,
                this, &RadarWindow::onImage, Qt::QueuedConnection);

        connect(ros_node_.get(), &RadarRosNode::vitalsReceived,
                this, &RadarWindow::onVitals, Qt::QueuedConnection);

        connect(pan_slider_,  &QSlider::valueChanged, this, &RadarWindow::onPanTiltChanged);
        connect(tilt_slider_, &QSlider::valueChanged, this, &RadarWindow::onPanTiltChanged);

        connect(estop_btn, &QPushButton::clicked, this, [this]() {
            ros_node_->publishCmdVel(0.0, 0.0);
            status_label_->setText("EMERGENCY STOP sent");
            status_label_->setStyleSheet("color: #f38ba8; font-weight: bold;");
        });

        // Heartbeat timer — marks ROS as alive
        auto* hb = new QTimer(this);
        connect(hb, &QTimer::timeout, this, [this]() {
            if (rclcpp::ok()) {
                status_label_->setText("Connected");
                status_label_->setStyleSheet("color: #a6e3a1;");
            } else {
                status_label_->setText("ROS 2 offline");
                status_label_->setStyleSheet("color: #f38ba8;");
            }
        });
        hb->start(1000);
    }

private slots:
    void onImage(QImage img) {
        QPixmap px = QPixmap::fromImage(img).scaled(
            video_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation
        );
        video_label_->setPixmap(px);
    }

    void onVitals(float hr, float spo2) {
        hr_label_->setText(QString::number(static_cast<int>(hr)));
        hr_bar_->setValue(static_cast<int>(hr));

        spo2_label_->setText(QString::number(spo2, 'f', 1));
        spo2_bar_->setValue(static_cast<int>(spo2));

        // Color-code SpO2 severity
        QString spo2_style;
        if (spo2 >= 95.0f)
            spo2_style = "color: #a6e3a1; font-size: 22px; font-weight: bold;";
        else if (spo2 >= 90.0f)
            spo2_style = "color: #f9e2af; font-size: 22px; font-weight: bold;";
        else
            spo2_style = "color: #f38ba8; font-size: 22px; font-weight: bold;";
        spo2_label_->setStyleSheet(spo2_style);
    }

    void onPanTiltChanged() {
        int pan  = pan_slider_->value();
        int tilt = tilt_slider_->value();
        pan_label_->setText(QString("Pan:  %1°").arg(pan));
        tilt_label_->setText(QString("Tilt: %1°").arg(tilt));
        ros_node_->publishPanTilt(static_cast<float>(pan), static_cast<float>(tilt));
    }

private:
    static QString groupStyle() {
        return "QGroupBox { border: 1px solid #313244; border-radius: 6px; "
               "  margin-top: 8px; color: #a6adc8; font-size: 12px; }"
               "QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }";
    }

    std::shared_ptr<RadarRosNode> ros_node_;

    QLabel*       video_label_  = nullptr;
    QLabel*       hr_label_     = nullptr;
    QLabel*       spo2_label_   = nullptr;
    QProgressBar* hr_bar_       = nullptr;
    QProgressBar* spo2_bar_     = nullptr;
    QLabel*       pan_label_    = nullptr;
    QLabel*       tilt_label_   = nullptr;
    QSlider*      pan_slider_   = nullptr;
    QSlider*      tilt_slider_  = nullptr;
    QLabel*       status_label_ = nullptr;
};


// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

#include "main.moc"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);
    app.setApplicationName("RADAR GUI");

    auto ros_node = std::make_shared<RadarRosNode>();

    RosThread ros_thread(ros_node);
    ros_thread.start();

    RadarWindow window(ros_node);
    window.show();

    int ret = app.exec();

    ros_thread.stop();
    ros_thread.wait();
    rclcpp::shutdown();

    return ret;
}
