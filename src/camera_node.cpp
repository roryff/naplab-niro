/**
 * camera_node.cpp
 *
 * ROS 2 node — receives H.265 MPEG-TS over UDP multicast, hardware-decodes
 * with nvv4l2decoder, hardware-encodes to H.264 with nvv4l2h264enc (all in
 * NVMM, zero CPU copy), and publishes as foxglove_msgs/msg/CompressedVideo
 * (format = "h264") for direct playback in Foxglove Studio.
 *
 * Foxglove Desktop (Electron/Chromium) does not support H.265 on Linux —
 * H.265 WebCodecs decode is only available in Safari on macOS/iOS.
 *
 * Parameters
 * ----------
 * multicast_ip    Multicast group IP     (default: "239.10.0.1")
 * port            UDP port               (default: 10030)
 * topic           Output ROS topic       (default: "image_compressed")
 * frame_id        TF frame id            (default: "camera")
 * multicast_iface Network interface      (default: "enP2p1s0")
 */

#include <chrono>
#include <string>
#include <thread>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <foxglove_msgs/msg/compressed_video.hpp>

using Clock = std::chrono::steady_clock;
using ms    = std::chrono::duration<double, std::milli>;

class CameraNode : public rclcpp::Node
{
public:
    explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("camera_node", options)
    {
        setenv("EGL_PLATFORM", "surfaceless", 0);
        if (!gst_is_initialized()) {
            gst_init(nullptr, nullptr);
        }

        // Lock all memory now — prevents page-fault stalls mid-frame in both
        // standalone and composable-component execution modes.
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            RCLCPP_WARN(get_logger(), "mlockall failed: %s", strerror(errno));
        }

        ip_     = declare_parameter<std::string>("multicast_ip",    "239.10.0.1");
        port_   = declare_parameter<int>        ("port",            10030);
        topic_  = declare_parameter<std::string>("topic",           "image_compressed");
        frame_  = declare_parameter<std::string>("frame_id",        "camera");
        iface_  = declare_parameter<std::string>("multicast_iface", "enP2p1s0");

        pub_ = create_publisher<foxglove_msgs::msg::CompressedVideo>(topic_, 10);

        // H.265 MPEG-TS → HW decode → HW H.264 encode → Annex-B byte-stream.
        // All stays in NVMM (zero CPU copy). Foxglove Desktop (Electron/Chromium)
        // supports H.264 universally; H.265 is only decoded on Safari/macOS.
        std::string pipeline_str =
            "udpsrc uri=udp://" + ip_ + ":" + std::to_string(port_) +
            " buffer-size=4194304 multicast-iface=" + iface_ +
            " ! tsdemux latency=0"
            " ! queue max-size-buffers=1 leaky=2"
            " ! h265parse"
            " ! nvv4l2decoder disable-dpb=true low-latency-mode=true"
            " ! nvv4l2h264enc idrinterval=30 insert-sps-pps=true"
            " ! h264parse config-interval=-1"
            " ! video/x-h264,stream-format=byte-stream,alignment=au"
            " ! appsink name=sink sync=false max-buffers=2 drop=true";

        RCLCPP_INFO(get_logger(), "Pipeline: %s", pipeline_str.c_str());

        GError* err = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &err);
        if (err) {
            RCLCPP_FATAL(get_logger(), "GStreamer parse error: %s", err->message);
            g_error_free(err);
            return;
        }

        sink_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "sink"));

        GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            RCLCPP_FATAL(get_logger(), "Failed to set pipeline to PLAYING");
            return;
        }

        RCLCPP_INFO(get_logger(), "Pipeline PLAYING — publishing H.265 bitstream");
        pull_thread_ = std::thread(&CameraNode::pull_loop, this);
    }

    ~CameraNode()
    {
        running_ = false;
        if (sink_) {
            gst_app_sink_set_emit_signals(sink_, FALSE);
            gst_element_send_event(pipeline_, gst_event_new_eos());
        }
        if (pull_thread_.joinable()) pull_thread_.join();
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }
    }

private:
    void pull_loop()
    {
        // SCHED_FIFO priority 65: camera publish thread.
        // Above ADR wheel-speed sender (60) — low-latency video matters more
        // than the 10 Hz feedback signal. Below control nodes (70+).
        struct sched_param sp{};
        sp.sched_priority = 65;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
            RCLCPP_WARN(get_logger(),
                "pull_loop: SCHED_FIFO failed (not root / no CAP_SYS_NICE).");
        }

        int    frame_count = 0;
        double sum_pull = 0, sum_pub = 0;
        double max_pull = 0, max_pub = 0;
        auto   report_t = Clock::now();

        while (running_ && rclcpp::ok()) {
            auto t0 = Clock::now();
            GstSample* sample = gst_app_sink_try_pull_sample(sink_, 100 * GST_MSECOND);
            auto t1 = Clock::now();
            double pull_ms = ms(t1 - t0).count();

            if (!sample) continue;

            GstBuffer* buf = gst_sample_get_buffer(sample);
            GstMapInfo map;
            if (gst_buffer_map(buf, &map, GST_MAP_READ)) {
                auto ros_msg = std::make_unique<foxglove_msgs::msg::CompressedVideo>();
                ros_msg->timestamp = get_clock()->now();
                ros_msg->frame_id  = frame_;
                ros_msg->format    = "h264";
                ros_msg->data.assign(map.data, map.data + map.size);
                gst_buffer_unmap(buf, &map);

                pub_->publish(std::move(ros_msg));
                auto t2 = Clock::now();
                double pub_ms = ms(t2 - t1).count();

                ++frame_count;
                sum_pull += pull_ms; max_pull = std::max(max_pull, pull_ms);
                sum_pub  += pub_ms;  max_pub  = std::max(max_pub,  pub_ms);

                if (pull_ms > 100 || pub_ms > 50) {
                    RCLCPP_WARN(get_logger(), "SLOW  pull=%.1fms pub=%.1fms",
                        pull_ms, pub_ms);
                }

                double elapsed = ms(t2 - report_t).count();
                if (elapsed > 5000.0) {
                    frame_count = 0;
                    sum_pull = sum_pub = 0;
                    max_pull = max_pub = 0;
                    report_t = t2;
                }
            }

            gst_sample_unref(sample);
        }
    }

    std::string ip_, topic_, frame_, iface_;
    int port_;

    rclcpp::Publisher<foxglove_msgs::msg::CompressedVideo>::SharedPtr pub_;
    GstElement*  pipeline_ = nullptr;
    GstAppSink*  sink_     = nullptr;
    std::thread  pull_thread_;
    std::atomic<bool> running_{true};
};

// Register as a composable node component
RCLCPP_COMPONENTS_REGISTER_NODE(CameraNode)

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // mlockall already called in constructor, but call again here in case
    // main thread allocates before the node is constructed.
    mlockall(MCL_CURRENT | MCL_FUTURE);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
