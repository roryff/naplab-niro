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
#include <algorithm>
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
        frame_timeout_ms_    = declare_parameter<double>("frame_timeout_ms", 2000.0);
        restart_backoff_ms_  = declare_parameter<double>("restart_backoff_ms", 250.0);
        encode_bitrate_bps_  = declare_parameter<int>("encode_bitrate_bps", 8000000);

        pub_ = create_publisher<foxglove_msgs::msg::CompressedVideo>(topic_, 10);

        // H.265 MPEG-TS → HW decode → HW H.264 encode → Annex-B byte-stream.
        // All stays in NVMM (zero CPU copy). Foxglove Desktop (Electron/Chromium)
        // supports H.264 universally; H.265 is only decoded on Safari/macOS.
        std::string pipeline_str =
            "udpsrc uri=udp://" + ip_ + ":" + std::to_string(port_) +
            " buffer-size=8388608 timeout=2000000000 multicast-iface=" + iface_ +
            " ! tsdemux latency=0"
            " ! queue max-size-buffers=2 leaky=2"
            " ! h265parse config-interval=-1"
            " ! nvv4l2decoder disable-dpb=true low-latency-mode=true skip-frames=2"
            " ! nvv4l2h264enc idrinterval=15 insert-sps-pps=true"
            " bitrate=" + std::to_string(encode_bitrate_bps_) +
            " ! h264parse config-interval=-1"
            " ! video/x-h264,stream-format=byte-stream,alignment=au"
            " ! appsink name=sink sync=false max-buffers=2 drop=true";

        RCLCPP_INFO(get_logger(), "Pipeline: %s", pipeline_str.c_str());
        pipeline_str_ = pipeline_str;
        if (!start_pipeline()) {
            return;
        }
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
        stop_pipeline();
    }

private:
    bool start_pipeline()
    {
        stop_pipeline();

        GError* err = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str_.c_str(), &err);
        if (err) {
            RCLCPP_ERROR(get_logger(), "GStreamer parse error: %s", err->message);
            g_error_free(err);
            pipeline_ = nullptr;
            return false;
        }

        sink_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "sink"));
        if (!sink_) {
            RCLCPP_ERROR(get_logger(), "Failed to find appsink in pipeline");
            stop_pipeline();
            return false;
        }

        GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            RCLCPP_ERROR(get_logger(), "Failed to set pipeline to PLAYING");
            stop_pipeline();
            return false;
        }

        RCLCPP_INFO(get_logger(), "Pipeline PLAYING");
        return true;
    }

    void stop_pipeline()
    {
        if (sink_) {
            gst_object_unref(sink_);
            sink_ = nullptr;
        }
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
        }
    }

    bool pipeline_needs_restart()
    {
        if (!pipeline_) {
            return true;
        }

        GstBus* bus = gst_element_get_bus(pipeline_);
        if (!bus) {
            return false;
        }

        bool should_restart = false;
        while (GstMessage* msg = gst_bus_pop_filtered(
                   bus,
                   static_cast<GstMessageType>(
                       GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_ELEMENT))) {
            switch (GST_MESSAGE_TYPE(msg)) {
                case GST_MESSAGE_ERROR: {
                    GError* err = nullptr;
                    gchar* debug = nullptr;
                    gst_message_parse_error(msg, &err, &debug);
                    RCLCPP_ERROR(
                        get_logger(),
                        "Camera pipeline error: %s%s%s",
                        err ? err->message : "unknown",
                        debug ? " | " : "",
                        debug ? debug : "");
                    if (err) g_error_free(err);
                    if (debug) g_free(debug);
                    should_restart = true;
                    break;
                }
                case GST_MESSAGE_EOS:
                    RCLCPP_WARN(get_logger(), "Camera pipeline reached EOS; restarting");
                    should_restart = true;
                    break;
                case GST_MESSAGE_ELEMENT: {
                    const GstStructure* s = gst_message_get_structure(msg);
                    if (s && gst_structure_has_name(s, "GstUDPSrcTimeout")) {
                        RCLCPP_WARN(get_logger(), "Camera UDP source timed out; restarting");
                        should_restart = true;
                    }
                    break;
                }
                default:
                    break;
            }
            gst_message_unref(msg);
            if (should_restart) break;
        }

        gst_object_unref(bus);
        return should_restart;
    }

    bool restart_pipeline(const char* reason)
    {
        RCLCPP_WARN(get_logger(), "Restarting camera pipeline: %s", reason);
        stop_pipeline();
        if (!running_ || !rclcpp::ok()) {
            return false;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(std::max<int64_t>(0, static_cast<int64_t>(restart_backoff_ms_))));
        return start_pipeline();
    }

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
        auto   last_frame_t = Clock::now();
        bool   seen_frame = false;

        while (running_ && rclcpp::ok()) {
            if (pipeline_needs_restart()) {
                if (!restart_pipeline("GStreamer bus event")) {
                    break;
                }
                seen_frame = false;
                last_frame_t = Clock::now();
                continue;
            }

            auto t0 = Clock::now();
            GstSample* sample = gst_app_sink_try_pull_sample(sink_, 100 * GST_MSECOND);
            auto t1 = Clock::now();
            double pull_ms = ms(t1 - t0).count();

            if (!sample) {
                if (ms(t1 - last_frame_t).count() > frame_timeout_ms_) {
                    const char* reason = seen_frame ? "frame timeout" : "startup frame timeout";
                    if (!restart_pipeline(reason)) {
                        break;
                    }
                    seen_frame = false;
                    last_frame_t = Clock::now();
                }
                continue;
            }

            last_frame_t = t1;
            seen_frame = true;

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
    std::string pipeline_str_;
    int port_;
    int encode_bitrate_bps_;
    double frame_timeout_ms_;
    double restart_backoff_ms_;

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
