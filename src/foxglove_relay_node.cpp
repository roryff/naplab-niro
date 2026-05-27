/**
 * foxglove_relay_node.cpp
 *
 * ROS 2 composable node — subscribes to a foxglove_msgs/CompressedVideo topic
 * carrying H.265 Annex-B byte-stream, transcodes it to H.264 using NVCodec hardware
 * (appsrc → h265parse → nvh265dec → nvh264enc), and republishes on a
 * preview topic for Foxglove Studio.
 *
 * Requires NVDEC/NVENC hardware (NVCodec GStreamer plugin).
 *
 * The GStreamer transcode pipeline starts lazily on the first output subscriber
 * and tears down when the last subscriber disconnects.
 *
 * Missing or late frames are acceptable — this is preview-quality only.
 *
 * Parameters
 * ----------
 * input_topic          H.265 CompressedVideo topic to subscribe to
 * output_topic         H.264 CompressedVideo topic to publish on (preview)
 * encode_bitrate_bps   H.264 target bitrate in bps   (default: 500 000)
 * restart_cooldown_ms  Minimum ms between pipeline restarts (default: 3000)
 */

#include <chrono>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstring>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <foxglove_msgs/msg/compressed_video.hpp>

// Process-wide mutex: only one relay decoder may initialise at a time.
// nvh265dec/nvh264enc can exhaust NVMM when multiple instances
// allocate buffers simultaneously.
static std::mutex g_pipeline_start_mutex;

class FoxgloveRelayNode : public rclcpp::Node
{
public:
    explicit FoxgloveRelayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("foxglove_relay", options)
    {
        if (!gst_is_initialized()) {
            gst_init(nullptr, nullptr);
        }

        input_topic_  = declare_parameter<std::string>("input_topic",  "");
        output_topic_ = declare_parameter<std::string>("output_topic", "");
        bitrate_      = declare_parameter<int>("encode_bitrate_bps", 2000000);
        restart_cooldown_ms_ = declare_parameter<int>("restart_cooldown_ms", 3000);

        if (input_topic_.empty() || output_topic_.empty()) {
            RCLCPP_ERROR(get_logger(), "input_topic and output_topic parameters must be set");
            return;
        }

        // Hardware decode/encode: nvh265dec (NVCodec NVDEC) → nvh264enc (NVCodec NVENC).
        // bitrate is in bps; gop-size sets the IDR keyframe interval (frames).
        // SPS/PPS re-injection is handled by h264parse config-interval=-1 downstream.
        pipeline_str_ =
            "appsrc name=src is-live=true do-timestamp=true format=3 "
            "caps=video/x-h265,stream-format=byte-stream,alignment=au,framerate=0/1 "
            "! h265parse config-interval=-1 "
            "! nvh265dec "
            "! nvh264enc bitrate=" + std::to_string(bitrate_) + " gop-size=15 zerolatency=true "
            "! h264parse config-interval=-1 "
            "! video/x-h264,stream-format=byte-stream,alignment=au "
            "! appsink name=sink sync=false max-buffers=2 drop=true";

        pub_ = create_publisher<foxglove_msgs::msg::CompressedVideo>(output_topic_, 10);

        sub_ = create_subscription<foxglove_msgs::msg::CompressedVideo>(
            input_topic_, 10,
            [this](foxglove_msgs::msg::CompressedVideo::SharedPtr msg) {
                on_frame(std::move(msg));
            });

        pull_thread_ = std::thread(&FoxgloveRelayNode::pull_loop, this);

        RCLCPP_INFO(get_logger(), "Relay ready: %s → %s",
            input_topic_.c_str(), output_topic_.c_str());
    }

    ~FoxgloveRelayNode()
    {
        running_ = false;
        if (pull_thread_.joinable()) pull_thread_.join();
        stop_pipeline();
    }

private:
    void on_frame(foxglove_msgs::msg::CompressedVideo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pipeline_mutex_);

        const bool has_subs = pub_->get_subscription_count() > 0;

        // Tear down when nobody is watching to free GPU resources.
        if (!has_subs) {
            if (pipeline_) {
                RCLCPP_INFO(get_logger(), "No subscribers — stopping relay pipeline");
                stop_pipeline_locked();
            }
            return;
        }

        // Drain bus errors from a running pipeline — stop and enter cooldown,
        // do NOT immediately restart (prevents the NVMM-exhaustion restart storm).
        if (pipeline_ && check_bus_error_locked()) {
            RCLCPP_WARN(get_logger(), "Relay pipeline error — cooling down %d ms before restart",
                restart_cooldown_ms_);
            stop_pipeline_locked();
            last_error_time_ = std::chrono::steady_clock::now();
            return;
        }

        // Lazy start (or restart after cooldown).
        if (!pipeline_) {
            if (last_error_time_ != std::chrono::steady_clock::time_point{}) {
                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - last_error_time_).count();
                if (elapsed_ms < restart_cooldown_ms_) {
                    return;  // still cooling down — drop frame silently
                }
                last_error_time_ = {};
            }
            RCLCPP_INFO(get_logger(), "Subscriber appeared — starting relay pipeline");
            if (!start_pipeline_locked()) {
                last_error_time_ = std::chrono::steady_clock::now();
                return;
            }
        }

        // Push Annex-B buffer into appsrc.
        GstBuffer* buf = gst_buffer_new_and_alloc(msg->data.size());
        GstMapInfo map;
        gst_buffer_map(buf, &map, GST_MAP_WRITE);
        std::memcpy(map.data, msg->data.data(), msg->data.size());
        gst_buffer_unmap(buf, &map);

        GstFlowReturn ret = gst_app_src_push_buffer(src_, buf);
        if (ret != GST_FLOW_OK) {
            RCLCPP_WARN(get_logger(), "appsrc push failed (%d) — dropping frame", ret);
        }

        // Cache frame_id for pull_loop publishing.
        last_frame_id_ = msg->frame_id;
    }

    // ---- pipeline helpers (all must be called with pipeline_mutex_ held) ----

    bool start_pipeline_locked()
    {
        // Serialise decoder initialisation across all relay instances in this process.
        // nvv4l2decoder exhausts the NVMM pool when multiple instances start in parallel.
        std::lock_guard<std::mutex> sg(g_pipeline_start_mutex);
        GError* err = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str_.c_str(), &err);
        if (err) {
            RCLCPP_ERROR(get_logger(), "GStreamer parse error: %s", err->message);
            g_error_free(err);
            pipeline_ = nullptr;
            return false;
        }

        src_  = GST_APP_SRC (gst_bin_get_by_name(GST_BIN(pipeline_), "src"));
        sink_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "sink"));
        if (!src_ || !sink_) {
            RCLCPP_ERROR(get_logger(), "Failed to locate appsrc/appsink in pipeline");
            stop_pipeline_locked();
            return false;
        }

        // Non-blocking: drop frames rather than stall the ROS callback.
        g_object_set(src_, "block", FALSE, "max-bytes", (guint64)0, nullptr);

        if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            RCLCPP_ERROR(get_logger(), "Failed to set relay pipeline to PLAYING");
            stop_pipeline_locked();
            return false;
        }

        RCLCPP_INFO(get_logger(), "Relay pipeline PLAYING");
        return true;
    }

    void stop_pipeline_locked()
    {
        if (src_)  { gst_object_unref(src_);  src_  = nullptr; }
        if (sink_) { gst_object_unref(sink_); sink_ = nullptr; }
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
        }
    }

    void stop_pipeline()
    {
        std::lock_guard<std::mutex> lock(pipeline_mutex_);
        stop_pipeline_locked();
    }

    // Returns true if the pipeline needs a restart.
    bool check_bus_error_locked()
    {
        if (!pipeline_) return false;
        GstBus* bus = gst_element_get_bus(pipeline_);
        if (!bus) return false;
        bool needs_restart = false;
        while (GstMessage* msg = gst_bus_pop_filtered(
                   bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS))) {
            if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
                GError* e = nullptr; gchar* dbg = nullptr;
                gst_message_parse_error(msg, &e, &dbg);
                RCLCPP_ERROR(get_logger(), "Relay pipeline error: %s",
                    e ? e->message : "unknown");
                if (e) g_error_free(e);
                if (dbg) g_free(dbg);
            }
            gst_message_unref(msg);
            needs_restart = true;
        }
        gst_object_unref(bus);
        return needs_restart;
    }

    // ---- pull thread --------------------------------------------------------

    void pull_loop()
    {
        while (running_) {
            // Grab a reference to sink_ safely so we can pull without holding the mutex.
            GstAppSink* sink_local = nullptr;
            {
                std::lock_guard<std::mutex> lock(pipeline_mutex_);
                sink_local = sink_;
                if (sink_local) gst_object_ref(sink_local);
            }

            if (!sink_local) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            GstSample* sample = gst_app_sink_try_pull_sample(sink_local, 50 * GST_MSECOND);
            gst_object_unref(sink_local);

            if (!sample) continue;

            GstBuffer* buf = gst_sample_get_buffer(sample);
            GstMapInfo map;
            if (gst_buffer_map(buf, &map, GST_MAP_READ)) {
                auto ros_msg = std::make_unique<foxglove_msgs::msg::CompressedVideo>();
                ros_msg->timestamp = get_clock()->now();
                ros_msg->frame_id  = last_frame_id_;
                ros_msg->format    = "h264";
                ros_msg->data.assign(map.data, map.data + map.size);
                gst_buffer_unmap(buf, &map);
                pub_->publish(std::move(ros_msg));
            }

            gst_sample_unref(sample);
        }
    }

    // ---- members ------------------------------------------------------------

    std::string input_topic_, output_topic_, pipeline_str_;
    std::string last_frame_id_;
    int bitrate_;

    std::mutex   pipeline_mutex_;
    GstElement*  pipeline_ = nullptr;
    GstAppSrc*   src_      = nullptr;
    GstAppSink*  sink_     = nullptr;

    rclcpp::Publisher<foxglove_msgs::msg::CompressedVideo>::SharedPtr pub_;
    rclcpp::Subscription<foxglove_msgs::msg::CompressedVideo>::SharedPtr sub_;

    std::thread       pull_thread_;
    std::atomic<bool> running_{true};

    int restart_cooldown_ms_{3000};
    std::chrono::steady_clock::time_point last_error_time_{};
};

RCLCPP_COMPONENTS_REGISTER_NODE(FoxgloveRelayNode)

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FoxgloveRelayNode>());
    rclcpp::shutdown();
    return 0;
}
