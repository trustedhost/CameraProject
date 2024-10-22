#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <condition_variable>

static GstElement *pipeline, *appsrc, *videoconvert, *x264enc, *rtph264pay, *udpsink;
static GMainLoop *loop;
static std::atomic<bool> is_running(true);
static std::mutex frame_mutex;
static std::condition_variable frame_cv;
static cv::Mat latest_frame;
static bool new_frame_available = false;

std::string generate_filename() {
    auto now = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm *ltm = std::localtime(&time);
    char filename[100];
    std::strftime(filename, sizeof(filename), "%Y-%m-%d_%H-%M-%S.mp4", ltm);
    return std::string(filename);
}

static void on_new_frame(cv::Mat &frame) {
    if (frame.empty()) return;
    
    // Convert BGR to I420 (YUV420)
    cv::Mat yuv_frame;
    cv::cvtColor(frame, yuv_frame, cv::COLOR_BGR2YUV_I420);
    
    GstBuffer *buffer = gst_buffer_new_allocate(NULL, yuv_frame.total(), NULL);
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
    memcpy(map.data, yuv_frame.data, yuv_frame.total());
    gst_buffer_unmap(buffer, &map);
    
    GstClockTime timestamp = GST_CLOCK_TIME_NONE;
    GST_BUFFER_PTS(buffer) = timestamp;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, 30);
    
    GstFlowReturn ret;
    g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    // Update latest frame for video saving
    std::lock_guard<std::mutex> lock(frame_mutex);
    latest_frame = frame.clone();
    new_frame_available = true;
    frame_cv.notify_one();
}

static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data) {
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_EOS:
            g_print("End of stream\n");
            g_main_loop_quit(loop);
            break;
        case GST_MESSAGE_ERROR: {
            gchar *debug;
            GError *error;
            gst_message_parse_error(msg, &error, &debug);
            g_free(debug);
            g_printerr("Error: %s\n", error->message);
            g_error_free(error);
            g_main_loop_quit(loop);
            break;
        }
        default:
            break;
    }
    return TRUE;
}

void start_video_saving_thread() {
    std::thread([&]() {
        while (is_running) {
            std::string filename = generate_filename();
            g_print("Saving video to file: %s\n", filename.c_str());

            cv::VideoWriter writer(filename, cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 30, cv::Size(1280, 720));
            
            auto start_time = std::chrono::steady_clock::now();
            while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 60) {
                std::unique_lock<std::mutex> lock(frame_mutex);
                frame_cv.wait(lock, [] { return new_frame_available; });
                
                writer.write(latest_frame);
                new_frame_available = false;
            }
            
            writer.release();
            g_print("Video saved: %s\n", filename.c_str());
        }
    }).detach();
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);
    
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open webcam!" << std::endl;
        return -1;
    }
    
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_FPS, 30);
    
    pipeline = gst_pipeline_new("webcam-rtp-stream");
    appsrc = gst_element_factory_make("appsrc", "source");
    videoconvert = gst_element_factory_make("videoconvert", "converter");
    x264enc = gst_element_factory_make("x264enc", "encoder");
    rtph264pay = gst_element_factory_make("rtph264pay", "payloader");
    udpsink = gst_element_factory_make("udpsink", "sink");
    
    if (!pipeline || !appsrc || !videoconvert || !x264enc || !rtph264pay || !udpsink) {
        g_printerr("One element could not be created. Exiting.\n");
        return -1;
    }
    
    g_object_set(G_OBJECT(appsrc), "caps",
                 gst_caps_new_simple("video/x-raw",
                                     "format", G_TYPE_STRING, "I420",
                                     "width", G_TYPE_INT, 1280,
                                     "height", G_TYPE_INT, 720,
                                     "framerate", GST_TYPE_FRACTION, 30, 1,
                                     NULL), NULL);
    g_object_set(G_OBJECT(appsrc), "format", GST_FORMAT_TIME, NULL);
    g_object_set(G_OBJECT(x264enc), "tune", 0x00000004, "speed-preset", 1, "bitrate", 2000, NULL);
    g_object_set(G_OBJECT(rtph264pay), "config-interval", 1, NULL);
    g_object_set(G_OBJECT(udpsink), "host", "127.0.0.1", "port", 5000, NULL);
    
    gst_bin_add_many(GST_BIN(pipeline), appsrc, videoconvert, x264enc, rtph264pay, udpsink, NULL);
    if (!gst_element_link_many(appsrc, videoconvert, x264enc, rtph264pay, udpsink, NULL)) {
        g_printerr("Elements could not be linked. Exiting.\n");
        return -1;
    }
    
    GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    guint bus_watch_id = gst_bus_add_watch(bus, bus_call, NULL);
    gst_object_unref(bus);
    
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    
    loop = g_main_loop_new(NULL, FALSE);
    
    std::thread streaming_thread([&]() {
        cv::Mat frame;
        while (is_running) {
            cap >> frame;
            if (frame.empty()) {
                std::cerr << "Error: Could not capture frame!" << std::endl;
                break;
            }
            on_new_frame(frame);
            cv::waitKey(1);
        }
    });
    
    start_video_saving_thread();

    g_main_loop_run(loop);
    
    is_running = false;
    streaming_thread.join();
    
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(GST_OBJECT(pipeline));
    g_source_remove(bus_watch_id);
    g_main_loop_unref(loop);
    
    return 0;
}