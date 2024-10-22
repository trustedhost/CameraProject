#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <glib.h>
#include "FaceLandmark.hpp"
#include <iostream>
#include <chrono>
#include <curl/curl.h>

// Static variable to track the last send time
static std::chrono::steady_clock::time_point last_send_time = std::chrono::steady_clock::now();
static const int send_interval = 60; // 60 seconds

static GstElement *pipeline, *udpsrc, *rtph264depay, *avdec_h264, *videoconvert, *appsink;
static GMainLoop *loop;
static cv::Mat current_frame;
static GMutex frame_mutex;
static GCond frame_cond;
static bool new_frame_available = false;

static my::FaceLandmark *landMarker;

static void on_new_sample(GstElement *sink) {
    GstSample *sample;
    g_signal_emit_by_name(sink, "pull-sample", &sample);
    if (sample) {
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        GstMapInfo map;
        gst_buffer_map(buffer, &map, GST_MAP_READ);

        GstCaps *caps = gst_sample_get_caps(sample);
        GstStructure *structure = gst_caps_get_structure(caps, 0);
        int width, height;
        gst_structure_get_int(structure, "width", &width);
        gst_structure_get_int(structure, "height", &height);

        cv::Mat yuv_image(height * 3 / 2, width, CV_8UC1, (void *)map.data);
        cv::Mat bgr_image;
        cv::cvtColor(yuv_image, bgr_image, cv::COLOR_YUV2BGR_I420);

        g_mutex_lock(&frame_mutex);
        bgr_image.copyTo(current_frame);
        new_frame_available = true;
        g_cond_signal(&frame_cond);
        g_mutex_unlock(&frame_mutex);

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
    }
}

static GstFlowReturn new_sample_callback(GstElement *sink, gpointer user_data) {
    on_new_sample(sink);
    return GST_FLOW_OK;
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


static gboolean update_frame(gpointer user_data) {
    g_mutex_lock(&frame_mutex);
    if (new_frame_available) {
        cv::Mat frame = current_frame.clone();
        g_mutex_unlock(&frame_mutex);

        landMarker->loadImageToInput(frame);
        landMarker->runInference();

        for (auto landmark : landMarker->getAllFaceLandmarks()) {
            cv::circle(frame, landmark, 2, cv::Scalar(0, 255, 0), -1);
        }
        
        auto headDirection = landMarker->detectHeadDirection();
        /* Display direction value on top left of the screen */
        cv::putText(frame, "Head Direction: " + std::to_string(headDirection[0]) + ", " + std::to_string(headDirection[1]), 
                    cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        
        if (headDirection[0] > -10 && headDirection[0] < 10 && headDirection[1] > 0 && headDirection[1] < 30) {
            // Show message indicating head is looking at the center of the screen
            cv::putText(frame, "Head is looking at the center of the screen", cv::Point(10, 40), 
                        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 1);

            
            // Check if enough time has passed since the last send
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = now - last_send_time;
            if (elapsed_seconds.count() >= send_interval) {
                // Send the HTTPS request
                CURL *curl;
                CURLcode res;

                curl = curl_easy_init();
                if(curl) {
                    std::string url = "https://yourmessagedomain.com/sendMessage.php?to=01042256954&body=FaceDetected";
                    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

                    // Enable SSL verification
                    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L); // Verify the peer's SSL certificate
                    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L); // Verify the host's SSL certificate

                    // Optional: Specify a CA bundle if needed (you can omit this if your system has a valid CA bundle)
                    // curl_easy_setopt(curl, CURLOPT_CAINFO, "/path/to/cacert.pem");

                    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
                    
                    // Perform the request, res will get the return code
                    res = curl_easy_perform(curl);

                    // Check for errors
                    if(res != CURLE_OK) {
                        fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
                    }
                    
                    // Cleanup
                    curl_easy_cleanup(curl);
                }

                // Update the last send time
                last_send_time = now;
            }
            
        }

        cv::imshow("RTP Stream with Face and Iris Detection", frame);
        cv::waitKey(1);

        g_mutex_lock(&frame_mutex);
        new_frame_available = false;
        g_mutex_unlock(&frame_mutex);
    } else {
        g_mutex_unlock(&frame_mutex);
    }
    return G_SOURCE_CONTINUE;
}


int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);

    // InitializeFaceLandmark 
    landMarker = new my::FaceLandmark("./models");

    // ... (existing GStreamer pipeline setup)

    pipeline = gst_pipeline_new("rtp-client");
    udpsrc = gst_element_factory_make("udpsrc", "source");
    rtph264depay = gst_element_factory_make("rtph264depay", "depayloader");
    avdec_h264 = gst_element_factory_make("avdec_h264", "decoder");
    videoconvert = gst_element_factory_make("videoconvert", "converter");
    appsink = gst_element_factory_make("appsink", "sink");

    if (!pipeline || !udpsrc || !rtph264depay || !avdec_h264 || !videoconvert || !appsink) {
        g_printerr("One element could not be created. Exiting.\n");
        return -1;
    }

    g_object_set(G_OBJECT(udpsrc), "port", 5000, NULL);
    
    GstCaps *caps = gst_caps_new_simple("application/x-rtp",
                                        "media", G_TYPE_STRING, "video",
                                        "clock-rate", G_TYPE_INT, 90000,
                                        "encoding-name", G_TYPE_STRING, "H264",
                                        NULL);
    g_object_set(G_OBJECT(udpsrc), "caps", caps, NULL);
    gst_caps_unref(caps);

    g_object_set(G_OBJECT(appsink), "emit-signals", TRUE, "sync", FALSE, NULL);

    gst_bin_add_many(GST_BIN(pipeline), udpsrc, rtph264depay, avdec_h264, videoconvert, appsink, NULL);
    gst_element_link_many(udpsrc, rtph264depay, avdec_h264, videoconvert, appsink, NULL);

    GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    guint bus_watch_id = gst_bus_add_watch(bus, bus_call, NULL);
    gst_object_unref(bus);

    g_signal_connect(appsink, "new-sample", G_CALLBACK(new_sample_callback), NULL);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    loop = g_main_loop_new(NULL, FALSE);

    cv::namedWindow("RTP camera stream", cv::WINDOW_AUTOSIZE);

    g_mutex_init(&frame_mutex);
    g_cond_init(&frame_cond);

    g_timeout_add(33, update_frame, NULL);  // ~30 fps

    g_main_loop_run(loop);

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(GST_OBJECT(pipeline));
    g_source_remove(bus_watch_id);
    g_main_loop_unref(loop);

    g_mutex_clear(&frame_mutex);
    g_cond_clear(&frame_cond);

    delete landMarker;

    return 0;
}