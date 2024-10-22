#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <iostream>
#include <string>

static GMainLoop *loop = NULL;

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);

    // 사용할 MP4 파일 경로 (절대 경로 또는 상대 경로로 변경 가능)
    std::string mp4_file_path = "{yourpath to video}.mp4";  // 수정 필요

    GstRTSPServer *server = gst_rtsp_server_new();
    g_object_set(server, "service", "8554", NULL);

    GstRTSPMountPoints *mounts = gst_rtsp_server_get_mount_points(server);
    GstRTSPMediaFactory *factory = gst_rtsp_media_factory_new();

    // MP4 파일을 소스로 사용하는 파이프라인
    std::string launch_desc =
        "( filesrc location=" + mp4_file_path + " ! "
        "decodebin ! "
        "videoconvert ! video/x-raw! "
        "x264enc tune=zerolatency bitrate=4000 speed-preset=ultrafast key-int-max=15 ! "
        "rtph264pay name=pay0 pt=96 )";

    gst_rtsp_media_factory_set_launch(factory, launch_desc.c_str());
    gst_rtsp_media_factory_set_shared(factory, TRUE);

    gst_rtsp_mount_points_add_factory(mounts, "/test", factory);
    g_object_unref(mounts);

    gst_rtsp_server_attach(server, NULL);

    loop = g_main_loop_new(NULL, FALSE);

    std::cout << "MP4 stream ready at rtsp://127.0.0.1:8554/test" << std::endl;
    std::cout << "Press Ctrl-C to quit" << std::endl;

    g_main_loop_run(loop);

    g_main_loop_unref(loop);
    gst_object_unref(server);

    return 0;
}
