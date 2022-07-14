// Copyright 2014 Robert Bosch, LLC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Robert Bosch, LLC nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#define __STDC_CONSTANT_MACROS
#include "usb_cam/usb_cam.hpp"

#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <ros/ros.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>              /* low-level i/o */
#include <iostream>
// #include <usb_cam/msg/formats.hpp>

// #include <sensor_msgs/fill_image.h>

#include <memory>
#include <string>
#include <vector>

#define CLEAR(x) memset(&(x), 0, sizeof(x))


namespace usb_cam
{

UsbCam::UsbCam()
: io_(IO_METHOD_MMAP), fd_(-1), buffers_(NULL), n_buffers_(0), avframe_camera_(NULL),
  avframe_rgb_(NULL), avcodec_(NULL), avoptions_(NULL), avcodec_context_(NULL),
  avframe_camera_size_(0), avframe_rgb_size_(0), video_sws_(NULL), image_(NULL),
  is_capturing_(false) {}
UsbCam::~UsbCam()
{
  shutdown();
}

int UsbCam::init_decoder(int image_width, int image_height, color_format color_format, AVCodecID codec_id, const char *codec_name)
{
  avcodec_register_all();

  avcodec_ = avcodec_find_decoder(codec_id);
  if (!avcodec_) {
    ROS_ERROR("Could not find %s decoder", codec_name);
    return 0;
  }

  avcodec_context_ = avcodec_alloc_context3(avcodec_);
#if LIBAVCODEC_VERSION_MAJOR < 55
  avframe_camera_ = avcodec_alloc_frame();
  avframe_rgb_ = avcodec_alloc_frame();
#else
  avframe_camera_ = av_frame_alloc();
  avframe_rgb_ = av_frame_alloc();
#endif

  avpicture_alloc(
    reinterpret_cast<AVPicture *>(avframe_rgb_), AV_PIX_FMT_RGB24, image_width, image_height);

  avcodec_context_->codec_id = codec_id;
  avcodec_context_->width = image_width;
  avcodec_context_->height = image_height;

#if LIBAVCODEC_VERSION_MAJOR > 52
  // TODO(lucasw) it gets set correctly here, but then changed later to deprecated J422P format
  if (color_format == COLOR_FORMAT_YUV420P) {
    avcodec_context_->pix_fmt = AV_PIX_FMT_YUV420P;
    ROS_INFO_STREAM(
      "using YUV420P " << AV_PIX_FMT_YUV420P << " " << avcodec_context_->pix_fmt);
  }
  else {
    avcodec_context_->pix_fmt = AV_PIX_FMT_YUV422P;
    ROS_INFO_STREAM(
      "using YUV422P " << AV_PIX_FMT_YUV422P << " " << avcodec_context_->pix_fmt);
  }
  
  avcodec_context_->codec_type = AVMEDIA_TYPE_VIDEO;
#endif
  if (color_format == COLOR_FORMAT_YUV420P) 
    avframe_camera_size_ = avpicture_get_size(AV_PIX_FMT_YUV420P, image_width, image_height);
  else 
    avframe_camera_size_ = avpicture_get_size(AV_PIX_FMT_YUV422P, image_width, image_height);
  avframe_rgb_size_ = avpicture_get_size(AV_PIX_FMT_RGB24, image_width, image_height);

  /* open it */
  if (avcodec_open2(avcodec_context_, avcodec_, &avoptions_) < 0) {
    ROS_ERROR("Could not open %s Decoder", codec_name);
    return 0;
  }
  ROS_INFO_STREAM(
    "pixel format " << AV_PIX_FMT_YUV422P << " " << avcodec_context_->pix_fmt);
  return 1;
}

int UsbCam::init_mjpeg_decoder(int image_width, int image_height, color_format cf) {
  return init_decoder(image_width, image_height, cf, AV_CODEC_ID_MJPEG, "MJPEG");
}

int UsbCam::init_h264_decoder(int image_width, int image_height, color_format cf) {
  return init_decoder(image_width, image_height, cf, AV_CODEC_ID_H264, "H264");
}

bool UsbCam::mjpeg2rgb(char * MJPEG, int len, char * RGB, int /* NumPixels */)
{
  int got_picture;

  // clear the picture
  memset(RGB, 0, avframe_rgb_size_);

#if LIBAVCODEC_VERSION_MAJOR > 52
  int decoded_len;
  AVPacket avpkt;
  av_init_packet(&avpkt);

  avpkt.size = len;
  avpkt.data = (unsigned char *)MJPEG;
  AVPixelFormat pix_fmt_backup = avcodec_context_->pix_fmt;
  // TODO(lucasw) this corrupts pixel format
  decoded_len = avcodec_decode_video2(avcodec_context_, avframe_camera_, &got_picture, &avpkt);

  if (decoded_len < 0) {
    ROS_ERROR("Error while decoding frame.");
    return false;
  }
#else
  avcodec_decode_video(
    avcodec_context_, avframe_camera_, &got_picture, reinterpret_cast<uint8_t *>(MJPEG), len);
#endif

  if (!got_picture) {
    ROS_ERROR("Webcam: expected picture but didn't get it...");
    return false;
  }

  int xsize = avcodec_context_->width;
  int ysize = avcodec_context_->height;
  // TODO(lucasw) avpicture_get_size corrupts the pix_fmt
  int pic_size = avpicture_get_size(avcodec_context_->pix_fmt, xsize, ysize);
  // int pic_size = av_image_get_buffer_size(avcodec_context_->pix_fmt, xsize, ysize);
  if (pic_size != avframe_camera_size_) {
    ROS_ERROR(
      "outbuf size mismatch.  pic_size: %d bufsize: %d", pic_size, avframe_camera_size_);
    return false;
  }

  // TODO(lucasw) why does the image need to be scaled?  Does it also convert formats?
  #if 1
  avcodec_context_->pix_fmt = pix_fmt_backup;
  // TODO(lucasw) only do if xsize and ysize or pix fmt is different from last time
  video_sws_ = sws_getContext(
    xsize, ysize, avcodec_context_->pix_fmt, xsize, ysize,
    AV_PIX_FMT_RGB24, SWS_BILINEAR, NULL, NULL, NULL);
  sws_scale(
    video_sws_, avframe_camera_->data, avframe_camera_->linesize,
    0, ysize, avframe_rgb_->data, avframe_rgb_->linesize);
  // TODO(lucasw) keep around until parameters change
  sws_freeContext(video_sws_);
  #endif

  int size = avpicture_layout(
    reinterpret_cast<AVPicture *>(avframe_rgb_), AV_PIX_FMT_RGB24,
    xsize, ysize, reinterpret_cast<uint8_t *>(RGB), avframe_rgb_size_);
  if (size != avframe_rgb_size_) {
    ROS_ERROR("webcam: avpicture_layout error: %d", size);
    return false;
  }
  return true;
}

bool UsbCam::process_image(const void * src, int len, camera_image_t * dest)
{
  // TODO(lucasw) return bool from all these
  if (pixelformat_ == V4L2_PIX_FMT_YUYV) {
    if (monochrome_) {
      // actually format V4L2_PIX_FMT_Y16, but xioctl gets unhappy
      // if you don't use the advertised type (yuyv)
      mono102mono8(
        const_cast<char *>(
          reinterpret_cast<const char *>(src)), dest->image, dest->width * dest->height);
    } else {
      yuyv2rgb(
        const_cast<char *>(
          reinterpret_cast<const char *>(src)), dest->image, dest->width * dest->height);
    }
  } else if (pixelformat_ == V4L2_PIX_FMT_UYVY) {
    uyvy2rgb(
      const_cast<char *>(
        reinterpret_cast<const char *>(src)), dest->image, dest->width * dest->height);
  } else if (pixelformat_ == V4L2_PIX_FMT_MJPEG) {
    return mjpeg2rgb(
      const_cast<char *>(
        reinterpret_cast<const char *>(src)), len, dest->image, dest->width * dest->height);
  } else if (pixelformat_ == V4L2_PIX_FMT_H264) {
    // libav handles the decoding, so reusing the same function is fine
    return mjpeg2rgb(
      const_cast<char *>(
        reinterpret_cast<const char *>(src)), len, dest->image, dest->width * dest->height);
  } else if (pixelformat_ == V4L2_PIX_FMT_RGB24) {
    rgb242rgb(
      const_cast<char *>(
        reinterpret_cast<const char *>(src)), dest->image, dest->width * dest->height);
  } else if (pixelformat_ == V4L2_PIX_FMT_GREY) {
    memcpy(
      dest->image,
      const_cast<char *>(reinterpret_cast<const char *>(src)), dest->width * dest->height);
  }

  return true;
}

bool UsbCam::read_frame()
{
  struct v4l2_buffer buf;
  unsigned int i;
  int len;
  ros::Time stamp;
  timespec buf_time;
  timespec real_time;

  switch (io_) {
    case IO_METHOD_READ:
      len = read(fd_, buffers_[0].start, buffers_[0].length);
      if (len == -1) {
        switch (errno) {
          case EAGAIN:
            return false;

          case EIO:
          /* Could ignore EIO, see spec. */

          /* fall through */

          default:
            std::cerr << "error, quitting " << errno << std::endl;
            return false;  // ("read");
        }
      }

      if (!process_image(buffers_[0].start, len, image_)) {
        return false;
      }
      // TODO(lucasw) how to get timestamp with this method?

      break;

    case IO_METHOD_MMAP:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      if (-1 == xioctl(fd_, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
          case EAGAIN:
            return false;

          case EIO:
          /* Could ignore EIO, see spec. */

          /* fall through */

          default:
            std::cerr << "error, quitting " << errno << std::endl;
            return false;  // ("VIDIOC_DQBUF");
        }
      }

      // need to get buf time here otherwise process_image will zero it
      TIMEVAL_TO_TIMESPEC(&buf.timestamp, &buf_time);
      monotonicToRealTime(buf_time, real_time);
      stamp.sec = real_time.tv_sec;
      stamp.nsec = real_time.tv_nsec;

      assert(buf.index < n_buffers_);
      len = buf.bytesused;
      if (!process_image(buffers_[buf.index].start, len, image_)) {
        return false;
      }

      if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf)) {
        std::cerr << "error, quitting " << errno << std::endl;
        return false;  // ("VIDIOC_QBUF");
      }

      image_->stamp = stamp;

      break;

    case IO_METHOD_USERPTR:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;

      if (-1 == xioctl(fd_, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
          case EAGAIN:
            return false;

          case EIO:
          /* Could ignore EIO, see spec. */

          /* fall through */

          default:
            std::cerr << "error, quitting " << errno << std::endl;
            return false;  // ("VIDIOC_DQBUF");
        }
      }

      TIMEVAL_TO_TIMESPEC(&buf.timestamp, &buf_time);
      monotonicToRealTime(buf_time, real_time);
      stamp.sec = real_time.tv_sec;
      stamp.nsec = real_time.tv_nsec;

      for (i = 0; i < n_buffers_; ++i) {
        if (buf.m.userptr == reinterpret_cast<uint64_t>(buffers_[i].start) && \
          buf.length == buffers_[i].length)
        {
          break;
        }
      }

      assert(i < n_buffers_);
      len = buf.bytesused;
      if (!process_image(reinterpret_cast<const void *>(buf.m.userptr), len, image_)) {
        return false;
      }

      if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf)) {
        std::cerr << "error, quitting " << errno << std::endl;
        return false;  // ("VIDIOC_QBUF");
      }

      image_->stamp = stamp;
      break;

    default:
      std::cerr << "Unknown io type " << io_ << std::endl;
      return false;
  }

  return true;
}

bool UsbCam::is_capturing()
{
  return is_capturing_;
}

bool UsbCam::stop_capturing(void)
{
  if (!is_capturing_) {return false;}

  is_capturing_ = false;
  enum v4l2_buf_type type;

  switch (io_) {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd_, VIDIOC_STREAMOFF, &type)) {
        std::cerr << "error, quitting " << errno << std::endl;
        return false;  // ("VIDIOC_STREAMOFF");
      }

      break;

    default:
      std::cerr << "Unknown io type " << io_ << std::endl;
      return false;
  }
  return true;
}

bool UsbCam::start_capturing(void)
{
  if (is_capturing_) {return false;}

  unsigned int i;
  enum v4l2_buf_type type;

  switch (io_) {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers_; ++i) {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf)) {
          std::cerr << "error, quitting " << errno << std::endl;
          return false;  // ("VIDIOC_QBUF");
        }
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd_, VIDIOC_STREAMON, &type)) {
        std::cerr << "error, quitting " << errno << std::endl;
        return false;  // ("VIDIOC_STREAMON");
      }
      break;

    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers_; ++i) {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.index = i;
        buf.m.userptr = reinterpret_cast<uint64_t>(buffers_[i].start);
        buf.length = buffers_[i].length;

        if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf)) {
          std::cerr << "error, quitting " << errno << std::endl;
          return false;  // ("VIDIOC_QBUF");
        }
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd_, VIDIOC_STREAMON, &type)) {
        std::cerr << "error, quitting " << errno << std::endl;
        return false;  // ("VIDIOC_STREAMON");
      }

      break;

    default:
      std::cerr << "Unknown io type " << io_ << std::endl;
      return false;
  }
  is_capturing_ = true;
  return true;
}

bool UsbCam::uninit_device(void)
{
  unsigned int i;

  switch (io_) {
    case IO_METHOD_READ:
      free(buffers_[0].start);
      break;

    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers_; ++i) {
        if (-1 == munmap(buffers_[i].start, buffers_[i].length)) {
          std::cerr << "error, quitting, TODO throw " << errno << std::endl;
          return false;  // ("munmap");
        }
      }
      break;

    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers_; ++i) {
        free(buffers_[i].start);
      }
      break;

    default:
      std::cerr << "Unknown io type " << io_ << std::endl;
      return false;
  }

  free(buffers_);
  return true;
}

bool UsbCam::init_read(unsigned int buffer_size)
{
  buffers_ = reinterpret_cast<buffer *>(calloc(1, sizeof(*buffers_)));

  if (!buffers_) {
    ROS_ERROR("Out of memory");
    return false;
  }

  buffers_[0].length = buffer_size;
  buffers_[0].start = malloc(buffer_size);

  if (!buffers_[0].start) {
    ROS_ERROR("Out of memory");
    return false;
  }
  return true;
}

bool UsbCam::init_mmap(void)
{
  struct v4l2_requestbuffers req;

  CLEAR(req);

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1 == xioctl(fd_, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      ROS_ERROR_STREAM(
        camera_dev_ << " does not support memory mapping");
      return false;
    } else {
      std::cerr << "error, quitting, TODO throw " << errno << std::endl;
      return false;  // ("VIDIOC_REQBUFS");
    }
  }

  if (req.count < 2) {
    ROS_ERROR_STREAM(
      "Insufficient buffer memory on " << camera_dev_);
    return false;
  }

  buffers_ = reinterpret_cast<buffer *>(calloc(req.count, sizeof(*buffers_)));

  if (!buffers_) {
    ROS_ERROR("Out of memory");
    return false;
  }

  for (n_buffers_ = 0; n_buffers_ < req.count; ++n_buffers_) {
    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers_;

    if (-1 == xioctl(fd_, VIDIOC_QUERYBUF, &buf)) {
      std::cerr << "error, quitting, TODO throw " << errno << std::endl;
      return false;  // ("VIDIOC_QUERYBUF");
    }

    buffers_[n_buffers_].length = buf.length;
    buffers_[n_buffers_].start =
      mmap(
      NULL /* start anywhere */, buf.length, PROT_READ | PROT_WRITE /* required */,
      MAP_SHARED /* recommended */, fd_, buf.m.offset);

    if (MAP_FAILED == buffers_[n_buffers_].start) {
      std::cerr << "error, quitting, TODO throw " << errno << std::endl;
      return false;  // ("mmap");
    }
  }
  return true;
}

bool UsbCam::init_userp(unsigned int buffer_size)
{
  struct v4l2_requestbuffers req;
  unsigned int page_size;

  page_size = getpagesize();
  buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

  CLEAR(req);

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;

  if (-1 == xioctl(fd_, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      ROS_ERROR_STREAM(
        camera_dev_ << " does not support user pointer i/o");
      return false;  // (EXIT_FAILURE);
    } else {
      std::cerr << "error, quitting, TODO throw " << errno << std::endl;
      return false;  // ("VIDIOC_REQBUFS");
    }
  }

  buffers_ = reinterpret_cast<buffer *>(calloc(4, sizeof(*buffers_)));

  if (!buffers_) {
    ROS_ERROR("Out of memory");
    return false;  // (EXIT_FAILURE);
  }

  for (n_buffers_ = 0; n_buffers_ < 4; ++n_buffers_) {
    buffers_[n_buffers_].length = buffer_size;
    buffers_[n_buffers_].start = memalign(/* boundary */ page_size, buffer_size);

    if (!buffers_[n_buffers_].start) {
      ROS_ERROR("Out of memory");
      return false;
    }
  }
  return true;
}

bool UsbCam::init_device(uint32_t image_width, uint32_t image_height, int framerate)
{
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min;

  if (-1 == xioctl(fd_, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL == errno) {
      ROS_ERROR_STREAM(
        camera_dev_ << " is no V4L2 device");
      return false;  // (EXIT_FAILURE);
    } else {
      std::cerr << "error, quitting, TODO throw " << errno << std::endl;
      return false;  // ("VIDIOC_QUERYCAP");
    }
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    ROS_ERROR_STREAM(
      camera_dev_ << " is no video capture device");
    return false;  // (EXIT_FAILURE);
  }

  switch (io_) {
    case IO_METHOD_READ:
      if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
        ROS_ERROR_STREAM(
          camera_dev_ << " does not support read i/o");
        return false;  // (EXIT_FAILURE);
      }

      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        ROS_ERROR_STREAM(
          camera_dev_ << " does not support streaming i/o");
        return false;  // (EXIT_FAILURE);
      }

      break;

    default:
      std::cerr << "Unknown io type " << io_ << std::endl;
      return false;
  }

  /* Select video input, video standard and tune here. */

  CLEAR(cropcap);

  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (0 == xioctl(fd_, VIDIOC_CROPCAP, &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */

    if (-1 == xioctl(fd_, VIDIOC_S_CROP, &crop)) {
      switch (errno) {
        case EINVAL:
          /* Cropping not supported. */
          break;
        default:
          /* Errors ignored. */
          break;
      }
    }
  } else {
    /* Errors ignored. */
  }

  CLEAR(fmt);

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = image_width;
  fmt.fmt.pix.height = image_height;
  fmt.fmt.pix.pixelformat = pixelformat_;
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

  if (-1 == xioctl(fd_, VIDIOC_S_FMT, &fmt)) {
    /* Check if selected format is already active - some hardware e.g. droidcam do not support setting values via VIDIOC_S_FMT*/
    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (xioctl(fd_, VIDIOC_G_FMT, &fmt) >= 0) {
      ROS_ERROR_STREAM(
          camera_dev_ << " does not support setting format options.");
      ROS_ERROR_STREAM(
          camera_dev_ << " supports: \n \t Width/Height \t : "<<fmt.fmt.pix.width<<"/"<<fmt.fmt.pix.height<<"\n"
                      <<"\t Pixel Format \t : "<<fcc2s(fmt.fmt.pix.pixelformat));

      if(fmt.fmt.pix.pixelformat == pixelformat_ &&
        fmt.fmt.pix.width == image_width &&
        fmt.fmt.pix.height == image_height) {
        ROS_ERROR_STREAM(
          "Selected format '"<< fcc2s(fmt.fmt.pix.pixelformat) <<"' is the same as the camera supports. Starting node...");
      } else {
        std::cerr << "error, quitting, TODO throw " << errno << std::endl;
        return false; // ("VIDIOC_S_FMT");
      }
    } else {
      std::cerr << "error, quitting, TODO throw " << errno << std::endl;
      return false;  // ("VIDIOC_S_FMT");
    }
  }

  /* Note VIDIOC_S_FMT may change width and height. */

  /* Buggy driver paranoia. */
  min = fmt.fmt.pix.width * 2;
  if (fmt.fmt.pix.bytesperline < min) {
    fmt.fmt.pix.bytesperline = min;
  }

  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage < min) {
    fmt.fmt.pix.sizeimage = min;
  }

  image_width = fmt.fmt.pix.width;
  image_height = fmt.fmt.pix.height;

  struct v4l2_streamparm stream_params;
  memset(&stream_params, 0, sizeof(stream_params));
  stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_G_PARM, &stream_params) < 0) {
    ROS_ERROR_STREAM("can't set stream params " << errno);
    return false;  // ("Couldn't query v4l fps!");
  }
  // ROS_ERROR_STREAM(
  // "Capability flag: 0x" << std::hex << stream_params.parm.capture.capability << std::dec);
  if (!(stream_params.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)) {
    ROS_ERROR("V4L2_CAP_TIMEPERFRAME not supported");
  }

  // TODO(lucasw) need to get list of valid numerator/denominator pairs
  // and match closest to what user put in.
  stream_params.parm.capture.timeperframe.numerator = 1;
  stream_params.parm.capture.timeperframe.denominator = framerate;
  if (xioctl(fd_, VIDIOC_S_PARM, &stream_params) < 0) {
    ROS_ERROR("Couldn't set camera framerate");
  } else {
    ROS_INFO_STREAM("Set framerate to be " << framerate);
  }

  switch (io_) {
    case IO_METHOD_READ:
      init_read(fmt.fmt.pix.sizeimage);
      break;

    case IO_METHOD_MMAP:
      init_mmap();
      break;

    case IO_METHOD_USERPTR:
      init_userp(fmt.fmt.pix.sizeimage);
      break;

    default:
      std::cerr << "Unknown io type " << io_ << std::endl;
      return false;
  }
  return true;
}

bool UsbCam::close_device(void)
{
  if (-1 == close(fd_)) {
    std::cerr << "error, quitting, TODO throw " << errno << std::endl;
    return false;  // ("close");
  }

  fd_ = -1;
  return true;
}

bool UsbCam::open_device(void)
{
  struct stat st;

  if (-1 == stat(camera_dev_.c_str(), &st)) {
    ROS_ERROR_STREAM(
      "Cannot identify '" << camera_dev_ << "': " << errno << ", " << strerror(errno));
    return false;  // (EXIT_FAILURE);
  }

  if (!S_ISCHR(st.st_mode)) {
    ROS_ERROR_STREAM(camera_dev_ << " is no device");
    return false;  // (EXIT_FAILURE);
  }

  fd_ = open(camera_dev_.c_str(), O_RDWR /* required */ | O_NONBLOCK, 0);

  if (-1 == fd_) {
    ROS_ERROR_STREAM(
      "Cannot open '" << camera_dev_ << "': " << errno << ", " << strerror(errno));
    return false;  // (EXIT_FAILURE);
  }
  return true;
}

bool UsbCam::start(
  const std::string & dev, io_method io_method, pixel_format pixel_format, color_format cf,
  uint32_t image_width, uint32_t image_height, int framerate)
{
  camera_dev_ = dev;

  io_ = io_method;
  monochrome_ = false;
  if (pixel_format == PIXEL_FORMAT_YUYV) {
    pixelformat_ = V4L2_PIX_FMT_YUYV;
  } else if (pixel_format == PIXEL_FORMAT_UYVY) {
    pixelformat_ = V4L2_PIX_FMT_UYVY;
  } else if (pixel_format == PIXEL_FORMAT_MJPEG) {
    pixelformat_ = V4L2_PIX_FMT_MJPEG;
    init_mjpeg_decoder(image_width, image_height, cf);
  } else if (pixel_format == PIXEL_FORMAT_H264) {
    pixelformat_ = V4L2_PIX_FMT_H264;
    init_h264_decoder(image_width, image_height, cf);
  } else if (pixel_format == PIXEL_FORMAT_YUVMONO10) {
    // actually format V4L2_PIX_FMT_Y16 (10-bit mono expresed as 16-bit pixels)
    // but we need to use the advertised type (yuyv)
    pixelformat_ = V4L2_PIX_FMT_YUYV;
    monochrome_ = true;
  } else if (pixel_format == PIXEL_FORMAT_RGB24) {
    pixelformat_ = V4L2_PIX_FMT_RGB24;
  } else if (pixel_format == PIXEL_FORMAT_GREY) {
    pixelformat_ = V4L2_PIX_FMT_GREY;
    monochrome_ = true;
  } else {
    ROS_ERROR("Unknown pixel format.");
    return false;  // (EXIT_FAILURE);
  }

  // TODO(lucasw) throw exceptions instead of return value checking
  if (!open_device()) {
    return false;
  }
  if (!init_device(image_width, image_height, framerate)) {
    return false;
  }
  if (!start_capturing()) {
    return false;
  }

  image_ = reinterpret_cast<camera_image_t *>(calloc(1, sizeof(camera_image_t)));

  image_->width = image_width;
  image_->height = image_height;
  image_->bytes_per_pixel = 3;  // corrected 11/10/15 (BYTES not BITS per pixel)

  image_->image_size = image_->width * image_->height * image_->bytes_per_pixel;
  image_->is_new = 0;
  image_->image = reinterpret_cast<char *>(calloc(image_->image_size, sizeof(char *)));
  memset(image_->image, 0, image_->image_size * sizeof(char *));
  return true;
}

bool UsbCam::shutdown(void)
{
  stop_capturing();
  uninit_device();
  close_device();

  if (avcodec_context_) {
    avcodec_close(avcodec_context_);
    av_free(avcodec_context_);
    avcodec_context_ = NULL;
  }
  if (avframe_camera_) {
    av_free(avframe_camera_);
  }
  avframe_camera_ = NULL;
  if (avframe_rgb_) {
    av_free(avframe_rgb_);
  }
  avframe_rgb_ = NULL;
  if (image_) {
    free(image_);
  }
  image_ = NULL;
  return true;
}

bool UsbCam::get_image(
  ros::Time & stamp,
  std::string & encoding, uint32_t & height, uint32_t & width,
  uint32_t & step, std::vector<uint8_t> & data)
{
  if ((image_->width == 0) || (image_->height == 0)) {
    return false;
  }
  // grab the image
  if (!grab_image()) {
    return false;
  }
  // TODO(lucasw) check if stamp is valid)
  // stamp the image
  stamp = image_->stamp;
  // fill in the info
  height = image_->height;
  width = image_->width;
  if (monochrome_) {
    encoding = "mono8";
    step = width;
  } else {
    // TODO(lucasw) aren't there other encoding types?
    encoding = "rgb8";
    step = width * 3;
  }
  // TODO(lucasw) create an Image here and already have the memory allocated,
  // eliminate this copy
  data.resize(step * height);
  memcpy(&data[0], image_->image, data.size());
  return true;
}

void UsbCam::get_formats()  // std::vector<usb_cam::msg::Format>& formats)
{
  ROS_INFO("This Cameras Supported Formats:");
  struct v4l2_fmtdesc fmt;
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.index = 0;
  for (fmt.index = 0; xioctl(fd_, VIDIOC_ENUM_FMT, &fmt) == 0; ++fmt.index) {
    ROS_INFO_STREAM(
      "  " << fmt.description << "[Index: " << fmt.index << ", Type: " << fmt.type <<
        ", Flags: " << fmt.flags << ", PixelFormat: " << std::hex << fmt.pixelformat << "]");

    struct v4l2_frmsizeenum size;
    size.index = 0;
    size.pixel_format = fmt.pixelformat;

    for (size.index = 0; xioctl(fd_, VIDIOC_ENUM_FRAMESIZES, &size) == 0; ++size.index) {
      ROS_INFO_STREAM(
        "  width: " << size.discrete.width << " x height: " << size.discrete.height);
      struct v4l2_frmivalenum interval;
      interval.index = 0;
      interval.pixel_format = size.pixel_format;
      interval.width = size.discrete.width;
      interval.height = size.discrete.height;
      for (interval.index = 0; xioctl(fd_, VIDIOC_ENUM_FRAMEINTERVALS, &interval) == 0;
        ++interval.index)
      {
        if (interval.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
          ROS_INFO_STREAM(
            "  " << interval.type << " " << interval.discrete.numerator << " / " <<
              interval.discrete.denominator);
        } else {
          ROS_INFO("other type");
        }
      }  // interval loop
    }  // size loop
  }  // fmt loop
}

bool UsbCam::grab_image()
{
  fd_set fds;
  struct timeval tv;
  int r;

  FD_ZERO(&fds);
  FD_SET(fd_, &fds);

  /* Timeout. */
  tv.tv_sec = 5;
  tv.tv_usec = 0;

  r = select(fd_ + 1, &fds, NULL, NULL, &tv);
  // if the v4l2_buffer timestamp isn't available use this time, though
  // it may be 10s of milliseconds after the frame acquisition.
  image_->stamp = ros::Time::now();

  if (-1 == r) {
    if (EINTR == errno) {
      return false;
    }

    ROS_ERROR_STREAM(
      "Something went wrong, exiting..." << errno);
    return false;  // ("select");
  }

  if (0 == r) {
    ROS_ERROR("select timeout");
    return false;
  }

  if (!read_frame()) {
    return false;
  }
  image_->is_new = 1;
  return true;
}

void UsbCam::query_ctrls(std::vector<ctrl>& ctrls) {
  struct v4l2_queryctrl queryctrl;

  memset(&queryctrl, 0, sizeof(queryctrl));
  queryctrl.id = V4L2_CTRL_CLASS_USER | V4L2_CTRL_FLAG_NEXT_CTRL;

  while (0 == xioctl(fd_, VIDIOC_QUERYCTRL, &queryctrl)) {
    if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
        continue;

    struct v4l2_control getctrl;
    getctrl.id = queryctrl.id;
    xioctl(fd_, VIDIOC_G_CTRL, &getctrl);

    ctrl c;
    c.id = queryctrl.id;
    c.value = getctrl.value;
    c.default_value = queryctrl.default_value;
    c.min_value = queryctrl.minimum;
    c.max_value = queryctrl.maximum;
    c.name = name2var(reinterpret_cast<char*>(queryctrl.name));

    if (queryctrl.type == V4L2_CTRL_TYPE_MENU) {
      struct v4l2_querymenu querymenu;
      memset(&querymenu, 0, sizeof(querymenu));
      querymenu.id = queryctrl.id;

      for (querymenu.index = queryctrl.minimum;
          querymenu.index <= queryctrl.maximum;
          querymenu.index++) {
        if (0 == xioctl(fd_, VIDIOC_QUERYMENU, &querymenu)) {
          c.menu[std::string(reinterpret_cast<char*>(querymenu.name))] = querymenu.index;
        }
      }
    }

    ctrls.push_back(c);

    queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
  }
}

void UsbCam::set_ctrl(int32_t id, int32_t value){
  struct v4l2_control setctrl;
  setctrl.id = id;
  setctrl.value = value;
  xioctl(fd_, VIDIOC_S_CTRL, &setctrl);
}

UsbCam::io_method UsbCam::io_method_from_string(const std::string & str)
{
  if (str == "mmap") {
    return IO_METHOD_MMAP;
  } else if (str == "read") {
    return IO_METHOD_READ;
  } else if (str == "userptr") {
    return IO_METHOD_USERPTR;
  } else {
    return IO_METHOD_UNKNOWN;
  }
}

UsbCam::pixel_format UsbCam::pixel_format_from_string(const std::string & str)
{
  if (str == "yuyv") {
    return PIXEL_FORMAT_YUYV;
  } else if (str == "uyvy") {
    return PIXEL_FORMAT_UYVY;
  } else if (str == "mjpeg") {
    return PIXEL_FORMAT_MJPEG;
  } else if (str == "h264") {
    return PIXEL_FORMAT_H264;
  } else if (str == "yuvmono10") {
    return PIXEL_FORMAT_YUVMONO10;
  } else if (str == "rgb24") {
    return PIXEL_FORMAT_RGB24;
  } else if (str == "grey") {
    return PIXEL_FORMAT_GREY;
  } else {
    return PIXEL_FORMAT_UNKNOWN;
  }
}
#if 0
std::string UsbCam::pixel_format_to_string(__u32 pixelformat)
{
  if (str == "yuyv") {
    return PIXEL_FORMAT_YUYV;
  } else if (str == "uyvy") {
    return PIXEL_FORMAT_UYVY;
  } else if (str == "mjpeg") {
    return PIXEL_FORMAT_MJPEG;
  } else if (str == "h264") {
    return PIXEL_FORMAT_H264;
  } else if (str == "yuvmono10") {
    return PIXEL_FORMAT_YUVMONO10;
  } else if (str == "rgb24") {
    return PIXEL_FORMAT_RGB24;
  } else if (str == "grey") {
    return PIXEL_FORMAT_GREY;
  } else {
    return PIXEL_FORMAT_UNKNOWN;
  }
}
#endif

UsbCam::color_format UsbCam::color_format_from_string(const std::string& str)
{
    if (str == "yuv420p")
      return COLOR_FORMAT_YUV420P;
    else if (str == "yuv422p")
      return COLOR_FORMAT_YUV422P;
    else
      return COLOR_FORMAT_UNKNOWN;
}

}  // namespace usb_cam